import sys
import numpy as np
import cv2
import serial
import time
import torch  # 🎯 新增：导入 PyTorch 核心库以开启无损推理加速
from numba import njit, prange  
from ultralytics import YOLO
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import CudaPacketPipeline

# ====================================================
# 【1. 定义 6x11 物理灯箱布局矩阵（同步自 HTML）】
# ====================================================
FRONT_GRID = [
    [1, 2, 3, 4, 5, 6, 7, 8, "a", 9, 10],
    [11, 12, 13, 14, "C", "C", "B", "B", "b", 15, 16],
    [17, 18, 19, 20, "C", "C", "B", "B", "c", 21, 22],
    ["F", "F", "E", "E", "D", "D", 23, 24, "A", "A", "d"],
    ["F", "F", "E", "E", "D", "D", 26, 27, "A", "A", 25],
    [28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38]
]

MERGED_BLOCKS = [
    {"row": 3, "col": 0, "row_span": 2, "col_span": 2}, # F
    {"row": 3, "col": 2, "row_span": 2, "col_span": 2}, # E
    {"row": 3, "col": 4, "row_span": 2, "col_span": 2}, # D
    {"row": 1, "col": 4, "row_span": 2, "col_span": 2}, # C
    {"row": 1, "col": 6, "row_span": 2, "col_span": 2}, # B
    {"row": 3, "col": 8, "row_span": 2, "col_span": 2}  # A
]

PURPLE_BOXES = {6, 7, 9, 13, 15, 19, 23, 26, 28, 29, 31, 33, 36, 37}

KEYS_ORDER = [
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38,
    "a", "b", "c", "d",
    "A", "B", "C", "D", "E", "F"
]

# HTML 配色定义 (转为 OpenCV 的 BGR 格式)
COLORS_BGR = {
    "blue": [216, 150, 70],      
    "yellow": [142, 232, 241],   
    "purple": [219, 93, 162]    
}

# ====================================================
# 【Numba 编译加速：生成整型索引矩阵】
# ====================================================
KEY_TO_INDEX = {key: i for i, key in enumerate(KEYS_ORDER)}
FRONT_GRID_INDEX = np.zeros((6, 11), dtype=np.int32)
for r in range(6):
    for c in range(11):
        FRONT_GRID_INDEX[r, c] = KEY_TO_INDEX[FRONT_GRID[r][c]]

# ====================================================
# 【🚀 Numba JIT 机器码级并行加速器】
# ====================================================
@njit(fastmath=True)
def accelerate_mapping(
    xs, ys, zs, 
    X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MAX,
    Z_REF, SCALE_STRENGTH, 
    LED_ROWS, LED_COLS, 
    FRONT_GRID_INDEX, 
    Z_BRIGHT_MIN, Z_MIN
):
    binary_grid = np.zeros((LED_ROWS, LED_COLS), dtype=np.uint8)
    depth_buffer = np.zeros((LED_ROWS, LED_COLS), dtype=np.float32)
    
    box_max_degrees = np.zeros(48, dtype=np.int32)
    box_counts = np.zeros(48, dtype=np.int32)
    
    n_points = len(xs)
    for i in range(n_points):
        x = xs[i]
        y = ys[i]
        z = zs[i]
        
        if z <= Z_MAX:
            z_clamped = max(z, 0.8)
            raw_scale = Z_REF / z_clamped
            scale = SCALE_STRENGTH * raw_scale + (1.0 - SCALE_STRENGTH)
            
            y_scaled = y * scale
            x_scaled = x * scale
            
            if (X_MIN <= x_scaled <= X_MAX) and (Y_MIN <= y_scaled <= Y_MAX):
                col = int((x_scaled - X_MIN) / (X_MAX - X_MIN) * (LED_COLS - 1))
                row = int((Y_MAX - y_scaled) / (Y_MAX - Y_MIN) * (LED_ROWS - 1))
                
                binary_grid[row, col] = 255
                depth_buffer[row, col] = z

                # 映射到 6x11 物理控制网格
                col_6x11 = int((x_scaled - X_MIN) / (X_MAX - X_MIN) * 11)
                row_6x11 = int((Y_MAX - y_scaled) / (Y_MAX - Y_MIN) * 6)
                
                if col_6x11 < 0: col_6x11 = 0
                if col_6x11 > 10: col_6x11 = 10
                if row_6x11 < 0: row_6x11 = 0
                if row_6x11 > 5: row_6x11 = 5

                box_idx = FRONT_GRID_INDEX[row_6x11, col_6x11]
                
                # 计算 255~0 程度值
                degree = 255.0 - (z - Z_BRIGHT_MIN) / (Z_MAX - Z_BRIGHT_MIN) * 255.0
                if degree < 0.0: degree = 0.0
                if degree > 255.0: degree = 255.0
                deg_int = int(degree)

                box_counts[box_idx] += 1
                if deg_int > box_max_degrees[box_idx]:
                    box_max_degrees[box_idx] = deg_int
                    
    return binary_grid, depth_buffer, box_max_degrees, box_counts

def is_servo_box(box_id):
    return isinstance(box_id, int)

def is_covered_by_merged(row, col):
    for b in MERGED_BLOCKS:
        if b["row"] <= row < b["row"] + b["row_span"] and b["col"] <= col < b["col"] + b["col_span"]:
            if row == b["row"] and col == b["col"]:
                return False
            return True
    return False

def get_merged_block(row, col):
    for b in MERGED_BLOCKS:
        if b["row"] == row and b["col"] == col:
            return b
    return None

def main():
    # 1. 启动硬件与 YOLO 模型
    print("正在加载 YOLOv11-Seg 实例分割模型 (首次运行会自动下载)...")
    model = YOLO("yolo11n-seg.pt")  

    pipeline = CudaPacketPipeline()
    freenect2 = Freenect2()
    if freenect2.enumerateDevices() == 0:
        print("未检测到 Kinect v2")
        sys.exit(1)

    device = freenect2.openDevice(freenect2.getDeviceSerialNumber(0), pipeline=pipeline)

    # 同时订阅彩色和深度
    listener = SyncMultiFrameListener(FrameType.Color | FrameType.Depth)
    device.setColorFrameListener(listener)
    device.setIrAndDepthFrameListener(listener)
    device.start()

    # 2. 获取参数并初始化对齐器
    color_params = device.getColorCameraParams()
    ir_params = device.getIrCameraParams()
    registration = Registration(ir_params, color_params)

    undistorted = Frame(512, 424, 4)
    registered = Frame(512, 424, 4)

    # ====================================================
    # 【空间物理参数 - 保留您的黄金微调参数】
    # ====================================================
    H = 2.0      
    theta = np.radians(30.0)  

    X_MIN, X_MAX = -1, 1  
    Y_MIN, Y_MAX = 0.0, 1.8     
    Z_MIN, Z_MAX = 1.3, 3.5

    # 亮度限制控制距离（2米内锁死 255）
    Z_BRIGHT_MIN = 2.0  

    LED_ROWS, LED_COLS = 64, 64  

    fx, fy = ir_params.fx, ir_params.fy
    cx, cy = ir_params.cx, ir_params.cy
    u_grid, v_grid = np.meshgrid(np.arange(512), np.arange(424))

    # 初始化串口
    ser = None
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200) 
        print("Teensy 串口已连接")
    except Exception as e:
        print(f"Teensy 串口未连接（本地预览调试开启）")

    # 滤波器、施密特、零延迟跳帧限频器初始化
    last_box_degrees = {k: 0.0 for k in KEYS_ORDER}
    box_active_states = {k: False for k in KEYS_ORDER}
    
    last_process_time = 0.0
    TARGET_FPS = 25  # 🎯 限制全局运行频率为 25 帧/秒

    print("\n✅ 驱动与 AI 模型就绪！正在进行实时高精度抠像映射...\n")

    try:
        while True:
            frames = listener.waitForNewFrame()

            # --- 零延迟跳帧逻辑（性能控制） ---
            current_time = time.time()
            if current_time - last_process_time < (1.0 / TARGET_FPS):
                listener.release(frames)
                continue
            last_process_time = current_time

            color_frame = frames[FrameType.Color]
            depth_frame = frames[FrameType.Depth]

            # 3. 将彩色图与深度图进行像素级对齐
            registration.apply(color_frame, depth_frame, undistorted, registered)
            color_aligned = registered.asarray(np.uint8)[:, :, :3]

            # 4. 【YOLO AI 抠像（NVIDIA 5080 显卡硬件级加速 + 内存优化）】
            # 🎯 核心优化：使用 with torch.inference_mode() 关闭梯度追踪，并在模型预测中启用 imgsz=320 降采样
            with torch.inference_mode():
                results = model.predict(color_aligned, classes=[0], verbose=False, device=0, imgsz=320)

            ai_mask = np.zeros((424, 512), dtype=np.uint8)

            if results[0].masks is not None:
                for mask_contour in results[0].masks.xy:
                    poly = np.array(mask_contour, dtype=np.int32)
                    cv2.fillPoly(ai_mask, [poly], 1)

            # 15x15 的大核闭运算
            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
            ai_mask = cv2.morphologyEx(ai_mask, cv2.MORPH_CLOSE, kernel_close)
            ai_mask_bool = ai_mask > 0

            # 5. 【3D 物理空间还原（使用无空洞原始深度 depth_frame）】
            Z_c = depth_frame.asarray(np.float32) / 1000.0 
            X_c = (u_grid - cx) * Z_c / fx
            Y_c = (v_grid - cy) * Z_c / fy

            Y_w = H - Y_c * np.cos(theta) - Z_c * np.sin(theta)
            Z_w = -Y_c * np.sin(theta) + Z_c * np.cos(theta)
            X_w = X_c

            # 6. 【高精度终极过滤】
            human_mask = ai_mask_bool & (Z_c > 0.1) & (Y_w > 0.15)

            # ====================================================
            # 【透视缩放参数】
            # ====================================================
            Z_REF = 1.2  
            SCALE_STRENGTH = 0.8  
            
            # 7. 【网格映射与数据提取（调用 Numba 并行加速器，直接拉满 100% 物理分辨率点云 [::1]）】
            xs = X_w[human_mask]
            ys = Y_w[human_mask]
            zs = Z_w[human_mask]

            # 🚀 机器码级执行！单核开销几乎归零
            binary_grid, depth_buffer, box_max_degrees, box_counts = accelerate_mapping(
                xs[::1], ys[::1], zs[::1], 
                X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MAX,
                Z_REF, SCALE_STRENGTH, 
                LED_ROWS, LED_COLS, 
                FRONT_GRID_INDEX, 
                Z_BRIGHT_MIN, Z_MIN
            )

            # ====================================================
            # 【物理灯箱计算：均值滤波 + 施密特双阈值迟滞防抖】
            # ====================================================
            box_degrees = {}
            for idx, key in enumerate(KEYS_ORDER):
                raw_max = box_max_degrees[idx]
                count = box_counts[idx]
                
                if count == 0:
                    box_degrees[key] = 0
                    box_active_states[key] = False
                    continue
                
                # 均值滤波计算
                avg_degree = raw_max  
                
                # 确定大/小灯箱的双门限
                is_large = isinstance(key, str) and key.isupper()
                high_thresh = 12.0 if is_large else 5.0  
                low_thresh = 5.0 if is_large else 2.0    
                
                was_active = box_active_states[key]
                
                if was_active:
                    is_active = (count >= low_thresh)
                else:
                    is_active = (count >= high_thresh)
                
                if is_active:
                    box_active_states[key] = True  
                    weight = np.clip(count / high_thresh, 0.0, 1.0)
                    box_degrees[key] = int(avg_degree * weight)
                else:
                    box_active_states[key] = False  
                    box_degrees[key] = 0

            # ====================================================
            # 【带 50 级变幅死区的非对称斜率限制器（绝杀频闪）】
            # ====================================================
            MAX_STEP_UP = 25    
            MAX_STEP_DOWN = 25  
            DEADBAND_LIMIT = 20  
            
            for key in KEYS_ORDER:
                target_val = box_degrees[key]
                current_val = last_box_degrees[key]
                
                diff = target_val - current_val
                
                # 变幅死区：剔除 50 级以内的小抖动，人离开时零值豁免允许安全熄灭
                if abs(diff) < DEADBAND_LIMIT and target_val > 0:
                    box_degrees[key] = int(round(current_val))
                    continue
                
                if diff > 0:
                    current_val += min(diff, MAX_STEP_UP)
                elif diff < 0:
                    current_val -= min(abs(diff), MAX_STEP_DOWN)
                    
                last_box_degrees[key] = current_val
                box_degrees[key] = int(round(current_val))

            # ========================================
            # 串口打包与发送 - 51字节二进制包
            # ========================================
            packet = bytearray()
            packet.append(0x41)  # 'A'
            packet.append(0x57)  # 'W'
            for key in KEYS_ORDER:
                packet.append(box_degrees[key])
            packet.append(sum(packet) % 256)

            if ser is not None:
                ser.write(packet)

            # 控制台输出防刷屏，只打印激活的格子
            active_list = {str(k): v for k, v in box_degrees.items() if v > 0}
            print(f"\r【活性格子】: {active_list}".ljust(100), end="")

            # 区分不同的人
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_grid)
            led_preview = np.zeros((LED_ROWS, LED_COLS, 3), dtype=np.uint8)

            for i in range(1, num_labels):
                if stats[i, cv2.CC_STAT_AREA] < 5:  
                    continue

                person_mask = (labels == i)
                active_pixels = person_mask & (depth_buffer > 0)
                pixel_depths = depth_buffer[active_pixels]

                brightness_values = 255 - (pixel_depths - Z_BRIGHT_MIN) / (Z_MAX - Z_BRIGHT_MIN) * 255
                brightness_values = np.clip(brightness_values, 50, 255).astype(np.uint8)

                led_preview[active_pixels, 2] = brightness_values

            # 8. 预览渲染 (双屏拼接)
            debug_view = cv2.resize(led_preview, (512, 512), interpolation=cv2.INTER_NEAREST)
            preview_color = color_aligned.copy()
            if np.any(ai_mask_bool):  
                preview_color[ai_mask_bool] = (preview_color[ai_mask_bool] * 0.5 + np.array([0, 255, 0]) * 0.5).astype(np.uint8)
            preview_color_resized = cv2.resize(preview_color, (512, 512))
            
            combined_view = np.hstack((debug_view, preview_color_resized))
            cv2.imshow("LED Matrix Live Preview & AI Vision", combined_view)

            # ====================================================
            # 【渲染物理灯箱模拟器窗口 (6x11 大小格子) 】
            # ====================================================
            CELL_SIZE = 70
            phys_view = np.zeros((420, 770, 3), dtype=np.uint8)

            for r in range(6):
                for c in range(11):
                    if is_covered_by_merged(r, c):
                        continue

                    merged = get_merged_block(r, c)
                    row_span = merged["row_span"] if merged else 1
                    col_span = merged["col_span"] if merged else 1

                    box_id = FRONT_GRID[r][c]
                    deg = box_degrees[box_id]

                    if is_servo_box(box_id):
                        color_name = "purple" if box_id in PURPLE_BOXES else "yellow"
                    else:
                        color_name = "blue"

                    intensity_factor = deg / 255.0
                    base_lum = 27
                    base_color = COLORS_BGR[color_name]
                    
                    b = int(base_lum + intensity_factor * (base_color[0] - base_lum))
                    g = int(base_lum + intensity_factor * (base_color[1] - base_lum))
                    r_val = int(base_lum + intensity_factor * (base_color[2] - base_lum))
                    cell_color = (b, g, r_val)

                    x1 = c * CELL_SIZE
                    y1 = r * CELL_SIZE
                    x2 = (c + col_span) * CELL_SIZE
                    y2 = (r + row_span) * CELL_SIZE

                    cv2.rectangle(phys_view, (x1, y1), (x2, y2), cell_color, -1)
                    cv2.rectangle(phys_view, (x1, y1), (x2, y2), (40, 40, 40), 1)

                    text = str(box_id)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 1.0 if col_span > 1 else 0.6
                    thickness = 2 if col_span > 1 else 1
                    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
                    text_x = x1 + (x2 - x1 - text_size[0]) // 2
                    text_y = y1 + (y2 - y1 + text_size[1]) // 2
                    
                    cv2.putText(phys_view, text, (text_x + 1, text_y + 1), font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
                    cv2.putText(phys_view, text, (text_x, text_y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

            cv2.imshow("Physical Grid Preview (6x11)", phys_view)

            listener.release(frames)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n") # 换行收尾
                break

    finally:
        device.stop()
        device.close()
        cv2.destroyAllWindows()
        print("已安全关闭 Kinect 设备")

if __name__ == "__main__":
    main()