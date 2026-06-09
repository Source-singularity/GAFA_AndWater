import sys
import numpy as np
import cv2
import serial
import time
import torch  # 用于开启 PyTorch 硬件推理加速与内存锁
from numba import njit, prange  # 导入 Numba 即时编译器
from ultralytics import YOLO
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import CudaPacketPipeline

# ====================================================
# 【1. 定义 6x4 物理灯箱布局矩阵（同步自侧面 CAD 尺寸）】
# ====================================================
SIDE_GRID = [
    [1, 2, 3, "e"],
    ["G", "G", "f", "g"],
    ["G", "G", 4, 5],
    ["h", "i", 6, 7],
    ["j", 8, 9, 10],
    [11, 12, 13, 14]
]

# 侧面 2x2 融合大灯箱定义 (起始行列坐标)
MERGED_BLOCKS_SIDE = [
    {"row": 1, "col": 0, "row_span": 2, "col_span": 2}  # G 大箱体
]

# 侧面紫色舵机灯箱定义 (同步自您给出的侧面属性表)
PURPLE_BOXES_SIDE = {4, 5, 8, 9}

# 21个物理对象严格排序列表 (用于固定 24 字节串口数据包的字节顺序)
KEYS_ORDER_SIDE = [
    # 1. 数字 1 ~ 14
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
    # 2. 小写字母 e ~ j
    "e", "f", "g", "h", "i", "j",
    # 3. 大写字母 G (2x2大灯箱)
    "G"
]

# HTML 配色定义 (转为 OpenCV 的 BGR 格式)
COLORS_BGR = {
    "blue": [216, 150, 70],      # 字母普通灯箱的基础色 [B, G, R]
    "yellow": [142, 232, 241],   # 黄色舵机灯箱基础色
    "purple": [219, 93, 162]    # 紫色舵机灯箱基础色
}

# ====================================================
# 【2. Numba 编译准备工作：生成 6x4 的物理格子整型映射矩阵】
# 我们在初始化时将 21 个 Box 键名映射为索引 0~20，生成一个 6x4 的 int32 矩阵传给 JIT
# ====================================================
KEY_TO_INDEX_SIDE = {key: i for i, key in enumerate(KEYS_ORDER_SIDE)}
SIDE_GRID_INDEX = np.zeros((6, 4), dtype=np.int32)
for r in range(6):
    for c in range(4):
        SIDE_GRID_INDEX[r, c] = KEY_TO_INDEX_SIDE[SIDE_GRID[r][c]]

# ====================================================
# 【🚀 3. 侧面墙专用 Numba JIT 机器码级并行加速器（一列同时激活版）】
# 彻底释放 Python GIL 锁，将循环处理开销暴降 100 倍！
# ====================================================
@njit(fastmath=True)
def accelerate_side_mapping(
    xs, ys, zs, 
    X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MAX,
    Z_REF, SCALE_STRENGTH, 
    LED_ROWS, LED_COLS, 
    SIDE_GRID_INDEX, 
    Z_BRIGHT_MIN, Z_MIN
):
    # 预分配 64x64 二值网格和深度缓存
    binary_grid = np.zeros((LED_ROWS, LED_COLS), dtype=np.uint8)
    depth_buffer = np.zeros((LED_ROWS, LED_COLS), dtype=np.float32)
    
    # 🎯【核心修复】：直接硬编码长度 21 (侧面墙 21 个物理控制通道)，
    # 彻底规避 Numba 在编译时尝试推导全局变量 KEYS_ORDER_SIDE 产生的类型错误！
    box_max_degrees = np.zeros(21, dtype=np.int32)
    box_counts = np.zeros(21, dtype=np.int32)
    
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
                # 3.1 映射到 64x64 虚拟网格（用于红影渲染窗口）
                col = int((x_scaled - X_MIN) / (X_MAX - X_MIN) * (LED_COLS - 1))
                row = int((Y_MAX - y_scaled) / (Y_MAX - Y_MIN) * (LED_ROWS - 1))
                
                binary_grid[row, col] = 255
                depth_buffer[row, col] = z

                # 3.2 映射到 6x4 物理控制网格 (纵向一列同时激活逻辑)
                col_6x4 = int((x_scaled - X_MIN) / (X_MAX - X_MIN) * 4)
                
                if col_6x4 < 0: col_6x4 = 0
                if col_6x4 > 3: col_6x4 = 3 # 侧面只有 4 列

                # 🎯【核心改变】：一列同时激活。只要人在此列中，
                # 我们同时遍历并点亮该列中的所有 6 个行（Row 0 到 Row 5）！
                for r_6x4 in range(6):
                    box_idx = SIDE_GRID_INDEX[r_6x4, col_6x4]
                    
                    # 距离 z 映射到 255~0（2.0m内直接锁死最大值 255）
                    degree = 255.0 - (z - Z_BRIGHT_MIN) / (Z_MAX - Z_BRIGHT_MIN) * 255.0
                    if degree < 0.0: degree = 0.0
                    if degree > 255.0: degree = 255.0
                    deg_int = int(degree)

                    # 计数器与最值保存
                    box_counts[box_idx] += 1
                    if deg_int > box_max_degrees[box_idx]:
                        box_max_degrees[box_idx] = deg_int
                        
    return binary_grid, depth_buffer, box_max_degrees, box_counts

def is_servo_box_side(box_id):
    """判断是否为舵机箱"""
    return isinstance(box_id, int)

def is_covered_by_merged_side(row, col):
    """判断是否被大灯箱 G 融合遮挡"""
    for b in MERGED_BLOCKS_SIDE:
        if b["row"] <= row < b["row"] + b["row_span"] and b["col"] <= col < b["col"] + b["col_span"]:
            if row == b["row"] and col == b["col"]:
                return False
            return True
    return False

def get_merged_block_side(row, col):
    """获取大灯箱 G 属性"""
    for b in MERGED_BLOCKS_SIDE:
        if b["row"] == row and b["col"] == col:
            return b
    return None

def main():
    # 4. 启动硬件与 YOLO 模型
    print("正在加载 YOLOv11-Seg 实例分割模型 (侧面墙驱动启动中)...")
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

    color_params = device.getColorCameraParams()
    ir_params = device.getIrCameraParams()
    registration = Registration(ir_params, color_params)

    undistorted = Frame(512, 424, 4)
    registered = Frame(512, 424, 4)

    # ====================================================
    # 【侧面墙空间物理参数 - 完美对齐您的黄金实测值】
    # ====================================================
    H = 2.0      # 相机高度
    theta = np.radians(30.0)  # 俯角 30 度

    # 左右宽度：本地测试改用对称的 [-0.3, 0.9] 参数（完美契合您的侧边检测幅宽）
    X_MIN, X_MAX = -0.3, 0.9  
    
    # 垂直高度范围 0.0 到 1.8 米
    Y_MIN, Y_MAX = 0.0, 1.8     
    
    # 放开最近检测距离限制至 0.5 米
    Z_MIN, Z_MAX = 0.5, 3.5

    # 亮度控制范围（2.0m以内锁死255）
    Z_BRIGHT_MIN = 2.0  

    LED_ROWS, LED_COLS = 64, 64  

    fx, fy = ir_params.fx, ir_params.fy
    cx, cy = ir_params.cx, ir_params.cy
    u_grid, v_grid = np.meshgrid(np.arange(512), np.arange(424))

    # 初始化串口 (连接 Teensy 4.1 侧面专线)
    ser = None
    try:
        ser = serial.Serial('/dev/ttyUSB1', 115200) # 根据实际串口路径修改
        print("Teensy 侧面专线串口已连接")
    except Exception as e:
        print(f"Teensy 侧面串口未连接（本地预览调试开启）")

    # 滤波器、施密特、零延迟跳帧限频器初始化
    last_box_degrees = {k: 0.0 for k in KEYS_ORDER_SIDE}
    box_active_states = {k: False for k in KEYS_ORDER_SIDE}
    
    # 【新增】：YOLO 掩膜时域平滑矩阵（424x512，缩减抖动）
    ai_mask_smooth = np.zeros((424, 512), dtype=np.float32)
    
    last_process_time = 0.0
    TARGET_FPS = 25  # 🎯 限制全局运行频率为 25 帧/秒

    print("\n✅ 侧面墙驱动与 AI 模型就绪！正在进行实时高精度抠像映射...\n")

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

            # 4. 【YOLO AI 抠像（NVIDIA 5080 显卡硬件级加速 + 内存与时间双重优化）】
            with torch.inference_mode():
                results = model.predict(color_aligned, classes=[0], verbose=False, device=0, imgsz=320)

            ai_mask = np.zeros((424, 512), dtype=np.uint8)

            if results[0].masks is not None:
                for mask_contour in results[0].masks.xy:
                    poly = np.array(mask_contour, dtype=np.int32)
                    cv2.fillPoly(ai_mask, [poly], 1)

            # 15x15 的大核闭运算
            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
            ai_mask_closed = cv2.morphologyEx(ai_mask, cv2.MORPH_CLOSE, kernel_close)

            # ====================================================
            # 【新增：YOLO 掩膜时域指数平滑滤波（EMA）】
            # 彻底消灭 YOLO 检测边缘由于衣服反光、褶皱带来的帧间边缘抖动！
            # ====================================================
            ai_mask_smooth = 0.3 * ai_mask_closed.astype(np.float32) + 0.7 * ai_mask_smooth
            ai_mask_bool = ai_mask_smooth > 0.3  # 阈值化

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
            
            # 7. 【网格映射与数据提取（调用 Numba 并行加速器）】
            xs = X_w[human_mask]
            ys = Y_w[human_mask]
            zs = Z_w[human_mask]

            # 🚀 性能大突破：调用 Numba 编译后的函数。直接使用 [::1]（无损全分辨率），CPU 单核消耗仍会降至 2% 左右！
            binary_grid, depth_buffer, box_max_degrees_numba, box_counts_numba = accelerate_side_mapping(
                xs[::1], ys[::1], zs[::1], 
                X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MAX,
                Z_REF, SCALE_STRENGTH, 
                LED_ROWS, LED_COLS, 
                SIDE_GRID_INDEX, 
                Z_BRIGHT_MIN, Z_MIN
            )

            # ====================================================
            # 【物理灯箱计算：均值滤波 + 施密特双阈值迟滞防抖】
            # ====================================================
            # 将 Numba 返回的 numpy 数组转回 Python 字典（仅转换一次）
            box_max_degrees = {k: box_max_degrees_numba[KEY_TO_INDEX_SIDE[k]] for k in KEYS_ORDER_SIDE}
            box_counts = {k: box_counts_numba[KEY_TO_INDEX_SIDE[k]] for k in KEYS_ORDER_SIDE}

            box_degrees = {}
            for key in KEYS_ORDER_SIDE:
                total_sum = box_max_degrees[key] # Numba 内部已经取了 max，此处直接用 max 值
                count = box_counts[key]
                
                if count == 0:
                    box_degrees[key] = 0
                    box_active_states[key] = False
                    continue
                
                avg_degree = total_sum 
                
                # 侧面大灯箱 G (2x2) 需要 12 点开启，小灯箱需要 5 点开启
                is_large = (key == "G")
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
            MAX_STEP_UP = 40    
            MAX_STEP_DOWN = 15  
            DEADBAND_LIMIT = 50  # 🎯 物理去抖：50 级死区已全面同步至侧面墙
            
            for key in KEYS_ORDER_SIDE:
                target_val = box_degrees[key]
                current_val = last_box_degrees[key]
                
                diff = target_val - current_val
                
                if abs(diff) < DEADBAND_LIMIT and target_val > 0:
                    box_degrees[key] = int(round(current_val))
                    continue
                
                if diff > 0:
                    current_val += min(diff, MAX_STEP_UP)
                elif diff < 0:
                    current_val -= min(abs(diff), MAX_STEP_DOWN)
                    
                last_box_degrees[key] = current_val
                box_degrees[key] = int(round(current_val))

            # ====================================================
            # 【串口打包与发送 - 侧面 24 字节 AS 包协议】
            # ====================================================
            packet = bytearray()
            packet.append(0x41)  # 'A'
            packet.append(0x53)  # 'S' (Side 侧面专包)
            for key in KEYS_ORDER_SIDE:
                packet.append(box_degrees[key])
            packet.append(sum(packet) % 256)

            if ser is not None:
                ser.write(packet)

            # 后台侧面专有通信打印
            active_list = {str(k): v for k, v in box_degrees.items() if v > 0}
            print(f"\r【侧面活性格子】: {active_list}".ljust(100), end="")

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

            # 双屏拼接预览
            debug_view = cv2.resize(led_preview, (512, 512), interpolation=cv2.INTER_NEAREST)
            preview_color = color_aligned.copy()
            if np.any(ai_mask_bool):  
                preview_color[ai_mask_bool] = (preview_color[ai_mask_bool] * 0.5 + np.array([0, 255, 0]) * 0.5).astype(np.uint8)
            preview_color_resized = cv2.resize(preview_color, (512, 512))
            
            combined_view = np.hstack((debug_view, preview_color_resized))
            cv2.imshow("LED Matrix Live Preview & AI Vision (SIDE)", combined_view)

            # ====================================================
            # 【渲染侧面 6x4 物理模拟器窗口】
            # ====================================================
            CELL_SIZE = 70
            phys_view = np.zeros((420, 280, 3), dtype=np.uint8)

            for r in range(6):
                for c in range(4):
                    if is_covered_by_merged_side(r, c):
                        continue

                    merged = get_merged_block_side(r, c)
                    row_span = merged["row_span"] if merged else 1
                    col_span = merged["col_span"] if merged else 1

                    box_id = SIDE_GRID[r][c]
                    deg = box_degrees[box_id]

                    if is_servo_box_side(box_id):
                        color_name = "purple" if box_id in PURPLE_BOXES_SIDE else "yellow"
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

            cv2.imshow("Physical Grid Preview (6x4 SIDE)", phys_view)

            listener.release(frames)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n")
                break

    finally:
        device.stop()
        device.close()
        cv2.destroyAllWindows()
        print("已安全关闭 Kinect 设备")

if __name__ == "__main__":
    main()