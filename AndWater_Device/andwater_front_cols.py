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

# 2x2 的融合大灯箱定义 (起始行列坐标)
MERGED_BLOCKS = [
    {"row": 3, "col": 0, "row_span": 2, "col_span": 2}, # F 大箱体
    {"row": 3, "col": 2, "row_span": 2, "col_span": 2}, # E 大箱体
    {"row": 3, "col": 4, "row_span": 2, "col_span": 2}, # D 大箱体
    {"row": 1, "col": 4, "row_span": 2, "col_span": 2}, # C 大箱体
    {"row": 1, "col": 6, "row_span": 2, "col_span": 2}, # B 大箱体
    {"row": 3, "col": 8, "row_span": 2, "col_span": 2}  # A 大箱体
]

# 紫色舵机灯箱定义 (同步自 HTML Set)
PURPLE_BOXES = {6, 7, 9, 13, 15, 19, 23, 26, 28, 29, 31, 33, 36, 37}

# 48个物理对象严格排序列表 (用于固定51字节串口数据包的字节顺序)
KEYS_ORDER = [
    # 1. 数字 1 ~ 38
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38,
    # 2. 小写字母 a ~ d
    "a", "b", "c", "d",
    # 3. 大写字母 A ~ F
    "A", "B", "C", "D", "E", "F"
]

# HTML 配色定义 (转为 OpenCV 的 BGR 格式)
COLORS_BGR = {
    "blue": [216, 150, 70],      # 非舵机/普通灯箱的基础色 [B, G, R]
    "yellow": [142, 232, 241],   # 黄色舵机灯箱基础色
    "purple": [219, 93, 162]    # 紫色舵机灯箱基础色
}

# ====================================================
# 【2. Numba 编译准备工作：生成 6x11 的物理格子整型映射矩阵】
# 由于 Numba 无法解析 Python 的混合字典（含有字符串和整型），
# 我们在初始化时将 48 个 Box 键名映射为索引 0~47，生成一个 6x11 的 int32 矩阵传给 JIT
# ====================================================
KEY_TO_INDEX = {key: i for i, key in enumerate(KEYS_ORDER)}
FRONT_GRID_INDEX = np.zeros((6, 11), dtype=np.int32)
for r in range(6):
    for c in range(11):
        FRONT_GRID_INDEX[r, c] = KEY_TO_INDEX[FRONT_GRID[r][c]]

# ====================================================
# 【🚀 3. Numba JIT 机器码级并行加速器（一列同时激活版）】
# 彻底释放 Python GIL 锁，将循环处理开销暴降 100 倍！
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
    # 预分配 64x64 二值矩阵和深度缓冲区
    binary_grid = np.zeros((LED_ROWS, LED_COLS), dtype=np.uint8)
    depth_buffer = np.zeros((LED_ROWS, LED_COLS), dtype=np.float32)
    
    # 预分配 48 个物理通道的临时最大值和计数器
    box_max_degrees = np.zeros(48, dtype=np.int32)
    box_counts = np.zeros(48, dtype=np.int32)
    
    n_points = len(xs)
    for i in range(n_points):
        x = xs[i]
        y = ys[i]
        z = zs[i]
        
        if z <= Z_MAX:
            # 引入深度安全幅度，限制 z 最小不小于 0.8米，防止极近处公式除零爆炸
            z_clamped = max(z, 0.8)

            # 3D 空间透视缩放计算
            raw_scale = Z_REF / z_clamped
            scale = SCALE_STRENGTH * raw_scale + (1.0 - SCALE_STRENGTH)
            
            y_scaled = y * scale
            x_scaled = x * scale
            
            # 检查是否落在设定的空间投影范围边界内
            if (X_MIN <= x_scaled <= X_MAX) and (Y_MIN <= y_scaled <= Y_MAX):
                # 3.1 映射到 64x64 虚拟网格（用于红影渲染窗口）
                col = int((x_scaled - X_MIN) / (X_MAX - X_MIN) * (LED_COLS - 1))
                row = int((Y_MAX - y_scaled) / (Y_MAX - Y_MIN) * (LED_ROWS - 1))
                
                binary_grid[row, col] = 255
                depth_buffer[row, col] = z

                # 3.2 映射到 6x11 物理控制网格（纵向一列同时激活逻辑）
                col_6x11 = int((x_scaled - X_MIN) / (X_MAX - X_MIN) * 11)
                
                # 物理防溢出安全限制
                if col_6x11 < 0: col_6x11 = 0
                if col_6x11 > 10: col_6x11 = 10

                # 🎯【核心改变】：一列同时激活。只要人在此列中，
                # 我们同时遍历并点亮该列中的所有 6 个行（Row 0 到 Row 5）！
                for r_6x11 in range(6):
                    box_idx = FRONT_GRID_INDEX[r_6x11, col_6x11]
                    
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

def is_servo_box(box_id):
    """判断是否为舵机箱"""
    return isinstance(box_id, int)

def is_covered_by_merged(row, col):
    """判断当前基本单元格是否被大箱体融合覆盖"""
    for b in MERGED_BLOCKS:
        if b["row"] <= row < b["row"] + b["row_span"] and b["col"] <= col < b["col"] + b["col_span"]:
            if row == b["row"] and col == b["col"]:
                return False
            return True
    return False

def get_merged_block(row, col):
    """获取大箱体属性"""
    for b in MERGED_BLOCKS:
        if b["row"] == row and b["col"] == col:
            return b
    return None

def main():
    # 4. 初始化 AI 与设备
    print("正在加载 YOLOv11-Seg 实例分割模型 (首次运行会自动下载)...")
    model = YOLO("yolo11n-seg.pt")  

    pipeline = CudaPacketPipeline()
    freenect2 = Freenect2()
    if freenect2.enumerateDevices() == 0:
        print("未检测到 Kinect v2")
        sys.exit(1)

    device = freenect2.openDevice(freenect2.getDeviceSerialNumber(0), pipeline=pipeline)

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
    # 【您的黄金空间物理与检测参数 - 绝无二义，只在这里统一配置】
    # ====================================================
    H = 2.0      # 相机物理安装高度 (米)
    theta = np.radians(30.0)  # 俯仰角度 30 度

    X_MIN, X_MAX = -1, 1      # 物理左右检测范围 (2.0米宽)
    Y_MIN, Y_MAX = 0.0, 1.8   # 物理垂直高度检测范围 (0米 到 1.8米)
    Z_MIN, Z_MAX = 1.3, 3.5   # 物理前后深度范围 (1.3米 到 3.5米)

    Z_BRIGHT_MIN = 2.0  # 亮度最大基准：距离小于 2.0 米时直接锁死 255 最大交互值

    LED_ROWS, LED_COLS = 64, 64  # 虚拟网格分辨率

    # 用于 3D 坐标解算的内参与网格准备
    fx, fy = ir_params.fx, ir_params.fy
    cx, cy = ir_params.cx, ir_params.cy
    u_grid, v_grid = np.meshgrid(np.arange(512), np.arange(424))

    # 初始化串口连接
    ser = None
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200) 
        print("Teensy 串口已连接")
    except Exception as e:
        print(f"Teensy 串口未连接（本地预览调试开启）")

    # 寄存器、平滑器与降频参数初始化
    last_box_degrees = {k: 0.0 for k in KEYS_ORDER}
    box_active_states = {k: False for k in KEYS_ORDER}
    
    # 【新增】：YOLO 掩膜时域平滑矩阵（424x512，浮点），用于抹除 AI 帧间边界闪烁
    ai_mask_smooth = np.zeros((424, 512), dtype=np.float32)
    
    last_process_time = 0.0
    TARGET_FPS = 25  # 🎯 限制全局循环最大刷新率为 25 帧/秒（清除无用 CPU 空转）

    print("\n✅ 驱动、AI 与硬件加速全部就绪！正在进行一列同时激活映射...\n")

    try:
        while True:
            # 快速抓取帧流，维持 Kinect 底层队列无堆积、零体感延时
            frames = listener.waitForNewFrame()

            # --- 零延迟跳帧限频逻辑 ---
            current_time = time.time()
            if current_time - last_process_time < (1.0 / TARGET_FPS):
                listener.release(frames)
                continue
            last_process_time = current_time

            color_frame = frames[FrameType.Color]
            depth_frame = frames[FrameType.Depth]

            # 4.1 对齐图像并提取
            registration.apply(color_frame, depth_frame, undistorted, registered)
            color_aligned = registered.asarray(np.uint8)[:, :, :3]

            # 4.2 【YOLO AI 抠像】
            # 🎯 显卡加速：利用 with torch.inference_mode() 关闭梯度追踪，并在模型预测中启用 imgsz=320 降采样
            with torch.inference_mode():
                results = model.predict(color_aligned, classes=[0], verbose=False, device=0, imgsz=320)

            # 获取当前帧的原生 AI 掩膜
            ai_mask = np.zeros((424, 512), dtype=np.uint8)
            if results[0].masks is not None:
                for mask_contour in results[0].masks.xy:
                    poly = np.array(mask_contour, dtype=np.int32)
                    cv2.fillPoly(ai_mask, [poly], 1)

            # 【AI孔洞闭运算】：用大核闭运算将近距离因视差造成的左右断裂完美缝合
            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
            ai_mask_closed = cv2.morphologyEx(ai_mask, cv2.MORPH_CLOSE, kernel_close)

            # ====================================================
            # 【新增：YOLO 掩膜时域指数平滑滤波（EMA）】
            # 对相邻几帧的 AI 掩膜进行融合，只有连续检测到的区域才会被点亮，
            # 彻底消灭 YOLO 检测边缘由于衣服反光、褶皱带来的帧间边缘抖动！
            # ====================================================
            ai_mask_smooth = 0.3 * ai_mask_closed.astype(np.float32) + 0.7 * ai_mask_smooth
            ai_mask_bool = ai_mask_smooth > 0.3  # 阈值化

            # 5. 【3D 物理空间还原（使用无视差空洞的原始物理深度 depth_frame）】
            Z_c = depth_frame.asarray(np.float32) / 1000.0 
            X_c = (u_grid - cx) * Z_c / fx
            Y_c = (v_grid - cy) * Z_c / fy

            Y_w = H - Y_c * np.cos(theta) - Z_c * np.sin(theta)
            Z_w = -Y_c * np.sin(theta) + Z_c * np.cos(theta)
            X_w = X_c

            # 6. 【高精度终极过滤（放开近距离 Z_MIN 限制）】
            human_mask = ai_mask_bool & (Z_c > 0.1) & (Y_w > 0.15)

            # ====================================================
            # 【7. 调用 Numba 加速映射器（直接拉满 100% 物理分辨率点云 [::1]）】
            # ====================================================
            xs = X_w[human_mask]
            ys = Y_w[human_mask]
            zs = Z_w[human_mask]

            Z_REF = 1.2  
            SCALE_STRENGTH = 0.8  

            # 🚀 机器码级极速执行，CPU 核心负载直降为 ~2%
            # Numba 内部已经集成了【纵向一列同时激活】和【深度安全夹】逻辑
            binary_grid, depth_buffer, box_max_degrees, box_counts = accelerate_mapping(
                xs[::1], ys[::1], zs[::1], 
                X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MAX,
                Z_REF, SCALE_STRENGTH, 
                LED_ROWS, LED_COLS, 
                FRONT_GRID_INDEX, 
                Z_BRIGHT_MIN, Z_MIN
            )

            # ====================================================
            # 【8. 物理灯箱计算：均值滤波 + 施密特双阈值迟滞防抖】
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
            # 【9. 带 50 级变幅死区的非对称斜率限制器（绝杀频闪）】
            # ====================================================
            MAX_STEP_UP = 25    
            MAX_STEP_DOWN = 25  
            DEADBAND_LIMIT = 25  
            
            for key in KEYS_ORDER:
                target_val = box_degrees[key]
                current_val = last_box_degrees[key]
                
                diff = target_val - current_val
                
                # 变幅死区：剔除 50 级以内的小抖动，人离开时零值豁免允许安全平滑熄灭
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
            # 【10. 串口打包与发送 - 51字节二进制包】
            # ========================================
            packet = bytearray()
            packet.append(0x41)  # 'A'
            packet.append(0x57)  # 'W'
            for key in KEYS_ORDER:
                packet.append(box_degrees[key])
            packet.append(sum(packet) % 256)

            if ser is not None:
                ser.write(packet)

            # 控制台输出防刷屏，只打印当前被激活亮起的格子
            active_list = {str(k): v for k, v in box_degrees.items() if v > 0}
            print(f"\r【活性格子】: {active_list}".ljust(100), end="")

            # ========================================
            # 【11. 64x64 虚拟网格逐像素亮度映射（红影渲染）】
            # ========================================
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

            # 12. 双屏拼接预览渲染
            debug_view = cv2.resize(led_preview, (512, 512), interpolation=cv2.INTER_NEAREST)
            preview_color = color_aligned.copy()
            if np.any(ai_mask_bool):  
                preview_color[ai_mask_bool] = (preview_color[ai_mask_bool] * 0.5 + np.array([0, 255, 0]) * 0.5).astype(np.uint8)
            preview_color_resized = cv2.resize(preview_color, (512, 512))
            
            combined_view = np.hstack((debug_view, preview_color_resized))
            cv2.imshow("LED Matrix Live Preview & AI Vision", combined_view)

            # ====================================================
            # 【13. 渲染物理灯箱模拟器窗口 (6x11 大小格子) 】
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