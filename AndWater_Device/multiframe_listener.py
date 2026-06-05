import sys
import numpy as np
import cv2
import serial
import time  # 新增：导入时间库，用于跳帧限速
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
    {"row": 3, "col": 0, "row_span": 2, "col_span": 2}, # F
    {"row": 3, "col": 2, "row_span": 2, "col_span": 2}, # E
    {"row": 3, "col": 4, "row_span": 2, "col_span": 2}, # D
    {"row": 1, "col": 4, "row_span": 2, "col_span": 2}, # C
    {"row": 1, "col": 6, "row_span": 2, "col_span": 2}, # B
    {"row": 3, "col": 8, "row_span": 2, "col_span": 2}  # A
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

def is_servo_box(box_id):
    """判断该对象是否为舵机箱（数字代表舵机箱，字母代表普通大/小灯箱）"""
    return isinstance(box_id, int)

def is_covered_by_merged(row, col):
    """判断当前网格是否被大灯箱覆盖（非大灯箱的左上角主点）"""
    for b in MERGED_BLOCKS:
        if b["row"] <= row < b["row"] + b["row_span"] and b["col"] <= col < b["col"] + b["col_span"]:
            if row == b["row"] and col == b["col"]:
                return False  # 左上角主点不视为被覆盖遮挡
            return True
    return False

def get_merged_block(row, col):
    """获取起始于 (row, col) 的融合大灯箱信息"""
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

    # 必须同时订阅彩色和深度，因为 YOLO 需要彩色，物理坐标需要深度
    listener = SyncMultiFrameListener(FrameType.Color | FrameType.Depth)
    device.setColorFrameListener(listener)
    device.setIrAndDepthFrameListener(listener)
    device.start()

    # 2. 获取参数并初始化对齐器 (Registration)
    color_params = device.getColorCameraParams()
    ir_params = device.getIrCameraParams()
    registration = Registration(ir_params, color_params)

    # 预分配对齐后的图像内存
    undistorted = Frame(512, 424, 4)
    registered = Frame(512, 424, 4)

    # ====================================================
    # 【您的黄金空间物理参数 - 严丝合缝保留，绝不改动】
    # ====================================================
    H = 2.0      # 您当前测试时相机的真实高度（2.0 米）
    theta = np.radians(30.0)  # 俯角 30 度

    # 1. 左右宽度：修改为 -1.2 到 1.2 米
    X_MIN, X_MAX = -1, 1  
    
    # 2. 垂直高度：修改为 0.0 到 1.3 米
    Y_MIN, Y_MAX = 0.0, 1.3     
    
    # 3. 前前后深度范围
    Z_MIN, Z_MAX = 1.3, 3.5

    LED_ROWS, LED_COLS = 64, 64  

    fx, fy = ir_params.fx, ir_params.fy
    cx, cy = ir_params.cx, ir_params.cy
    u_grid, v_grid = np.meshgrid(np.arange(512), np.arange(424))

    # 初始化串口通信 (连接 Teensy 4.1)
    ser = None
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200) # 根据实际串口路径修改
        print("Teensy 串口已连接")
    except Exception as e:
        print(f"Teensy 串口未连接（本地预览调试开启）")

    # ====================================================
    # 【新增：一阶低通平滑滤波器与零延迟跳帧限频器初始化】
    # ====================================================
    # 缓存 48 个通道的历史平滑浮点值
    smoothed_box_degrees = {k: 0.0 for k in KEYS_ORDER}
    
    last_process_time = 0.0
    TARGET_FPS = 20  # 🎯 限制发送和画面刷新为 20 帧/秒（完美降低串口负担，防止机械舵机震颤）

    print("\n✅ 驱动与 AI 模型就绪！正在进行实时高精度抠像映射...\n")

    try:
        while True:
            # 高频全速读取 Kinect 数据流，防止驱动队列积压产生物理延时
            frames = listener.waitForNewFrame()

            # --- 零延迟跳帧限频核心逻辑 ---
            current_time = time.time()
            if current_time - last_process_time < (1.0 / TARGET_FPS):
                # 间隔未到，以极快的速度释放本帧并跳过，实现零延迟降频
                listener.release(frames)
                continue
            
            # 记录本次成功执行的时间戳
            last_process_time = current_time

            color_frame = frames[FrameType.Color]
            depth_frame = frames[FrameType.Depth]

            # 3. 将彩色图与深度图进行像素级对齐
            registration.apply(color_frame, depth_frame, undistorted, registered)

            # 提取对齐后的彩色图 (去除 Alpha 通道)
            color_aligned = registered.asarray(np.uint8)[:, :, :3]

            # 4. 【YOLO AI 抠像】
            results = model.predict(color_aligned, classes=[0], verbose=False)

            # 创建一个全黑的 AI 掩膜
            ai_mask = np.zeros((424, 512), dtype=np.uint8)

            if results[0].masks is not None:
                for mask_contour in results[0].masks.xy:
                    poly = np.array(mask_contour, dtype=np.int32)
                    cv2.fillPoly(ai_mask, [poly], 1)

            ai_mask_bool = ai_mask > 0

            # 5. 【3D 物理空间还原】
            Z_c = depth_frame.asarray(np.float32) / 1000.0
            X_c = (u_grid - cx) * Z_c / fx
            Y_c = (v_grid - cy) * Z_c / fy

            # ====================================================
            # 【核心修复】：把 Y_c 相关的加号全部改为减号！
            # 彻底修复地面影子被错误计算到半空中的 Bug
            # ====================================================
            Y_w = H - Y_c * np.cos(theta) - Z_c * np.sin(theta)
            Z_w = -Y_c * np.sin(theta) + Z_c * np.cos(theta)
            X_w = X_c

            # 6. 【高精度终极过滤】
            # AI蒙版 + 剔除盲区 + 砍掉离地 15cm 以下的所有影子和贴地线缆
            human_mask = ai_mask_bool & (Z_c > 0.1) & (Y_w > 0.15)

            # ====================================================
            # 【您的透视缩放参数 - 完美保留】
            # ====================================================
            Z_REF = 1.2  
            SCALE_STRENGTH = 0.8  
            
            # 7. 【网格映射与亮度计算】
            binary_grid = np.zeros((LED_ROWS, LED_COLS), dtype=np.uint8)
            depth_buffer = np.zeros((LED_ROWS, LED_COLS), dtype=np.float32)

            xs = X_w[human_mask]
            ys = Y_w[human_mask]
            zs = Z_w[human_mask]

             # ========= 修改后（去掉 Z_MIN 的左侧限制，仅保留 Z_MAX 右侧背景限制） =========
            for x, y, z in zip(xs, ys, zs):
                if z <= Z_MAX:  # <-- 解除 Z_MIN 限制，允许近距离通过
                    z_clamped = max(z, 1.1)  # <-- 用安全夹锁死近距离的最大缩放比例
                    
                    raw_scale = Z_REF / z_clamped
                    scale = SCALE_STRENGTH * raw_scale + (1.0 - SCALE_STRENGTH)
                    
                    y_scaled = y * scale
                    x_scaled = x * scale
                    
                    if (X_MIN <= x_scaled <= X_MAX) and (Y_MIN <= y_scaled <= Y_MAX):
                        col = int((x_scaled - X_MIN) / (X_MAX - X_MIN) * (LED_COLS - 1))
                        row = int((Y_MAX - y_scaled) / (Y_MAX - Y_MIN) * (LED_ROWS - 1))
                        
                        binary_grid[row, col] = 255
                        depth_buffer[row, col] = z
                        
                # ====================================================
            # 【核心拯救：OpenCV 闭运算（自动缝合黑衣服引起的身体空洞）】
            # ====================================================
            # 使用一个 7x7 的椭圆核，自动检测并缝合人体内部的所有“黑色真空地带”
            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            binary_grid = cv2.morphologyEx(binary_grid, cv2.MORPH_CLOSE, kernel_close)

            # 区分不同的人
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_grid)
            led_preview = np.zeros((LED_ROWS, LED_COLS, 3), dtype=np.uint8)

            # ====================================================
            # 【8. 新增：将 3D 人体点云高精度映射到 6x11 物理网格上】
            # ====================================================
            box_degrees = {k: 0 for k in KEYS_ORDER}

            for x, y, z in zip(xs, ys, zs):
                if z <= Z_MAX:  # <-- 解除 Z_MIN 限制
                    z_clamped = max(z, 1.1)

                    scale = SCALE_STRENGTH * (Z_REF / z_clamped) + (1.0 - SCALE_STRENGTH)
                    y_scaled = y * scale
                    x_scaled = x * scale

                    if (X_MIN <= x_scaled <= X_MAX) and (Y_MIN <= y_scaled <= Y_MAX):
                        col_6x11 = int((x_scaled - X_MIN) / (X_MAX - X_MIN) * 11)
                        row_6x11 = int((Y_MAX - y_scaled) / (Y_MAX - Y_MIN) * 6)
                        
                        col_6x11 = np.clip(col_6x11, 0, 10)
                        row_6x11 = np.clip(row_6x11, 0, 5)

                        box_id = FRONT_GRID[row_6x11][col_6x11]
                        
                        # 距离越近，计算值越大。如果 z 只有 1.0米，计算结果会自动溢出并被下面的 clip 锁死在 255 满分状态
                        degree = 255 - (z - Z_MIN) / (Z_MAX - Z_MIN) * 255
                        degree = int(np.clip(degree, 0, 255))
                        box_degrees[box_id] = max(box_degrees[box_id], degree)

            # ====================================================
            # 【新增：一阶低通平滑滤波算法，彻底干掉物理灯箱与舵机的频闪】
            # ====================================================
            ALPHA = 0.25  # 滤波系数（0.1~0.4之间。值越小越平滑，0.25为滤波防抖与物理时滞的最佳平衡点）
            for key in KEYS_ORDER:
                raw_val = box_degrees[key]
                # 指数平滑滤波计算
                smoothed_box_degrees[key] = ALPHA * raw_val + (1.0 - ALPHA) * smoothed_box_degrees[key]
                # 四舍五入回整型覆盖原始输出
                box_degrees[key] = int(round(smoothed_box_degrees[key]))

            # ====================================================
            # 【9. 新增：打包发送固定 51 字节的串口二进制数据包并后台打印】
            # ====================================================
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
                if stats[i, cv2.CC_STAT_AREA] < 5:  # 稍微过滤极小色块
                    continue

                person_mask = (labels == i)
                
                # 1. 找出当前这个人在灯板上所有带有真实深度数据的像素点
                active_pixels = person_mask & (depth_buffer > 0)
                pixel_depths = depth_buffer[active_pixels]

                # 2. 逐像素映射亮度：最近 1.3m 对应最大亮度 255，最远 3.5m 对应最小亮度 50
                brightness_values = 255 - (pixel_depths - Z_MIN) / (Z_MAX - Z_MIN) * (255 - 0)
                # 限制在 50 到 255 范围内，并转换为 uint8 整数格式
                brightness_values = np.clip(brightness_values, 50, 255).astype(np.uint8)

                # 3. 将对应像素点区域的红色通道（BGR中的 R 通道，索引为 2）赋值为渐变亮度值
                led_preview[active_pixels, 2] = brightness_values

            # 8. 预览渲染 (双屏拼接)
            debug_view = cv2.resize(led_preview, (512, 512), interpolation=cv2.INTER_NEAREST)
            
            # (可选) 在旁边同时显示 AI 看到的对齐画面，方便您调试对比
            preview_color = color_aligned.copy()
            
            # 【修复1】：使用 NumPy 原生矩阵运算替代 cv2.addWeighted 解决蒙版切片维度报错
            if np.any(ai_mask_bool):  # 画面里有人的时候才渲染绿幕
                preview_color[ai_mask_bool] = (preview_color[ai_mask_bool] * 0.5 + np.array([0, 255, 0]) * 0.5).astype(np.uint8)
            
            # 【修复2】：统一两张图像的高度。原图是 424 高，LED 图是 512 高，把右侧图片也拉伸到 512x512
            preview_color_resized = cv2.resize(preview_color, (512, 512))
            
            # 将灯板图和 AI 识别图水平拼接在一起显示
            combined_view = np.hstack((debug_view, preview_color_resized))
            cv2.imshow("LED Matrix Live Preview & AI Vision", combined_view)

            # ====================================================
            # 【新增：渲染物理灯箱模拟器独立窗口 (6x11 大小格子) 】
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