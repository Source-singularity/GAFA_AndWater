import sys
import numpy as np
import cv2
from ultralytics import YOLO
from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, Registration, Frame, CudaPacketPipeline

def main():
    print("正在加载 YOLOv11-Seg 实例分割模型...")
    model = YOLO("yolo11n-seg.pt")  

    try:
        pipeline = CudaPacketPipeline()
        print("成功启用 CUDA 硬件加速")
    except Exception as e:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()

    freenect2 = Freenect2()
    if freenect2.enumerateDevices() == 0:
        print("未检测到 Kinect v2 相机")
        sys.exit(1)

    device = freenect2.openDevice(freenect2.getDeviceSerialNumber(0), pipeline=pipeline)

    # 必须同时订阅彩色和深度
    listener = SyncMultiFrameListener(FrameType.Color | FrameType.Depth)
    device.setColorFrameListener(listener)
    device.setIrAndDepthFrameListener(listener)
    device.start()

    # 初始化对齐器
    color_params = device.getColorCameraParams()
    ir_params = device.getIrCameraParams()
    registration = Registration(ir_params, color_params)

    undistorted = Frame(512, 424, 4)
    registered = Frame(512, 424, 4)

    # ====================================================
    # 【核心黄金物理参数】
    # ====================================================
    H = 2.0                   # 相机高度
    theta = np.radians(30.0)  # 俯角 (如果您觉得 30 度人影拉得太长，可改为 15.0 度)
    
    X_MIN, X_MAX = -1.2, 1.2  # 左右宽度
    Y_MIN, Y_MAX = 0.0, 1.8   # 垂直高度：最高 1.8 米，防止走近时上半身被裁剪
    Z_MAX = 3.5               # 最远深度

    LED_ROWS, LED_COLS = 64, 64

    # 提前生成坐标系，使用红外相机内参
    fx, fy = ir_params.fx, ir_params.fy
    cx, cy = ir_params.cx, ir_params.cy
    u_grid, v_grid = np.meshgrid(np.arange(512), np.arange(424))

    print("\n--------------------------------------------------")
    print(" 基础投影测试已启动。")
    print(" 左侧：64x64 纯净投影结果 | 右侧：AI 掩膜 + 原始深度")
    print("--------------------------------------------------\n")

    try:
        while True:
            frames = listener.waitForNewFrame()
            color_frame = frames[FrameType.Color]
            depth_frame = frames[FrameType.Depth]

            # 1. 图像对齐
            registration.apply(color_frame, depth_frame, undistorted, registered)
            color_aligned = registered.asarray(np.uint8)[:, :, :3]

            # 2. YOLO 提取掩膜
            results = model.predict(color_aligned, classes=[0], verbose=False)
            ai_mask = np.zeros((424, 512), dtype=np.uint8)

            if results[0].masks is not None:
                for mask_contour in results[0].masks.xy:
                    poly = np.array(mask_contour, dtype=np.int32)
                    cv2.fillPoly(ai_mask, [poly], 1)

            # 【防黑洞补丁】：使用 15x15 的闭运算，强行缝合黑色衣服或视差导致的人体裂缝
            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
            ai_mask = cv2.morphologyEx(ai_mask, cv2.MORPH_CLOSE, kernel_close)
            ai_mask_bool = ai_mask > 0

            # 3. 提取原始物理深度（坚决不用 undistorted，防止产生视差空洞）
            Z_c = depth_frame.asarray(np.float32) / 1000.0  
            X_c = (u_grid - cx) * Z_c / fx
            Y_c = (v_grid - cy) * Z_c / fy

            # 4. 真实空间解算（注意 Y_c 前面必须是减号）
            Y_w = H - Y_c * np.cos(theta) - Z_c * np.sin(theta)
            Z_w = -Y_c * np.sin(theta) + Z_c * np.cos(theta)
            X_w = X_c

            # 5. 高精度剔除（保留大于 0.15 米的高度的物体，且属于人像掩膜内）
            human_mask = ai_mask_bool & (Z_c > 0.1) & (Y_w > 0.15)

            # 6. 网格映射计算
            Z_REF = 1.2  
            SCALE_STRENGTH = 0.8  
            
            binary_grid = np.zeros((LED_ROWS, LED_COLS), dtype=np.uint8)
            depth_buffer = np.zeros((LED_ROWS, LED_COLS), dtype=np.float32)

            xs = X_w[human_mask]
            ys = Y_w[human_mask]
            zs = Z_w[human_mask]

            for x, y, z in zip(xs, ys, zs):
                # 唯一限制：只裁掉大于 Z_MAX (3.5米) 的背景，完全放开近距离限制！
                if z <= Z_MAX:
                    # 安全限幅：防止人贴到 50cm 时，透视缩放比例除以极小的 Z 导致数值爆炸
                    z_clamped = max(z, 0.8) 
                    
                    scale = SCALE_STRENGTH * (Z_REF / z_clamped) + (1.0 - SCALE_STRENGTH)
                    y_scaled = y * scale
                    x_scaled = x * scale
                    
                    if (X_MIN <= x_scaled <= X_MAX) and (Y_MIN <= y_scaled <= Y_MAX):
                        col = int((x_scaled - X_MIN) / (X_MAX - X_MIN) * (LED_COLS - 1))
                        row = int((Y_MAX - y_scaled) / (Y_MAX - Y_MIN) * (LED_ROWS - 1))
                        
                        binary_grid[row, col] = 255
                        depth_buffer[row, col] = z

            # 7. 渲染左侧 64x64 红影预览窗
            led_preview = np.zeros((LED_ROWS, LED_COLS, 3), dtype=np.uint8)
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_grid)
            
            for i in range(1, num_labels):
                if stats[i, cv2.CC_STAT_AREA] < 5:  
                    continue
                person_mask = (labels == i)
                active_pixels = person_mask & (depth_buffer > 0)
                # 简单粗暴的纯红色填充（用来检查轮廓是否完整）
                led_preview[active_pixels] = [0, 0, 255] 

            debug_view = cv2.resize(led_preview, (512, 512), interpolation=cv2.INTER_NEAREST)

            # ====================================================
            # 【8. 渲染右侧辅助验证窗（修复了漏写定义与高度对齐的 Bug）】
            # ====================================================
            # 渲染右侧辅助验证窗：在原始深度图上叠加一层半透明的绿色 AI 掩膜
            # 帮助您直观检查 YOLO 抠出来的人像，是否与原始深度完美重合
            depth_scaled = np.clip(depth_frame.asarray(), 500.0, 3500.0)
            depth_norm = ((depth_scaled - 500.0) / 3000.0 * 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            
            if np.any(ai_mask_bool):
                depth_color[ai_mask_bool] = (depth_color[ai_mask_bool] * 0.5 + np.array([0, 255, 0]) * 0.5).astype(np.uint8)

            # 【修复点】：在这里补充定义 raw_depth_resized，将 424x512 的图像等比缩放到 512x512
            raw_depth_resized = cv2.resize(depth_color, (512, 512))

            # 拼接显示
            combined_view = np.hstack((debug_view, raw_depth_resized))
            cv2.imshow("Base Mapping Test", combined_view)

            listener.release(frames)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        device.stop()
        device.close()
        cv2.destroyAllWindows()
        print("设备已关闭")

if __name__ == "__main__":
    main()