import sys
import numpy as np
import cv2
from ultralytics import YOLO
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import CudaPacketPipeline

def main():
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

    # 您的黄金空间物理参数
    H = 2.0      
    theta = np.radians(30.0)  

    X_MIN, X_MAX = -1, 1  
    Y_MIN, Y_MAX = 0.0, 1.8     
    Z_MIN, Z_MAX = 1.3, 3.5

    LED_ROWS, LED_COLS = 64, 64  

    fx, fy = ir_params.fx, ir_params.fy
    cx, cy = ir_params.cx, ir_params.cy
    u_grid, v_grid = np.meshgrid(np.arange(512), np.arange(424))

    print("\n✅ 基础版驱动与 AI 模型就绪！正在进行实时高精度抠像展示...\n")

    try:
        while True:
            frames = listener.waitForNewFrame()
            color_frame = frames[FrameType.Color]
            depth_frame = frames[FrameType.Depth]

            registration.apply(color_frame, depth_frame, undistorted, registered)
            color_aligned = registered.asarray(np.uint8)[:, :, :3]

            results = model.predict(color_aligned, classes=[0], verbose=False, device=0)
            ai_mask = np.zeros((424, 512), dtype=np.uint8)

            if results[0].masks is not None:
                for mask_contour in results[0].masks.xy:
                    poly = np.array(mask_contour, dtype=np.int32)
                    cv2.fillPoly(ai_mask, [poly], 1)

            # 【AI掩膜强力缝合补丁】：用大核闭运算将近距离因视差造成的左右断裂完美缝合
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

            # 6. 【高精度终极过滤（放开近处 Z_MIN 限制）】
            human_mask = ai_mask_bool & (Z_c > 0.1) & (Y_w > 0.15)

            # 透视缩放参数
            Z_REF = 1.2  
            SCALE_STRENGTH = 0.8  
            
            # 7. 【网格映射与亮度计算】
            binary_grid = np.zeros((LED_ROWS, LED_COLS), dtype=np.uint8)
            depth_buffer = np.zeros((LED_ROWS, LED_COLS), dtype=np.float32)

            xs = X_w[human_mask]
            ys = Y_w[human_mask]
            zs = Z_w[human_mask]

            for x, y, z in zip(xs, ys, zs):
                if z <= Z_MAX:
                    # 引入深度安全幅度，限制 z 最小不小于 0.8，防止贴到镜头前时公式溢出
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

            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_grid)
            led_preview = np.zeros((LED_ROWS, LED_COLS, 3), dtype=np.uint8)

            for i in range(1, num_labels):
                if stats[i, cv2.CC_STAT_AREA] < 5:  
                    continue

                person_mask = (labels == i)
                active_pixels = person_mask & (depth_buffer > 0)
                pixel_depths = depth_buffer[active_pixels]

                # 逐像素映射：最近 1.3m 对应最大亮度 255，最远 3.5m 对应最小亮度 0
                brightness_values = 255 - (pixel_depths - Z_MIN) / (Z_MAX - Z_MIN) * (255 - 0)
                brightness_values = np.clip(brightness_values, 50, 255).astype(np.uint8)

                led_preview[active_pixels, 2] = brightness_values

            # 双屏预览
            debug_view = cv2.resize(led_preview, (512, 512), interpolation=cv2.INTER_NEAREST)
            preview_color = color_aligned.copy()
            if np.any(ai_mask_bool):  
                preview_color[ai_mask_bool] = (preview_color[ai_mask_bool] * 0.5 + np.array([0, 255, 0]) * 0.5).astype(np.uint8)
            preview_color_resized = cv2.resize(preview_color, (512, 512))
            
            combined_view = np.hstack((debug_view, preview_color_resized))
            cv2.imshow("LED Matrix Live Preview & AI Vision", combined_view)

            listener.release(frames)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        device.stop()
        device.close()
        cv2.destroyAllWindows()
        print("已安全关闭 Kinect 设备")

if __name__ == "__main__":
    main()