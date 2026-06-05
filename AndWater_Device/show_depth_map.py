import sys
import numpy as np
import cv2
from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, CudaPacketPipeline

def main():
    # 1. 优先使用 CUDA 硬件加速管线
    try:
        pipeline = CudaPacketPipeline()
        print("成功启用 CUDA 硬件加速深度图处理器")
    except Exception as e:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()
        print("CUDA 启动失败，降级为 CPU 处理器:", e)

    freenect2 = Freenect2()
    if freenect2.enumerateDevices() == 0:
        print("未检测到 Kinect v2 相机")
        sys.exit(1)

    device = freenect2.openDevice(freenect2.getDeviceSerialNumber(0), pipeline=pipeline)
    
    # 仅订阅深度图
    listener = SyncMultiFrameListener(FrameType.Depth)
    device.setIrAndDepthFrameListener(listener)
    device.start()

    print("\n--------------------------------------------------")
    print(" 原始物理深度探测器已启动。")
    print(" 画面中【纯黑色区域】代表传感器完全无法接收数据的物理盲区/空洞。")
    print(" 请在图像窗口上按下 'q' 键退出程序。")
    print("--------------------------------------------------\n")

    try:
        while True:
            frames = listener.waitForNewFrame()
            depth_frame = frames[FrameType.Depth]
            
            # 获取物理深度矩阵（float32，单位：毫米）
            depth_data = depth_frame.asarray()  

            # 2. 归一化处理便于彩色可视化（限制在 0.5 米到 4.0 米之间）
            min_depth = 500.0   # 0.5 米
            max_depth = 4000.0  # 4.0 米
            
            depth_scaled = np.clip(depth_data, min_depth, max_depth)
            depth_normalized = ((depth_scaled - min_depth) / (max_depth - min_depth) * 255).astype(np.uint8)
            
            # 3. 应用高对比度伪彩色映射（越近越红，越远越蓝）
            color_depth = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # 4. 重点：将绝对盲区（深度值接近 0 毫米的像素）标记为【纯黑色】，暴露所有的空洞
            invalid_pixels = (depth_data < 100.0)  # 深度小于 10 厘米的均视为无效盲区
            color_depth[invalid_pixels] = [0, 0, 0]

            # 5. 在左上角实时绘制画面正中心的物理深度数值（毫米）
            h, w = depth_data.shape
            center_val = depth_data[h // 2, w // 2]
            cv2.putText(color_depth, f"Center Depth: {center_val:.1f} mm", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            # 绘制中心十字靶心以便对齐
            cv2.drawMarker(color_depth, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)

            cv2.imshow("Raw Depth Map Analyzer", color_depth)

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