import cv2
import numpy as np
import pyrealsense2 as rs
import math
from ultralytics import YOLO

CAMERA_PITCH_DEG = 66.0  # 鏡頭視線與地面的夾角 (俯角 66 度)
CAMERA_HEIGHT_M = 0.704  # 相機距離地面的高度 (公尺)

# ==========================================
# 1. 座標轉換核心函式 (以相機正下方為 0,0)
# ==========================================
def calculate_camera_ground_position(p_cam):
    """
    將相機 3D 座標轉換為「以相機正下方地面為原點」的 2D 座標
    :param p_cam: [Xc, Yc, Zc] RealSense 算出的相機座標 (公尺)
    :return: ground_x (前方距離), ground_y (橫向距離), target_z (離地高度)
    """
    Xc, Yc, Zc = p_cam
    
    pitch_rad = math.radians(CAMERA_PITCH_DEG)
    cos_p = math.cos(pitch_rad)
    sin_p = math.sin(pitch_rad)
    
    # 【座標計算】
    # ground_x: 投影到地面的「正前方距離」
    ground_x = (Zc * cos_p) - (Yc * sin_p)
    
    # ground_y: 相機左/右側的距離 (RealSense 的 Xc 往右為正，這裡加個負號讓 Y 軸往左為正，符合一般數學直覺)
    ground_y = -Xc
    
    # target_z: 物件離地高度 (若為 0 代表貼在地上)
    target_z = CAMERA_HEIGHT_M - (Zc * sin_p + Yc * cos_p)
    
    return ground_x, ground_y, target_z

# ==========================================
# 2. 主程式
# ==========================================
def main():
    print("=== YOLO + RealSense (以相機腳下為原點) 定位系統 ===")

    # 1. 載入 YOLO 模型
    model = YOLO("best.pt") 

    # 2. 初始化 RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color) 
    
    # 取得相機內參
    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intrinsics = color_profile.get_intrinsics()

    try:
        while True:
            # 獲取並對齊影像
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            # 3. YOLO 執行推論
            results = model(color_image, verbose=False)
            
            for box in results[0].boxes:
                # 解析 Bounding Box 座標與信心度
                cls_id = int(box.cls[0])
                class_name = model.names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                conf = float(box.conf[0])
                if class_name != 'goose':
                    continue
                if conf < 0.7: 
                    continue

                depth = depth_frame.get_distance(cx, cy)
                if depth < 0.1 or depth > 3.0: 
                    continue

                # 5. 反投影：將像素 + 深度轉換為相機 3D 座標
                p_cam = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
                
                # 6. 座標轉換：計算以相機腳下為 (0,0) 的地面座標
                ground_x, ground_y, target_z = calculate_camera_ground_position(p_cam)

                # ==========================================
                # 畫面繪製與輸出
                # ==========================================
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image, f"{class_name} {conf:.2f}", (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                
                # 在畫面上印出座標 (以相機為中心)
                text = f"X:{ground_x:.2f}m, Y:{ground_y:.2f}m"
                cv2.putText(color_image, text, (cx + 10, cy - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
#175.29
                robo_x = ground_x + 175.29
                print(f"相機前方: {ground_x:.2f}m, 偏左: {ground_y:.2f}m | 離地高度: {target_z:.2f}m")
                print(f"機器人前方: {robo_x:.2f}m, 偏左: {ground_y:.2f}m | 離地高度: {target_z:.2f}m")

            # 顯示影像
            cv2.imshow("Camera-Centric Localization", color_image)

            # 按 'q' 離開
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()