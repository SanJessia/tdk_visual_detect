import math

import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    print("準備啟動 RealSense 相機...")
    
    # 1. 建立一個 RealSense 影像傳輸管線
    pipeline = rs.pipeline()
    config = rs.config()
    camera_offset = -0.1536
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # ==========================================
    pitch_angle = 45.0  # 你的俯角
    theta = math.radians(pitch_angle)
    try:
        align_to = rs.stream.color
        align = rs.align(align_to)
        pipeline.start(config)
        print("相機啟動成功！請按 'q' 鍵關閉視窗。")

        while True:
            # 等待相機傳送最新的一組畫面
            
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not color_frame:
                continue
            img = np.asanyarray(color_frame.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_bound = np.array([15, 100, 80])  
            upper_bound = np.array([40, 255, 255])  
            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            max_area = 0
            for cnt in contours:
                
                if cv2.contourArea(cnt) > 3000:
                    if cv2.contourArea(cnt) > max_area:
                        x,y,w,h = cv2.boundingRect(cnt)
                        max_area = cv2.contourArea(cnt)
    

            if max_area is not 0 and depth_frame:
                cx = x + w // 2
                cy = y + h // 2
                distance = depth_frame.get_distance(cx, cy)
                depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
                camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [cx, cy], distance)

                target_x = camera_coordinate[0]
                target_y = camera_coordinate[1]
                target_z = camera_coordinate[2]
                robot_z = (target_z * math.cos(theta))# - (target_y * math.sin(theta))
                print(f"the bale is on ({target_x}, {target_y}, {target_z})")
                print(f"deduction{target_x + camera_offset}")
                print(f"target z is {robot_z}")


                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.circle(img, (cx, cy), 5, (0, 255, 0), -1)
            cv2.imshow('straw center tracking', img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"發生錯誤：{e}")

    finally:
        # 8. 安全關閉相機與視窗 (這步很重要，不然下次開程式相機會卡住)
        pipeline.stop()
        cv2.destroyAllWindows()
        print("程式結束。")

if __name__ == "__main__":
    main()