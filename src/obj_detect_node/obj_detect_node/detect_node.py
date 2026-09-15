import math
import cv2
from geometry_msgs.msg import Point
from tdk_interfaces.msg import TargetInfo

import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from ultralytics import YOLO

class ObjDetectNode(Node):
  def __init__(self):
    super().__init__('obj_detect_node')
    self.publisher_ = self.create_publisher(TargetInfo, 'goose_target_info', 10)
    
    # --- 省資源開關設定 ---
    self.enable_debug_view = True    # TODO: 實際上場比賽時，請改為 False 關閉畫面顯示
    
    # 參數設定
    self.camera_pitch_deg = 66.0
    self.camera_height_m = 0.704
    self.robo_offset_x = 0.1753
    self.robo_offset_y = 0.152
    
    self.image_width = 640
    self.image_height = 480
    self.max_box_area_ratio = 0.5  
    self.edge_margin = 5

    self.history_x = []
    self.history_y = []
    self.history_max_size = 5

    self.get_logger().info('Loading YOLO model...')
    self.model = YOLO('/workspace/weights/best.pt')

    # 初始化 RealSense
    self.pipeline = rs.pipeline()
    self.config = rs.config()
    self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    self.profile = self.pipeline.start(self.config)
    self.align = rs.align(rs.stream.color)
    
    if self.enable_debug_view:
        self.colorizer = rs.colorizer()

    color_profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
    self.intrinsics = color_profile.get_intrinsics()

    # 優化：將頻率降至 15Hz (0.066秒)，對機器人導航已足夠，可省下一半算力
    self.timer = self.create_timer(0.066, self.timer_callback)
    self.get_logger().info('ObjDetectNode started. (Optimized Version)')

  def calculate_camera_ground_position(self, p_cam):
    Xc, Yc, Zc = p_cam
    pitch_rad = math.radians(self.camera_pitch_deg)
    cos_p = math.cos(pitch_rad)
    sin_p = math.sin(pitch_rad)
    ground_x = (Zc * cos_p) - (Yc * sin_p)
    ground_y = -Xc
    target_z = self.camera_height_m - (Zc * sin_p + Yc * cos_p)
    return ground_x, ground_y, target_z

  def timer_callback(self):
    # 🌟 關鍵修正 1：用 while 迴圈把舊畫面全部抽乾，只保留「最新」的一幀
    latest_frames = None
    while True:
      f = self.pipeline.poll_for_frames()
      if not f:
        break
      latest_frames = f

    # 如果連最新的畫面都沒有，就保持視窗刷新然後退出
    if not latest_frames:
      if self.enable_debug_view:
        cv2.waitKey(1)
      return

    frames = latest_frames

    # 取出彩色影像
    color_frame = frames.get_color_frame()
    if not color_frame:
      if self.enable_debug_view:
        cv2.waitKey(1)
      return

    color_image = np.asanyarray(color_frame.get_data())

    # 🌟 關鍵修正 2：在 YOLO 運算前先刷新一次視窗，防止作業系統誤判程式當機
    if self.enable_debug_view:
        cv2.waitKey(1)

    # ================= 以下維持你原本的 YOLO 執行推論代碼 =================
    # YOLO 執行推論

    results = self.model(color_image, verbose=False, imgsz=320)
    boxes = results[0].boxes
    
    if len(boxes) == 0:
        if self.enable_debug_view:
            cv2.imshow('ObjDetectNode - Color', color_image)
            cv2.waitKey(1)
        return

    # 將預測結果照信心度 (Confidence) 由高到低排序，確保我們優先處理最確定的目標
    sorted_boxes = sorted(boxes, key=lambda b: float(b.conf[0]), reverse=True)
    
    target_found = False

    for box in sorted_boxes:
      conf = float(box.conf[0])
      if conf < 0.76:
        continue

      cls_id = int(box.cls[0])
      class_name = self.model.names[cls_id]
      if class_name == 'vase':
        continue

      x1, y1, x2, y2 = map(int, box.xyxy[0])
      
      # 面積與邊緣過濾
      if (x1 < self.edge_margin or y1 < self.edge_margin or 
          x2 > (self.image_width - self.edge_margin) or y2 > (self.image_height - self.edge_margin)):
        continue
      
      box_area = (x2 - x1) * (y2 - y1)
      if (box_area / (self.image_width * self.image_height)) > self.max_box_area_ratio:
        continue

      # === 只有確認這是一個有效的目標後，我們才花費 CPU 資源去做深度對齊 ===
      aligned_frames = self.align.process(frames)
      depth_frame = aligned_frames.get_depth_frame()
      if not depth_frame:
          continue

      cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
      depth = depth_frame.get_distance(cx, cy)

      if depth < 0.01 or depth > 4.0:
        continue

      # 座標計算與平滑
      p_cam = rs.rs2_deproject_pixel_to_point(self.intrinsics, [cx, cy], depth)
      ground_x, ground_y, target_z = self.calculate_camera_ground_position(p_cam)
      
      robo_x = ground_x + self.robo_offset_x
      robo_y = ground_y + self.robo_offset_y
      self.history_x.append(robo_x)
      self.history_y.append(robo_y)

      if len(self.history_x) > self.history_max_size:
        self.history_x.pop(0)
        self.history_y.pop(0)

      smoothed_x = sum(self.history_x) / len(self.history_x)
      smoothed_y = sum(self.history_y) / len(self.history_y)

      # 發布 ROS 2 座標
      target_msg = TargetInfo()
      target_msg.x = float(smoothed_x)
      target_msg.y = float(smoothed_y)
      target_msg.z = float(target_z)
      target_msg.class_name = class_name
      target_msg.confidence = float(conf)

      self.publisher_.publish(target_msg)

      self.get_logger().info(f'✅ 發布 [{class_name}] -> X:{smoothed_x:.2f}m, Y:{smoothed_y:.2f}m')
      
      target_found = True

      # UI 繪製 (僅在開啟 Debug 模式時才浪費資源去畫)
      if self.enable_debug_view:
          depth_colormap = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
          cv2.circle(depth_colormap, (cx, cy), 5, (255, 255, 255), -1)
          cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
          cv2.putText(color_image, f'{class_name} {conf:.2f}', (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
          cv2.putText(color_image, f'X: {smoothed_x:.2f}m, Y: {smoothed_y:.2f}m', (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
          cv2.imshow('ObjDetectNode - Depth Debug', depth_colormap)

      # 優化：我們只需要追蹤畫面上信心度最高的那個目標，處理完直接跳出迴圈，防止陣列錯亂
      break

    # 顯示主畫面 (僅 Debug 模式)
    if self.enable_debug_view:
        cv2.imshow('ObjDetectNode - Color', color_image)
        cv2.waitKey(1)

  def destroy_node(self):
    self.pipeline.stop()
    if self.enable_debug_view:
        cv2.destroyAllWindows()
    super().destroy_node()

def main(args=None):
  rclpy.init(args=args)
  node = ObjDetectNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()