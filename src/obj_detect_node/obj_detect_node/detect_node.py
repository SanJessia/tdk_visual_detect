import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from ultralytics import YOLO

# 🌟 引入 ROS 2 影像傳輸與橋接套件 (取代 pyrealsense2)
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters

from tdk_interfaces.msg import TargetInfo

class ObjDetectNode(Node):
  def __init__(self):
    super().__init__('obj_detect_node')
    self.publisher_ = self.create_publisher(TargetInfo, 'goose_target_info', 10)
    
    # --- 省資源開關設定 ---
    self.enable_debug_view = True    # TODO: 實際上場比賽時，請改為 False 關閉畫面顯示
    
    # 參數設定 (完全保留你的數值)
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

    # 🌟 初始化 OpenCV 影像橋接器
    self.bridge = CvBridge()
    self.intrinsics = None  # 用來存相機內部參數
    
    # =========================================================
    # 🌟 建立訂閱者 (Subscriber) 架構，取代原本的 pyrealsense2
    # =========================================================
    # 1. 訂閱相機參數 (為了計算 3D 座標)
    self.info_sub = self.create_subscription(
        CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
    
    # 2. 訂閱彩色與深度影像，並使用「時間同步器」確保兩張畫面是同一瞬間拍的
    self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
    self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
    
    # queue_size=5, slop=0.1 代表容忍 0.1 秒內的時間差
    self.ts = message_filters.ApproximateTimeSynchronizer(
        [self.color_sub, self.depth_sub], queue_size=5, slop=0.1)
    self.ts.registerCallback(self.sync_callback)
    
    self.get_logger().info('YOLO 辨識節點已啟動 (解耦訂閱版)，等待相機畫面...')

  def info_callback(self, msg):
    # 只抓取一次相機參數就存起來
    if self.intrinsics is None:
        self.intrinsics = msg
        self.get_logger().info('成功獲取 RealSense 內部參數！')

  def calculate_camera_ground_position(self, Xc, Yc, Zc):
    pitch_rad = math.radians(self.camera_pitch_deg)
    cos_p = math.cos(pitch_rad)
    sin_p = math.sin(pitch_rad)
    ground_x = (Zc * cos_p) - (Yc * sin_p)
    ground_y = -Xc
    target_z = self.camera_height_m - (Zc * sin_p + Yc * cos_p)
    return ground_x, ground_y, target_z

  def sync_callback(self, color_msg, depth_msg):
    # 如果還沒拿到相機參數，先跳過
    if self.intrinsics is None:
        return

    color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
    depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

    # ================= 以下維持你原本的 YOLO 執行推論代碼 =================
    results = self.model(color_image, verbose=False, imgsz=320)
    boxes = results[0].boxes
    
    if len(boxes) == 0:
        if self.enable_debug_view:
            cv2.imshow('ObjDetectNode - Color', color_image)
            cv2.waitKey(1)
        return

    sorted_boxes = sorted(boxes, key=lambda b: float(b.conf[0]), reverse=True)
    
    target_found = False

    for box in sorted_boxes:
      conf = float(box.conf[0])
      if conf < 0.76:  # 保留你的 0.76 設定
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

      cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
      
      depth_mm = depth_image[cy, cx]
      if depth_mm == 0:  # 0 代表深度感測器破圖沒測到
        continue
      
      Zc = depth_mm / 1000.0
      if Zc < 0.01 or Zc > 4.0:
        continue

      #  手動進行 3D 座標轉換 (取代原本 pyrealsense2 的 deproject 函式)
      fx = self.intrinsics.k[0]
      fy = self.intrinsics.k[4]
      ppx = self.intrinsics.k[2]
      ppy = self.intrinsics.k[5]
      
      Xc = (cx - ppx) * Zc / fx
      Yc = (cy - ppy) * Zc / fy

      # 座標計算與平滑
      ground_x, ground_y, target_z = self.calculate_camera_ground_position(Xc, Yc, Zc)
      
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

      self.get_logger().info(f'發布 [{class_name}] -> X:{smoothed_x:.2f}m, Y:{smoothed_y:.2f}m')
      target_found = True

      # UI 繪製
      if self.enable_debug_view:
          # 取代 rs.colorizer()，用 OpenCV 產生深度熱力圖
          depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
          cv2.circle(depth_colormap, (cx, cy), 5, (255, 255, 255), -1)
          cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
          cv2.putText(color_image, f'{class_name} {conf:.2f}', (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
          cv2.putText(color_image, f'X: {smoothed_x:.2f}m, Y: {smoothed_y:.2f}m', (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
          cv2.imshow('ObjDetectNode - Depth Debug', depth_colormap)

      break # 只處理信心度最高的一個

    # 顯示主畫面 (僅 Debug 模式)
    if self.enable_debug_view:
        cv2.imshow('ObjDetectNode - Color', color_image)
        cv2.waitKey(1)

  def destroy_node(self):
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