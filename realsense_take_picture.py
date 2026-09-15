import cv2
import numpy as np
import pyrealsense2 as rs
import os
import datetime
from PIL import Image  # ⬅️ 增加這行
print("=== Intel RealSense 自訂資料夾與 JPG 拍照程式 ===")

# 1. 讓使用者輸入指定的資料夾名稱
default_dir = "bamboo_bracket"
user_input = input(f"請輸入儲存 JPG 照片的資料夾名稱 (直接按下 Enter 則使用預設 '{default_dir}')：").strip()

# 如果使用者沒有輸入，就使用預設資料夾
output_dir = user_input if user_input else default_dir

# 建立資料夾（如果資料夾不存在的話）
os.makedirs(output_dir, exist_ok=True)
print(f"-> JPG 照片將會儲存至資料夾：./{output_dir}/")

# 2. 設定 RealSense Pipeline
pipeline = rs.pipeline()
config = rs.config()

# 啟用彩色串流 (Color Stream)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 3. 開始串流
pipeline.start(config)
print("\nRealSense 相機已成功啟動！")
print("操作說明：")
print("  - 點擊即時預覽視窗後，按下鍵盤的 [s] 鍵：拍照並存為 JPG 格式")
print("  - 按下鍵盤的 [q] 鍵：離開程式")

try:
    while True:
        # 等待一組連續的畫面
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            continue

        # 將 RealSense 畫面轉換為 OpenCV 的 NumPy 格式
        color_image = np.asanyarray(color_frame.get_data())

        # 顯示即時畫面
        cv2.imshow('RealSense JPG Capture', color_image)

        # 監聽鍵盤事件
        key = cv2.waitKey(1) & 0xFF

        # 按下 's' 鍵拍照存檔為 JPG
        if key == ord('s'):
            # 產生時間戳記作為檔名（副檔名改為 .jpg）
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            file_path = os.path.join(output_dir, f"capture_{timestamp}.jpg")
            color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                        
            # 2. 將 NumPy 陣列轉換為 PIL 圖片物件
            pil_img = Image.fromarray(color_image_rgb)
            
            # 3. 儲存為高品質的標準 JPG (這樣 Roboflow 100% 認得)
            pil_img.save(file_path, "JPEG", quality=95)
            print(f"已成功儲存 JPG 照片：{file_path}")

        # 按下 'q' 鍵離開迴圈
        elif key == ord('q'):
            print("正在關閉相機與程式...")
            break

finally:
    # 4. 停止串流並釋放資源
    pipeline.stop()
    cv2.destroyAllWindows()