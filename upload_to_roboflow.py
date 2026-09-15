import os
from roboflow import Roboflow

# 1. 初始化 Roboflow (請填入您的 API Key)
rf = Roboflow(api_key="您的_ROBOFLOW_API_KEY")

# 2. 指定您的 Workspace 與 Project 名稱
project = rf.workspace("您的WORKSPACE名稱").project("您的PROJECT名稱")

# 3. 設定要上傳的資料夾
target_dir = "goose"

if not os.path.exists(target_dir):
    print(f"錯誤：找不到名為 '{target_dir}' 的資料夾！")
else:
    print(f"=== 開始上傳 '{target_dir}' 資料夾中的圖片到 Roboflow ===")
    
    valid_extensions = (".jpg", ".jpeg", ".png")
    files = os.listdir(target_dir)
    count = 0

    for filename in files:
        if filename.lower().endswith(valid_extensions):
            file_path = os.path.join(target_dir, filename)
            
            try:
                # 4. 上傳圖片到專案
                project.upload(file_path)
                print(f"成功上傳：{filename}")
                count += 1
            except Exception as e:
                print(f"上傳失敗 {filename}，原因：{e}")

    print(f"\n=== 上傳完成！總共成功上傳 {count} 張圖片到 Roboflow ===")