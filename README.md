# 🤖 TDK Visual Object Detection Node (`obj_detect_node`)

專為機器人競賽（如 **TDK 機器人競賽、ASME** 等）設計的 ROS 2 視覺辨識與 3D 空間定位節點。

本專案結合 **Intel RealSense 深度攝影機** 與 **YOLO 深度學習模型**，實現：

* 即時目標辨識
* 相機座標到機器人基座座標的轉換
* 深度與座標雜訊濾波
* 3D 目標位置計算
* 將計算後的 3D 目標點發布至 ROS 2 Topic

以支援後端導航系統進行目標追蹤與導航。

---

# ✨ 核心功能特色

## 1. ⚡ 高效能即時推論（Optimized CPU/APU Performance）

針對沒有獨立 GPU 的輕量型電腦進行最佳化，例如：

* OneXPlayer
* 工控機
* 其他低功耗 CPU / APU 平台

主要最佳化方式：

### Lazy Alignment（延遲對齊機制）

僅在確認 YOLO 偵測到目標後，才執行耗時的深度圖對齊。

降低不必要的 CPU 運算負擔。

### 降低 YOLO 推論解析度

預設使用：

```text
imgsz=320
```

在維持基本辨識能力的同時提升推論速度。

### 模型加速

支援將 YOLO 模型轉換為：

* OpenVINO
* ONNX

藉此在無獨立顯卡的環境下，也能維持穩定且高效的即時推論幀率。

---

## 2. 📐 精準的 3D 地面座標投影

透過 Intel RealSense 提供的相機資訊與幾何計算，將 YOLO 偵測到的影像像素位置轉換為實際 3D 空間座標。

計算過程會使用：

* RealSense Camera Intrinsics（相機內參）
* 相機俯仰角（Pitch）
* 相機安裝高度
* 影像像素座標
* 深度資訊
* 幾何投影公式

最終將目標位置轉換成機器人座標系中的：

```text
(X, Y, Z)
```

其中座標會進一步套用機器人的安裝位移補償：

```text
robo_offset
```

---

## 3. 🛡️ 多重安全過濾機制

為避免錯誤辨識或深度資料造成機器人導航異常，本節點提供多重過濾機制。

### 🎯 信心度過濾

自動排除 YOLO 信心度低於指定門檻的偵測結果。

預設門檻：

```text
confidence >= 0.8
```

可以有效降低低信心度誤判。

---

### 🖼️ 邊緣防呆過濾

自動剔除過度靠近影像邊緣的物件。

原因是影像邊緣可能存在：

* 鏡頭畸變
* 深度資料不穩定
* 物件 Bounding Box 不完整

避免這些因素造成後續 3D 座標計算出現嚴重偏差。

---

### 📦 面積比例過濾

根據 YOLO Bounding Box 佔整張影像的比例進行過濾。

若偵測框過大，可能代表：

* 物件距離攝影機過近
* 錯誤偵測
* 背景被誤判為目標

因此可以透過面積比例限制排除異常偵測結果。

---

## 4. 📊 滑動平均濾波器（Moving Average Filter）

內建歷史座標緩衝區，對連續偵測到的 3D 座標進行滑動平均。

主要可以降低：

* YOLO Bounding Box 抖動
* RealSense 深度雜訊
* 單幀錯誤座標
* 目標位置瞬間跳動

使發布給導航系統的座標更加平滑穩定。

這對機器人導航非常重要，可以避免導航目標點因為影像或深度雜訊而產生不必要的亂跳。

---

# 📂 專案架構

```text
tdk_visual_detect/
├── src/
│   └── obj_detect_node/
│       ├── obj_detect_node/
│       │   └── detect_node.py
│       │       # 主要視覺辨識與 ROS 2 發布節點
│       ├── package.xml
│       └── setup.py
│
├── weights/
│   └── best.pt
│       # YOLO 訓練權重檔
│       # 依個人需求放置
│
└── README.md
```

---

# ⚙️ 系統環境需求

## 作業系統

* Ubuntu 22.04

## ROS 2

* ROS 2 Humble Hawksbill

## Python

* Python 3.10+

## 主要 Python 套件

* `rclpy`
* `ultralytics`
* `pyrealsense2`
* `opencv-python`
* `numpy`
* `geometry_msgs`

---

# 🛠️ 安裝與編譯

## 1. 複製專案

將專案複製到 ROS 2 工作空間的 `src` 資料夾：

```bash
cd ~/your_ws/src

git clone https://github.com/SanJessia/tdk_visual_detect.git
```

---

## 2. 回到工作空間根目錄

```bash
cd ~/your_ws
```

---

## 3. 編譯 ROS 2 Package

```bash
colcon build --packages-select obj_detect_node
```

---

## 4. 載入工作空間環境

```bash
source install/setup.bash
```

如果希望每次開啟 Terminal 都自動載入，也可以將以下指令加入 `~/.bashrc`：

```bash
source ~/your_ws/install/setup.bash
```

---

# 🚀 執行方式

完成編譯並載入 ROS 2 工作空間後，執行：

```bash
ros2 run obj_detect_node detect_node
```

啟動後，節點會開始：

1. 取得 RealSense 相機影像
2. 執行 YOLO 目標辨識
3. 篩選符合條件的 Bounding Box
4. 取得目標深度資訊
5. 計算目標 3D 座標
6. 進行座標平滑
7. 發布至 ROS 2 Topic

---

# 💡 實戰調校小撇步（比賽部署）

在 `detect_node.py` 的 `__init__()` 初始化區塊中，設有除錯畫面開關：

```python
self.enable_debug_view = True
```

---

## 🧪 測試時

建議設定：

```python
self.enable_debug_view = True
```

可以開啟即時 OpenCV 畫面，方便觀察：

* YOLO Bounding Box
* 偵測結果
* 目標位置
* 深度資訊
* 其他除錯資訊

---

## 🏆 實機比賽時

建議設定：

```python
self.enable_debug_view = False
```

關閉除錯畫面後，可以避免不必要的：

* OpenCV UI Rendering
* 影像顯示
* 熱度圖計算
* 額外影像處理

進一步降低 CPU 負載，提升整體執行效率與即時性。

### 比賽部署建議

```python
self.enable_debug_view = False
```

在正式上場前，建議先確認關閉 Debug View 後，ROS 2 Topic 仍能正常發布目標座標。

---

# 📡 ROS 2 介面與資料格式

## 📤 Publisher

本節點會發布目標的 3D 空間座標。

| 項目           | 內容                        |
| ------------ | ------------------------- |
| Topic Name   | `/goose_target_position`  |
| Message Type | `geometry_msgs/msg/Point` |

---

## 📊 欄位定義與座標系統

本節點發布的座標系以**機器人車體中心／基準點**為參考。

計算過程會包含：

```text
robo_offset
```

進行相機安裝位置造成的座標位移補償。

### 座標定義

| 欄位名稱 | 型態        | 單位    | 說明                         |
| ---- | --------- | ----- | -------------------------- |
| `x`  | `float64` | 公尺（m） | 機器人前方的直線距離，正值代表前方          |
| `y`  | `float64` | 公尺（m） | 機器人左右方向的偏離距離，正值代表偏左、負值代表偏右 |
| `z`  | `float64` | 公尺（m） | 目標相對於相機地面的高度計算值，通常供參考使用    |

---

# 📐 座標方向示意

```text
                  前方
                   ↑
                   │
                   │  +X
                   │
             Target ●
                   │
                   │
        +Y ←──── Robot ────→ -Y
              車體中心
```

因此：

```text
X > 0  → 目標在機器人前方

Y > 0  → 目標位於機器人左側

Y < 0  → 目標位於機器人右側
```

---

# 🔍 查看 ROS 2 Topic

如果想在命令列即時查看節點發布的座標，可以使用：

```bash
ros2 topic echo /goose_target_position
```

輸出格式會類似：

```text
x: 1.25
y: 0.35
z: 0.00
```

代表：

```text
X = 1.25 m
Y = 0.35 m
Z = 0.00 m
```

也就是目標位於：

* 機器人前方約 **1.25 公尺**
* 機器人左側約 **0.35 公尺**

---

# 🧪 除錯與測試建議

正式比賽前，可以依照以下流程進行測試。

## 1. 確認 RealSense 正常運作

確認相機可以正常取得：

* RGB Image
* Depth Image
* Camera Intrinsics

---

## 2. 確認 YOLO 偵測結果

開啟：

```python
self.enable_debug_view = True
```

觀察 Bounding Box 是否正確。

---

## 3. 確認座標計算

使用：

```bash
ros2 topic echo /goose_target_position
```

確認目標移動時：

```text
目標向前 → X 增加

目標向左 → Y 增加

目標向右 → Y 減少
```

---

## 4. 確認座標穩定性

讓目標停在固定位置，觀察輸出的：

```text
x
y
z
```

是否會產生明顯跳動。

如果仍有明顯抖動，可以檢查：

* RealSense 深度品質
* YOLO Bounding Box 穩定度
* Moving Average Buffer 大小
* 信心度門檻
* 邊緣過濾條件

---

# 🏁 比賽部署 Checklist

正式上場前建議確認以下項目：

* [ ] RealSense 可以正常取得 RGB / Depth
* [ ] YOLO `best.pt` 權重檔已放置正確
* [ ] ROS 2 Package 編譯成功
* [ ] `ros2 run obj_detect_node detect_node` 可以正常啟動
* [ ] `ros2 run realsense2_camera realsense2_camera_node --ros-args -p align_depth.enable:=true` 可以正常啟動
* [ ] YOLO 偵測結果正常
* [ ] 目標深度資料正常
* [ ] 3D 座標方向正確
* [ ] `robo_offset` 已校正
* [ ] Moving Average Filter 運作正常
* [ ] `/goose_target_position` Topic 正常發布
* [ ] Debug View 已關閉
* [ ] CPU 使用率與推論 FPS 符合比賽需求

---

# 📌 快速使用

如果環境都已經設定完成，可以直接：

```bash
cd ~/your_ws

colcon build --packages-select obj_detect_node

source install/setup.bash

ros2 run obj_detect_node detect_node
```

另一個 Terminal 查看目標座標：

```bash
ros2 topic echo /goose_target_position
```

---

# 📜 License

本專案可依實際競賽與團隊需求自行補充 License 資訊。
