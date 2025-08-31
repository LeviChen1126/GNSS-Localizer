# GNSS_Localizer

[English Version](./README.md)

> 此專案自 GNSS/NMEA 資料產生 `/gnss_pose`，並轉換到地圖座標系 (map frame)，可與 `ndt_matching` 等套件結合。  

---

## 介紹

### 1. `nmea2tfpose`
- 訂閱：`/nmea_sentence`（序列GNSS接收器輸出）  
- 輸出：`/gnss_pose`（`geometry_msgs/PoseStamped`，frame = `map`）  
- 修改重點：  
  - **支援 GNRMC 訊息**，並導入 `yaw_`（航向角）。  
  - 可設定起始原點（Plane 13 = 臺北北科大寧馨公園水準點）。  

### 2. `fix2tfpose`
- 訂閱：`/fix`（`sensor_msgs/NavSatFix`）  
- 輸出：`/gnss_pose`  
- 功能：將 GNSS Fix 直接轉換成地圖座標系下 Pose。  

### 3. `geo_pos_conv`
提供經緯度 (WGS84) → 本地平面座標 (x,y,z) 的轉換。  
- **`geo_pos_conv_SPIE.cpp`**
  - 採用**完整橫麥卡托投影公式**（從地球橢球體轉換到平面直角坐標的嚴謹數學公式），精度高，適用於大範圍移動。    

- **`geo_pos_conv_mydicoor.cpp`**
  - 使用**線性近似法**：  
    ```
    m_x = (m_lat - m_PLato) * 110773.52...
    m_y = (m_lon - m_PLo) * 100914.35...
    m_z = 0
    ```
  - 適合在特定原點附近（如校園/測試場）快速換算，計算量小。  
  - 缺點：離原點越遠誤差越大。  

**選用建議**：  
- 長距離、多地點測試 → 用 `SPIE`  
- 小區域、快速 Demo → 可用 `mydicoor`  

### RQT關係
![](./images/rqt_graph.png)

---

## 開發環境
- **CMake** 3.10+  
- **C++17** 編譯器  
- （ROS1 建置時需安裝）`roscpp`, `sensor_msgs`, `geometry_msgs`, `tf`, `nmea_msgs`

---

## 安裝 / 使用範例
> ⚠️ 注意：請依需求將對應的 `CMakeLists.*.txt` 檔案重新命名為 `CMakeLists.txt`。

### ROS (catkin)
```bash
mv CMakeLists.catkin.txt CMakeLists.txt       # 重新命名
# package.xml 需放在根目錄
cd ~/catkin_ws/src
ln -s /path/to/GNSS-Localizer .
cd ..
catkin_make
source devel/setup.bash
rosrun gnss_localizer nmea2tfpose
```

### CMake (僅 Demo)
```bash
mv CMakeLists.cmake.txt CMakeLists.txt   # 重新命名
mkdir -p build && cd build
cmake ..
make -j
./gnss_localizer
```

---

## 與 Autoware 整合

- ### 設定步驟
  1. 啟動 Serial GNSS 節點，發布 `/nmea_sentence`。  
  2. 啟動 `nmea2tfpose` 節點，參數：`plane: 13`（北科寧馨公園基準點）。  
  3. 在 `ndt_matching`：
     - 初始姿態 (`initial_pos`) 設定為 `gnss`。  
     - `fitness_score` 門檻調整為 **1.0**（預設 500 不適用 RTK-GPS 精度）。  
  4. 使用 `rqt_tf_tree` 檢查 TF 架構是否正確。  

---

## 專案結構
```bash
GNSS-Localizer/
├── .gitattributes
├── .gitignore
├── LICENSE
├── CMakeLists.catkin.txt
├── CMakeLists.cmake.txt
├── package.xml
├── images
│   └── rqt_graph.png
├── gnss/
│   ├── geo_pos_conv_mydicoor.cpp    # 在地簡化座標轉換
│   └── geo_pos_conv_SPIE.cpp        # 精確投影公式座標轉換
└── gnss_localizer/
    ├── fix2tfpose/
    │   └── fix2tfpose.cpp           # GNSS Fix → TF Pose
    └── nmea2tfpose/
        └── nmea2tfpose_core_SPIE.cpp # NMEA Sentence → Pose
```

---

## 授權與致謝
- [MIT License](./LICENSE)  
- [Autoware](https://www.autoware.org/)