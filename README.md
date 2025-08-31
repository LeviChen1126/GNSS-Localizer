# GNSS_Localizer

> This project generates `/gnss_pose` from GNSS/NMEA data, transforms it into the map frame, and can be integrated with modules such as `ndt_matching`.  

---

## Introduction

### 1. `nmea2tfpose`
- Subscribes: `/nmea_sentence` (from serial GNSS receiver)  
- Publishes: `/gnss_pose` (`geometry_msgs/PoseStamped`, frame = `map`)  
- Key modifications:  
  - **Supports GNRMC messages** and integrates `yaw_` (heading).  
  - Supports plane selection (Plane 13 = NTUT Ningxin Park in Taipei).  

### 2. `fix2tfpose`
- Subscribes: `/fix` (`sensor_msgs/NavSatFix`)  
- Publishes: `/gnss_pose`  
- Function: Converts GNSS Fix directly into pose in the map frame.  

### 3. `geo_pos_conv`
Converts latitude/longitude (WGS84) → local Cartesian coordinates (x,y,z).  
- **`geo_pos_conv_SPIE.cpp`**  
  - Implements the **complete Transverse Mercator projection** (rigorous mathematical formulas for converting from the Earth ellipsoid to a planar Cartesian coordinate system).  
  - High accuracy, suitable for large-area use.  

- **`geo_pos_conv_mydicoor.cpp`**  
  - Uses a **linear approximation**:  
    ```cpp
    m_x = (m_lat - m_PLato) * 110773.52...
    m_y = (m_lon - m_PLo) * 100914.35...
    m_z = 0
    ```
  - Suitable for small local areas (e.g., campus/test field), with lower computation cost.  
  - Limitation: Errors increase with distance from the origin.  

**Recommendation**:  
- Large-scale, multi-location tests → use `SPIE`  
- Small-area quick demos → `mydicoor` is sufficient  

---

## Installation / Usage Examples
> ⚠️ Note: Please rename the corresponding `CMakeLists.*.txt` file to `CMakeLists.txt` before building.

### ROS (catkin)
```bash
mv CMakeLists.catkin.txt CMakeLists.txt       # rename
# package.xml must also be placed in the root directory
cd ~/catkin_ws/src
ln -s /path/to/GNSS-Localizer .
cd ..
catkin_make
source devel/setup.bash
rosrun gnss_localizer nmea2tfpose
```

### CMake (Only for Demo)
```bash
mv CMakeLists.cmake.txt CMakeLists.txt   # rename
mkdir -p build && cd build
cmake ..
make -j
./gnss_localizer
```

---

## Integration with Autoware

- ### RQT graph
  ```
  /nmea_sentence → nmea2tfpose → /gnss_pose (map frame) → ndt_matching
  ```

- ### Setup steps
  1. Launch the Serial GNSS node that publishes `/nmea_sentence`.  
  2. Launch the `nmea2tfpose` node with parameter `plane: 13` (Ningxin Park).  
  3. In `ndt_matching`:  
     - Set the initial pose (`initial_pos`) to `gnss`.  
     - Adjust the `fitness_score` threshold to **1.0** (default 500 is not suitable for RTK-GPS precision).  
  4. Use `rqt_tf_tree` to verify the TF structure.  

---

## Project Structure
```bash
GNSS-Localizer/
├── .gitattributes
├── .gitignore
├── LICENSE
├── CMakeLists.catkin.txt
├── CMakeLists.cmake.txt
├── package.xml
├── gnss/
│   ├── geo_pos_conv_mydicoor.cpp    # Simplified local coordinate conversion
│   └── geo_pos_conv_SPIE.cpp        # Accurate projection-based conversion
└── gnss_localizer/
    ├── fix2tfpose/
    │   └── fix2tfpose.cpp           # GNSS Fix → TF Pose
    └── nmea2tfpose/
        └── nmea2tfpose_core_SPIE.cpp # NMEA Sentence → Pose
```

---

## License and Acknowledgment
- [MIT License](./LICENSE)  
- [Autoware](https://www.autoware.org/)
