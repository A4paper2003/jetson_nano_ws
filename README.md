## Packages

Please note lidar driver is not is this workspace, you have to clone ws_livox if you want to run SLAM.

* `mvs_ros_driver`: camera driver
* `gps_rtk-main`: gps driver with rtk
* `FAST-LIVO2`: SLAM algorithm
* `rpg_vikit`: library that provides essential computer vision tools, particularly for camera geometry and 3D transformations


## How to Build
To compile jetson_nano_ws with fast_livo2:
1. **Source the lidar driver in the other workspace:**
    ```bash
    source ~/ws_livox/devel/setup.bash
    ```
    
2. **Check if ws_livos is properly sourced:** 
    ```bash
    echo $CMAKE_PREFIX_PATH
    ```
    
3. **Go to your main workspace:** 
    ```bash
    cd jetson_nano_ws/
    ```
    
4. **Export Library required for FAST-LIVO2:** 
    ```bash
    export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
    ```
    
3. **Build the workspace with specific opencv version:** 
    ```bash
    catkin_make -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4
    ```



