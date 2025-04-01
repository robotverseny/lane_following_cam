# `lane_following_cam` package
Lane following based on camera as a ROS 2 python package.  

[![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)
[![Static Badge](https://img.shields.io/badge/ROS_2-Jazzy-34aec5)](https://docs.ros.org/en/jazzy/)

## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages
``` r
cd ~/ros2_ws/src
```
``` r
git clone https://github.com/kopaj/savtartas
```

### Build ROS 2 packages
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select lane_following_cam --symlink-install
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash![readme](https://github.com/user-attachments/assets/a306b813-e863-478f-b4dd-1a01c30e439b)

source ~/ros2_ws/install/setup.bash
```
</details>

## Run the package

Start the lane detection node with **compressed** or **raw** image, e.g.:

``` r
ros2 run lane_following_cam lane_detect --ros-args -p image_topic:=/image_raw/compressed -p raw_image:=false
```

``` r
ros2 run lane_following_cam lane_detect --ros-args -p image_topic:=/image_raw -p raw_image:=true
```

``` r
ros2 run lane_following_cam lane_detect --ros-args -p image_topic:=/camera/color/image_raw -p raw_image:=true
```

There are launch files as well: 

``` r
ros2 launch lane_following_cam example_bag.launch.py
```

``` r
ros2 launch lane_following_cam robot_raw1.launch.py
```

``` r
ros2 launch lane_following_cam robot_compressed1.launch.py
```


### There are aliases for the shell scripts 
Start the camera
``` r 
start_drivers
```
Start the lane detection
``` r 
start_lane
```
Shut down everything
``` r 
stop_all
```
<details>
<summary> Don't forget to source them</summary>

``` bash![readme](https://github.com/user-attachments/assets/a306b813-e863-478f-b4dd-1a01c30e439b)

source ~/ros2_ws/src/savtartas/shell/$NAME.sh
```
</details>

### Use ROS 2 bags (mcap)

Link for bags: [https://1drv.ms/f/s!Ao2v58VBa73_a1e1dBA9IAPirxo?e=VeP7gE](https://1drv.ms/f/s!Ao2v58VBa73_a1e1dBA9IAPirxo?e=VeP7gE)

```r
ros2 bag play runde_vdi_lausitz_1.mcap --loop
```

```r
ros2 bag play big_track_munchen_only_camera_a.mcap --loop
```
```r
ros2 bag play filtered_bag.mcap_.mcap --loop
```

These bag files contain `/camera/color/image_raw` topic, so easiest way to use them is:

**For runde:**
``` r
ros2 launch lane_following_cam example_bag.launch.py brightness:=125 saturation:=10 multiplier_bottom:=0.8 multiplier_top:=0.65 divisor:=7.5 cam_align:=-50
```
**For big_track_munchen:**
``` r
ros2 launch lane_following_cam robot_compressed1.launch.py brightness:=-10 saturation:=10 multiplier_bottom:=1.0 multiplier_top:=0.45 divisor:=5.0
```
**For filtered_bag:**
```r
ros2 launch lane_following_cam robot_compressed1.launch.py multiplier_bottom:=1.0 multiplier_top:=0.65 divisor:=5.0 islane:=false
```
## Displaying the lane detection using foxglove bridge

```r
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

It is recommended to loop the bag, using the -l parameter

```r
ros2 bag play runde_vdi_lausitz_1.mcap -l
```
This recording was made on the Runde mcap


![readme](https://github.com/user-attachments/assets/b837cc85-3c19-4c08-9b2f-ddbe1032c89f)
