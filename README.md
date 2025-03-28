## Ball tracker for NAO


### Install

To use a web-cam you need to install: 

```
sudo apt install ros-humble-usb-cam
```
Then you can run a topic called "image_raw" using the camera

```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video0"
```

### Usage

```
cd ~/ros2_ws/src
-- put the ball_tracker inside of this folder.
colcon build
source install/setup.bash
```

### Basket
```
ros2 run ball_tracker basket_nao --ros-args -p ball_color:=red -p image_topic:=/camera/bottom/image_raw
```

### Kick
```
ros2 run ball_tracker kick_nao --ros-args -p ball_color:=red -p image_topic:=/camera/bottom/image_raw
```

### Testing
```
ros2 run ball_tracker basket_nao --ros-args -p ball_color:=red -p image_topic:=/image_raw
ros2 run ball_tracker kick_nao --ros-args -p ball_color:=red -p image_topic:=/image_raw

```