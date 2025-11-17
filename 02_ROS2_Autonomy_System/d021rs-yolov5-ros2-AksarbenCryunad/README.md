# YOLOv5 ROS2

## Set up
```
python3 -m venv .venv
source .venv/bin/activate
(.venv) pip install -r requirements.txt
```

## Build ROS2 workspace
```
(.venv) cd ros2_ws
(.venv) colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$(which python3)" -DPYTHON_EXECUTABLE="$(which python3)"
```

## Run YOLOv5_ROS2
* Terminal 1
```
(.venv) source ./install/setup.bash
(.venv) ros2 run image_tools cam2image
```

* Terminal 2
```
(.venv) source ./install/setup.bash
(.venv) ros2 run yolov5_ros2 yolo_detect \
  --ros-args \
  -p device:=cpu \
  -p pub_result_img:=true \
  -p image_topic:=/image
```
