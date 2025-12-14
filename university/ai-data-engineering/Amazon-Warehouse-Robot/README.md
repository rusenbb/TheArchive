# README for hakanserdarrusen

## Package Name
hakanserdarrusen

## Assumption
We assume that you have the docker container that is given in this repo.

## Setup Environment
(Note: This may be a little bit time-consuming due to torch and nvidia_cuxx installations, be patient.)

```bash
pip install 'numpy<2.0' ultralytics==8.3.26
```

**WARNING:** If you would try to use different numpy version or a ultralytics version, ros2 humble becomes unstable. Please use the versions that we give above.

## Installation Steps

1. Place src folder and best.pt into the ros2_ws folder.

2. Set up the environment variables by running the following commands in your terminal:
```bash
export GAZEBO_MODEL_PATH=/root/ros2_ws/src/hakanserdarrusen/models:$GAZEBO_MODEL_PATH
export TURTLEBOT3_MODEL=waffle
```

3. ros2_ws has the weights for YOLO model. We need to transfer it to root folder. You may manually download the weights and place it onto root or do the following. 
4. cd ~/ros2_ws 
5. mv best.pt /root/

## Running the Application

1. First, launch the environment and wait a few seconds:
```bash
colcon build && source install/setup.bash && ros2 launch hakanserdarrusen warehouse_launch.py
```

2. Then run our robot:
```bash
ros2 run hakanserdarrusen warehouseboxrobot
```

## Resources
1. You can find the required model weights and annotated data in this [Google Drive folder](https://drive.google.com/drive/folders/1_lEBpiWORVqi9gzEKz2abHea5l792-ow?usp=drive_link)

2. Project Video: https://youtu.be/h-Jxs_y9kNA

