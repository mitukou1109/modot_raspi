# modot_raspi

ROS 2 package that runs on Raspberry Pi on MODOT

## Setup & launch

### Requirements

- ROS 2 Humble

```
$ mkdir -p ~/modot_ws/src
$ cd modot_ws
$ source <(wget -O - https://raw.githubusercontent.com/mitukou1109/modot_raspi/master/setup.sh)
$ python3 src/modot/face_recognition_ros/register_known_face.py <path to image> [name]
$ ros2 launch modot_raspi bringup.launch
```
