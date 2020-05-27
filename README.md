# ROS Interface plugin for CoppeliaSim

### Compiling

_NOTE:_ the directory containing all files (i.e. package.xml etc) must be called sim_ros_interface, otherwise build will fail.

1. Install required packages for [libPlugin](https://github.com/CoppeliaRobotics/libPlugin): see libPlugin's [README](external/libPlugin/README.md)
2. Checkout
```
$ git clone --recursive https://github.com/CoppeliaRobotics/simExtROSInterface.git sim_ros_interface
```
3. Edit `meta/messages.txt` and `meta/services.txt` if you need to include more ROS messages/services. You need to specify the full message/service type, i.e. geometry_msgs/Twist rather than Twist.
4. Compile
```
$ catkin build
```

Notes for building on Ubuntu 18.04 / ROS Melodic:

 - install cmake 3.16 using Kitware's apt repository
 - pass option `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python2 --` to `catkin build`
