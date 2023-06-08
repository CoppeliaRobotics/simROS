# ROS Interface plugin for CoppeliaSim

### Compiling

_NOTE:_ the directory containing all files (i.e. package.xml etc) must be called sim_ros_interface, otherwise build will fail.

1. Install required packages for simStubsGen: see simStubsGen's [README](https://github.com/CoppeliaRobotics/include/blob/master/simStubsGen/README.md)
2. Checkout
```
$ git clone https://github.com/CoppeliaRobotics/simROS.git sim_ros_interface
$ cd sim_ros_interface
$ git checkout coppeliasim-v4.5.0-rev0
```

NOTE: replace `coppeliasim-v4.5.0-rev0` with the actual CoppeliaSim version you have.

3. Edit `meta/messages.txt` and `meta/services.txt` if you need to include more ROS messages/services. You need to specify the full message/service type, i.e. geometry_msgs/Twist rather than Twist.
4. Compile
```
$ catkin build
```

For ROS melodic / Ubuntu 18, use the 'melodic' branch
