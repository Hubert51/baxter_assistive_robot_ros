# Baxter Assistive Robot
This is the project to use baxter as an assistive robot to help people at home

## ROS Buildfarm
Baxter assistive Package | Indigo(Ubuntu 14.04) | Kinetic(Ubuntu 16.04) | Melodic(Windows Subsystem Linux)
--------------- | ------------- | ------------- | ------------- 
moveit | ![Build Status](https://img.shields.io/badge/build-success-brightgreen.svg) | ![Build Status](https://img.shields.io/badge/build-failing-red.svg) | ![Build Status](https://img.shields.io/badge/build-failing-red.svg)

## Compatibility Notes
### Move from Indigo(14.04) to Kinetic(16.04)
* `/usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at "BOOST_JOIN"`  
Refer [this](https://answers.ros.org/question/233786/parse-error-at-boost_join/) link to solve this error

* In Kinetic environment, we need to use c++11 to compile the code.  
Adding `add_compile_options(-std=c++11)` in CMakeLists.txt can solve compile error

* For building this project, we need `gazebo_ros_pkgs` and `gazebo_ros_control`. Normally, refering [gazebo website](http://gazebosim.org/tutorials?tut=ros_installing#B.InstallfromSource(onUbuntu)) can install the packages. For
some reasons `sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control` does not work.  
Giving the error `Err:1 http://packages.ros.org/ros/ubuntu xenial/main amd64 ros-kinetic-gazebo-ros-control amd64 2.5.18-1xenial-20190320-175439-0800 404  Not Found [IP: 64.50.236.52 80]`  
**Solution**: install this package from source, refering this link. And then copy the source file into ros system folder  
`cp ~/ros_ws/src/gazebo_ros_pkgs/gazebo_ros_control/include/gazebo_ros_control_plugin.h /opt/ros/kineticc/include/gazebo_ros_control/gazebo_ros_control_plugin.h`  
and  
`cp ~/ros_ws/src/gazebo_ros_pkgs/gazebo_ros_control/include/robot_hw_sim.h /opt/ros/kineticc/include/gazebo_ros_control/robot_hw_sim.h`  

<!-- for substitution -->
<!-- [![Build Status](https://travis-ci.org/ros-planning/moveit.svg?branch=indigo-devel)](https://travis-ci.org/ros-planning/moveit/branches) -->
