# Mobile-swarm
## Cooperative Control of Multiple Mobile Robots Based on CO-VINS




## Prerequisites 
1.1 **Ubuntu** and **ROS**
Ubuntu  18.04.
ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
additional ROS pacakge

```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```

1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.
(Testing environment: Ubuntu 18.04, ROS Melodic, OpenCV 3.3.1, Eigen 3.3.3) 

## Acknowledgements
This repository use [Co-VINS](https://github.com/qintonguav/Co-VINS) for collaborative localization for multiple monocular visual-inertial systems and [iBoW-LCD](https://github.com/emiliofidalgo/ibow-lcd) for incremental loop detection to improve the existing DBOW2 method, and a laser radar drive [rplidar_ros](https://github.com/robopeak/rplidar_ros).

## License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.