# Mobile-swarm
## Cooperative Control of Multiple Mobile Robots Based on Co-VINS

Based on  [Co-VINS](https://github.com/qintonguav/Co-VINS) , this repository extends  [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) to swarm for  cooperative location and formation control of multi-robot system. And use  [iBoW](https://github.com/emiliofidalgo/ibow-lcd) instead of  [DBoW2](https://github.com/dorian3d/DBoW2)  for online loop detection.

## Prerequisites 

Testing environment: Ubuntu 18.04, ROS Melodic, OpenCV 3.4.0, Eigen 3.3.3

**1.1** **Ubuntu** and **ROS**
Ubuntu  18.04, ROS Melodic,  [ROS Installation](http://wiki.ros.org/ROS/Installation) 

Additional ROS pacakge

```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```

**1.2 Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.
**1.3 Pangolin**

Use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## Acknowledgements
This repository use [Co-VINS](https://github.com/qintonguav/Co-VINS) and [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) for collaborative localization for multiple monocular visual-inertial systems and [iBoW-LCD](https://github.com/emiliofidalgo/ibow-lcd) for incremental loop detection to improve the existing [DBoW2](https://github.com/dorian3d/DBoW2) method, and a laser radar drive [rplidar_ros](https://github.com/robopeak/rplidar_ros).

## License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.