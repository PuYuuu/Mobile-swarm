#pragma once
#ifndef __COMMON_INCLUDE_H
#define __COMMON_INCLUDE_H

// std
#include <cmath>
#include <list>
#include <vector>
#include <string>
#include <thread>
#include <unistd.h>
#include <algorithm>
#include <unordered_map>
#include <iostream>

// for ros
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>

// for opencv
#include <opencv2/core/core.hpp>

// for pangolin
#include <pangolin/pangolin.h>

using std::vector;
using std::string;
using std::list;
using std::unordered_map;
using std::to_string;

#define inf std::numeric_limits<double>::infinity() 

#endif

