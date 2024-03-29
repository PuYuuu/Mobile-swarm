#pragma once
#ifndef __AGENT_H
#define __AGENT_H

#include "common_include.h"
#include <mutex>


class Agent {
public:
    Agent(string ns, ros::NodeHandle* n);
    ~Agent();

    // void registerSub(ros::NodeHandle& n);
    string getAgentName(void);
    vector<vector<float>> getLaserData(void);
    vector<float> getObsCoor(void);
    vector<float> getBotCoor(void);
    vector<vector<float>> getBotPath(void);
    cv::Mat getFeatureImg(void);

    // 订阅图像和点云话题占用大量带宽，故在不显示时关闭订阅，
    // 节省带宽，保证关键信息传输的实时性
    // void startSubPath(void);
    // void stopSubPath(void);
    // void startSubObs(void);
    // void stopSubObs(void);
    void startSubLaser(void);
    void stopSubLaser(void);
    void startSubImg(void);
    void stopSubImg(void);
    
    
private:
    ros::NodeHandle* _n;

    string _agentname;                      // 订阅的path话题名
    string _subOdomStr;
    string _subPathStr;     
    string _subLaserStr;
    string _subObsStr;
    string _subImgStr;

    bool _isSubPath  = false;
    bool _isSubLaser = false;
    bool _isSubObs   = false;
    bool _isSubImg   = false;

    ros::Subscriber _subOdom;
    ros::Subscriber _subPath;
    ros::Subscriber _subLaser;
    ros::Subscriber _subObs;
    ros::Subscriber _subImg;
    tf::TransformListener* _listener;

    std::mutex pathData_mutex_;             // 数据锁
    std::mutex laserData_mutex_;
    std::mutex obsData_mutex_;
    std::mutex botData_mutex_;
    std::mutex imgData_mutex_;

    vector<vector<float>> laserData;            
    vector<float> obsCoor = vector<float>(2);
    vector<float> botCoor = vector<float>(3, 0);
    vector<vector<float>> botPath;
    cv::Mat featureImg;

    
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void imgCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void obsCallback(const geometry_msgs::Point32::ConstPtr& msg);
};


#endif

