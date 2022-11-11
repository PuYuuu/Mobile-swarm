#include "../include/agent.h"


unordered_map<string, string> subTopicPair = {{"vins_0","path_0"}, {"vins_1","path_1"}, {"vins_2","path_2"},
        {"vins_3","path_3"},{"vins_4","path_4"},{"vins_5","path_5"},};
constexpr double DDPI = 180 / 3.1415926;

Agent::Agent(string ns, ros::NodeHandle* n)
{
    _n = n;
    _agentname = ns;
    _subPathStr = "/pose_graph/" + subTopicPair[ns];
    _subLaserStr = "/" + ns + "/scan";
    _subObsStr = "/" + ns + "/obs_coor";
    _subImgStr = "/" + ns + "/vins_estimator/image_track/compressed";
    _subOdomStr = "/" + ns + "/vins_estimator/odometry";

    // ROS_INFO_STREAM("\r\n\tAgent:" << ns 
    //     << "\r\n\tPath Topicname: \"" << _subPathStr
    //     << "\"\r\n\tLaser Topicname: \"" << _subLaserStr
    //     << "\"\r\n\tObstacle Topicname: \"" << _subObsStr
    //     << "\"\r\n\tFeatureImg Topicname:\"" << _subImgStr
    //     << "\"\r\n");

    // 保持订阅路径和障碍物话题
    _subOdom = (*_n).subscribe(_subOdomStr, 1000, &Agent::odomCallback, this);
    _subObs  = (*_n).subscribe(_subObsStr, 1000, &Agent::obsCallback, this);
    _subPath = (*_n).subscribe(_subPathStr, 1000, &Agent::pathCallback, this);

    _listener = new tf::TransformListener(*n);
}

Agent::~Agent()
{
    delete _listener;
}

// 注册ros节点的订阅函数
// void Agent::registerSub(ros::NodeHandle& n)
// {
//     _subPath = n.subscribe(_subPathStr, 1000, &Agent::pathCallback, this);
//     _subLaser = n.subscribe(_subLaserStr, 1000, &Agent::scanCallback, this);
//     _subObs = n.subscribe(_subObsStr, 1000, &Agent::obsCallback, this);
//     _subImg = n.subscribe(_subImgStr, 1000, &Agent::imgCallback, this);
// }

string Agent::getAgentName(void) 
{
    return _agentname;
}

vector<vector<float>> Agent::getLaserData(void)
{
    // 在unique_lock对象的声明周期内，它所管理的锁对象会一直保持上锁状态；
    // 而unique_lock的生命周期结束之后，它所管理的锁对象会被解锁
    std::unique_lock<std::mutex> lock_(laserData_mutex_);
    return laserData;
}

vector<float> Agent::getObsCoor(void)
{
    std::unique_lock<std::mutex> lock_(obsData_mutex_);
    return obsCoor;
}

vector<float> Agent::getBotCoor(void)
{
    std::unique_lock<std::mutex> lock_(botData_mutex_);
    return botCoor;
}

vector<vector<float>> Agent::getBotPath(void)
{
    std::unique_lock<std::mutex> lock_(pathData_mutex_);
    return botPath;
}

cv::Mat Agent::getFeatureImg(void)
{
    std::unique_lock<std::mutex> lock_(imgData_mutex_);
    return featureImg;
}

void Agent::startSubLaser(void) 
{
    if (!_isSubLaser) {
        _subLaser = (*_n).subscribe(_subLaserStr, 1000, &Agent::scanCallback, this);
        _isSubLaser = true;
        // ROS_INFO_STREAM("Agent:" << _agentname << " start to subscribe laser topic!");
    }
}

void Agent::stopSubLaser(void) 
{
    if (_isSubLaser) {
        _subLaser.shutdown();
        _isSubLaser = false;
        // ROS_INFO_STREAM("Agent:" << _agentname << " stop subscribing image topic!");
    }
}

void Agent::startSubImg(void) 
{
    if (!_isSubImg) {
        _subImg = (*_n).subscribe(_subImgStr, 1000, &Agent::imgCallback, this);
        _isSubImg = true;
        // ROS_INFO_STREAM("Agent:" << _agentname << " start to subscribe image topic!");
    }
}

void Agent::stopSubImg(void) 
{
    if (_isSubImg) {
        _subImg.shutdown();
        _isSubImg = false;
        // ROS_INFO_STREAM("Agent:" << _agentname << " stop subscribing image topic!");
    }
}

void Agent::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    std::unique_lock<std::mutex> lock_(pathData_mutex_);
    vector<float> _poseTmp(2, 0.f);
    
    botPath.clear();
    for (int i = 0; i < msg->poses.size(); ++i) {
        _poseTmp[0] = msg->poses[i].pose.position.x;
        _poseTmp[1] = msg->poses[i].pose.position.y;
        botPath.emplace_back(_poseTmp); 
    }
}

void Agent::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::unique_lock<std::mutex> lock_(botData_mutex_);

    tf::StampedTransform transform;
    int sequence = std::stoi(msg->child_frame_id);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z);
    Eigen::Vector3d t(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    try{
        _listener->lookupTransform("/global", "/drone_" + to_string(sequence),
                                ros::Time(0), transform);

        tf::Vector3 tf_t = transform.getOrigin();
        tf::Quaternion tf_q = transform.getRotation();
        
        Eigen::Vector3d w_T_local = Eigen::Vector3d(tf_t.x(), tf_t.y(), tf_t.z());
        geometry_msgs::Quaternion g_Q;
        tf::quaternionTFToMsg(tf_q, g_Q);   
        Eigen::Quaterniond w_Q_local(g_Q.w, g_Q.x, g_Q.y, g_Q.z);

        q = w_Q_local * q;
        t = w_Q_local * t + w_T_local;
    }
    catch (tf::TransformException &ex) {
        //ROS_WARN("no %d transform yet", sequence);
    }

    botCoor[0] = t.x();
    botCoor[1] = t.y();
    double siny_cosp = 2 * (q.w() * q.z()+ q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    botCoor[2] = std::atan2(siny_cosp, cosy_cosp) * 180 / M_PI + 90.0;
}

void Agent::imgCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    std::unique_lock<std::mutex> lock_(imgData_mutex_);

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    featureImg = cv_ptr->image;
}

void Agent::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // 激光雷达扫描一次的激光点数（(最大角度-最小角度)/单位角度 = 激光点的个数）
	// int _scanSize = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);
    static const int _scanSize = 360;           // 思岚A1雷达一圈360个点
    float _curPonitAngle = scan->angle_min;
    float _angle_increment = scan->angle_increment;
    std::unique_lock<std::mutex> lock_(laserData_mutex_);

    laserData.clear();
    for (int i = 0; i < _scanSize; ++i) {
        // 舍弃无效点
        if (scan->ranges[i] != inf) {
            vector<float> _ponitTmp(2);
            // 雷达坐标系和里程计坐标系相差90度
            _ponitTmp[0] = -scan->ranges[i] * sin(_curPonitAngle);
            _ponitTmp[1] =  scan->ranges[i] * cos(_curPonitAngle);
            laserData.emplace_back(_ponitTmp);
        }
        _curPonitAngle += _angle_increment;
    }
}

void Agent::obsCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
    std::unique_lock<std::mutex> lock_(obsData_mutex_);

    obsCoor[0] = -msg->y;
    obsCoor[1] = msg->x;
}

