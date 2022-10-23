#include <ros/ros.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>

#include <iostream>
#include <cmath>
#include <string>

using Eigen::Vector3d;
using Eigen::Quaterniond;

// 全局变量
uint8_t sendBuffer[32];
tf::TransformListener* tfListener;

static void bufferConvert(uint8_t* buffer, int data0, int data1)
{
    uint16_t _x_abs = abs(data0);
    uint16_t _y_abs = abs(data1);

    buffer[0] = ((data0 < 0) ? 0x01 : 0x00);
    buffer[1] = _x_abs / 256;       // x坐标高8位 单位mm
    buffer[2] = _x_abs % 256;       // x坐标低8位
    buffer[3] = ((data1 < 0) ? 0x01 : 0x00);
    buffer[4] = _y_abs / 256;       // y坐标高8位
    buffer[5] = _y_abs % 256;       // y坐标低8位
}

static void tfTransfer(const nav_msgs::Odometry::ConstPtr& msg, double& outputX, double& outputY)
{
    tf::StampedTransform transform;
    int sequence = std::stoi(msg->child_frame_id);
    Quaterniond q(msg->pose.pose.orientation.w,
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z);
    Vector3d t(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    try{
        tfListener->lookupTransform("/global", "/drone_" + std::to_string(sequence),
                                ros::Time(0), transform);

        tf::Vector3 tf_t = transform.getOrigin();
        tf::Quaternion tf_q = transform.getRotation();
        
        Vector3d w_T_local = Vector3d(tf_t.x(), tf_t.y(), tf_t.z());
        geometry_msgs::Quaternion g_Q;
        tf::quaternionTFToMsg(tf_q, g_Q);   
        Quaterniond w_Q_local(g_Q.w, g_Q.x, g_Q.y, g_Q.z);

        q = w_Q_local * q;
        t = w_Q_local * t + w_T_local;
    }
    catch (tf::TransformException &ex) {
        //ROS_WARN("no %d transform yet", sequence);
    }
    outputX = t.x();
    outputY = t.y();
}

void path1Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double _pos_x = 0.0, _pos_y = 0.0;
    tfTransfer(msg, _pos_x, _pos_y);
    int _converted_x = (int)(_pos_x * -1000);
    int _converted_y = (int)(_pos_y * -1000);

    bufferConvert(sendBuffer, _pos_x, _pos_y);
}

void path2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double _pos_x = 0.0, _pos_y = 0.0;
    tfTransfer(msg, _pos_x, _pos_y);
    int _converted_x = (int)(_pos_x * -1000);
    int _converted_y = (int)(_pos_y * -1000);

    bufferConvert(sendBuffer + 6, _pos_x, _pos_y);
}

void path3Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double _pos_x = 0.0, _pos_y = 0.0;
    tfTransfer(msg, _pos_x, _pos_y);
    int _converted_x = (int)(_pos_x * -1000);
    int _converted_y = (int)(_pos_y * -1000);
    
    bufferConvert(sendBuffer + 12, _pos_x, _pos_y);
}

void obstCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
    geometry_msgs::Point32 _obstaction = *msg; 
    // 障碍物坐标系和里程计坐标系相差90度
    int _obs_x = -1 * (int)(_obstaction.y * 1000.0);
    int _obs_y = (int)(_obstaction.x * 1000.0);
    
    bufferConvert(sendBuffer + 18, _obs_x, _obs_y);
}

void goalCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std_msgs::Float32MultiArray _goal = *msg;
    int _pos_x = (int)(_goal.data[0] * 1000);
    int _pos_y = (int)(_goal.data[1] * 1000);

    bufferConvert(sendBuffer + 24, _pos_x, _pos_y);
}
void debugInfoPrint(void)
{
    int _coor_x, _coor_y;
    int _obs_x, _obs_y;

    _coor_x = sendBuffer[13] * 256 + sendBuffer[14];
    _coor_y = sendBuffer[16] * 256 + sendBuffer[17];
    _obs_x = sendBuffer[19] * 256 + sendBuffer[20];
    _obs_y = sendBuffer[22] * 256 + sendBuffer[23];

    _coor_x = ((sendBuffer[12] == 0x01) ? -_coor_x : _coor_x);
    _coor_y = ((sendBuffer[15] == 0x01) ? -_coor_y : _coor_y);
    _obs_x = ((sendBuffer[18]  == 0x01) ? -_obs_x : _obs_x);
    _obs_y = ((sendBuffer[21] == 0x01) ? -_obs_y : _obs_y);
    ROS_INFO("x : %d, y : %d, obs_x : %d, obs_y : %d", _coor_x,
        _coor_y, _obs_x, _obs_y);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;
    ros::NodeHandle nh_private("~");
    bool print_debug;
    nh_private.getParam("print_debug", print_debug);

    tfListener = new tf::TransformListener(n);

    // serial相关
    serial::Serial sp;                  // 创建一个serial类
    serial::Timeout to = serial::Timeout::simpleTimeout(30);   // 创建timeout
    sp.setPort("/dev/stm32board");      // 设置要打开的串口名称
    sp.setBaudrate(115200);             // 设置串口通信的波特率
    sp.setTimeout(to);                  // 串口设置timeout
 
    try {
        sp.open();
    } catch(serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    //判断串口是否打开成功
    if(sp.isOpen()) {
        ROS_INFO_STREAM("/dev/stm32board is opened.");
    } else {
        return -1;
    }
    sendBuffer[30] = 0x0d;
    sendBuffer[31] = 0x0a;

    ros::Subscriber subPath_1 = n.subscribe("/vins_1/vins_estimator/odometry", 1000, path1Callback);
    ros::Subscriber subPath_2 = n.subscribe("/vins_2/vins_estimator/odometry", 1000, path2Callback);
    ros::Subscriber subPath_3 = n.subscribe("/vins_3/vins_estimator/odometry", 1000, path3Callback);
    ros::Subscriber subObst = n.subscribe("obs_coor", 1000, obstCallback);
    ros::Subscriber subGoal = n.subscribe("/target_pos", 100, goalCallback);
    ROS_INFO_STREAM("Node \"car_serial\" initial successful !");

    ros::Rate loop_rate(20);        // 以20Hz频率向下发送更新的数据
    int loop_rate_count = 0;
    while(ros::ok()){
        sp.write(sendBuffer, 32);
        if (print_debug) {          // 1s打印一次调试信息
            if (loop_rate_count >= 20) {
                debugInfoPrint();
                loop_rate_count = 0;
            }
            loop_rate_count++;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    //关闭串口
    sp.close();
 
    return 0;
}

