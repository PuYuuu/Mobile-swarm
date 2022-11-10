#include <ros/ros.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>

#include <iostream>
#include <cmath>
#include <string>

using Eigen::Vector3d;
using Eigen::Quaterniond;

struct Ground {
    Ground() {
        x = 0;
        y = 0;
        yaw = 90;
    }
    Ground (int _x, int _y, int _yaw) {
        this->x = _x;
        this->y = _y;
        this->yaw = _yaw;
    }

    int x;
    int y;
    int yaw;
};

Ground Forma[][3] = { 
        {{0, 0, 0}, {-700, 0, 0}, {700, 0, 0}},
        {{0, 0, 0}, {0, -700, 0}, {0, -1400, 0}},
        {{0, 0, 0}, {-700, -700, 0}, {700, -700, 0}} 
    };

const int AGENT_NUM = 0;

uint8_t sendBuffer[6];
Ground gt_pos[3], obs_pos(INT_MAX, INT_MAX, 90), goal_pos;
int forma_type = 0;
bool advoidMode = true;
tf::TransformListener* tfListener;

static void tfTransfer(const nav_msgs::Odometry::ConstPtr& msg, double& outputX, double& outputY, double& outputZ)
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
    double siny_cosp = 2 * (q.w() * q.z()+ q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    outputZ = std::atan2(siny_cosp, cosy_cosp) * 180 / M_PI + 90.0;
}

void path1Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double _pos_x = 0.0, _pos_y = 0.0, _yaw = 90.0;
    tfTransfer(msg, _pos_x, _pos_y, _yaw);
    gt_pos[0].x = static_cast<int>(_pos_x * 1000);
    gt_pos[0].y = static_cast<int>(_pos_y * 1000);
    gt_pos[0].yaw = static_cast<int>(round(_yaw));
}

void path2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double _pos_x = 0.0, _pos_y = 0.0, _yaw = 90.0;
    tfTransfer(msg, _pos_x, _pos_y, _yaw);
    gt_pos[1].x = static_cast<int>(_pos_x * 1000);
    gt_pos[1].y = static_cast<int>(_pos_y * 1000);
    gt_pos[1].yaw = static_cast<int>(round(_yaw));
}

void path3Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double _pos_x = 0.0, _pos_y = 0.0, _yaw = 90.0;
    tfTransfer(msg, _pos_x, _pos_y, _yaw);
    gt_pos[2].x = static_cast<int>(_pos_x * 1000);
    gt_pos[2].y = static_cast<int>(_pos_y * 1000);
    gt_pos[2].yaw = static_cast<int>(round(_yaw));
}

void obstCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
    geometry_msgs::Point32 _obstaction = *msg; 
    // 障碍物坐标系和里程计坐标系相差90度
    obs_pos.x = static_cast<int>(_obstaction.x * 1000.0);
    obs_pos.y = static_cast<int>(_obstaction.y * 1000.0);
}

void goalCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std_msgs::Float32MultiArray _goal = *msg;
    goal_pos.x = static_cast<int>(_goal.data[0] * 1000);
    goal_pos.y = static_cast<int>(_goal.data[1] * 1000);
}

void orderCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "line") {
        forma_type = 0;
        ROS_WARN("Switch formation : LINE");
    } else if (msg->data == "column") {
        forma_type = 1;
        ROS_WARN("Switch formation : COLUMN");
    } else if (msg->data == "trig") {
        forma_type = 2;
        ROS_WARN("Switch formation : TRIANGLE");
    } else if (msg->data == "avoid-yes") {
        advoidMode = true;
        ROS_WARN("Open avoid mode");
    } else if (msg->data == "avoid-no") {
        advoidMode = false;
        ROS_WARN("Close avoid mode");
    }
}

// void numLimit(int& x, int& y, int amplitude)
// {
//     int __maximum = abs(x) > abs(y) ? abs(x) : abs(y);
//     if (__maximum > amplitude) {
//         x = x * amplitude / __maximum;
//         y = y * amplitude / __maximum;
//     } 
// }

void numLimit(int& x, int& y, int amplitude)
{
    int mag = static_cast<int>(sqrt(x * x + y * y));
    if (mag > amplitude) {
        x = x * amplitude / mag;
        y = y * amplitude / mag;
    }
}

int mysign(int num)
{
    if (num > 0) {
        return 1;
    } else if (num < 0) {
        return -1;
    }
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    tfListener = new tf::TransformListener(n);
    ros::NodeHandle nh_private("~");
    bool print_debug;
    nh_private.getParam("print_debug", print_debug);

    // serial相关
    serial::Serial sp;                  // 创建一个serial类
    serial::Timeout to = serial::Timeout::simpleTimeout(30);   // 创建timeout
    sp.setPort("/dev/stm32board");      // 设置要打开的串口名称
    sp.setBaudrate(9600);               // 设置串口通信的波特率
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
    sendBuffer[0] = 0x3d;
    sendBuffer[4] = 0x0d;
    sendBuffer[5] = 0x0a;

    ros::Subscriber subPath_1 = n.subscribe("/vins_1/vins_estimator/imu_propagate", 1000, path1Callback);
    ros::Subscriber subPath_2 = n.subscribe("/vins_2/vins_estimator/imu_propagate", 1000, path2Callback);
    ros::Subscriber subPath_3 = n.subscribe("/vins_3/vins_estimator/imu_propagate", 1000, path3Callback);
    ros::Subscriber subObst = n.subscribe("obs_coor", 1000, obstCallback);
    ros::Subscriber subGoal = n.subscribe("/target_pos", 100, goalCallback);
    ros::Subscriber subOrder = n.subscribe("/key_order", 100, orderCallback);
    ROS_INFO_STREAM("Node \"controller\" successfully launch !");

    ros::Rate loop_rate(100);        // 以20Hz频率向下发送更新的数据
    int loop_rate_count = 0;
    while(ros::ok()){
        int dist;
        double avoid_gain = 0.0;
        Ground target;
        Ground velo_goal, velo_avoid, velo_sum;

        if (AGENT_NUM == 0) {
            target = goal_pos; 
        } else {
            target.x = gt_pos[0].x + Forma[forma_type][AGENT_NUM].x;
            target.y = gt_pos[0].y + Forma[forma_type][AGENT_NUM].y;
        }
        dist = (target.x - gt_pos[AGENT_NUM].x) * (target.x - gt_pos[AGENT_NUM].x) + 
            (target.y - gt_pos[AGENT_NUM].y) * (target.y - gt_pos[AGENT_NUM].y);
        if (dist > 30) {
            velo_goal.x = (target.x - gt_pos[AGENT_NUM].x) / 6;
            velo_goal.y = (target.y - gt_pos[AGENT_NUM].y) / 6;
            if (dist < 150) {
                float slow_gain = ((float)dist / 150.0);
                velo_goal.x = (int)((float)velo_goal.x * slow_gain);
                velo_goal.y = (int)((float)velo_goal.y * slow_gain);
            }
            numLimit(velo_goal.x, velo_goal.y, 25);
        }

        if (advoidMode) {
            dist = (int)sqrt(obs_pos.x * obs_pos.x + obs_pos.y * obs_pos.y);
            if (dist < 250 && obs_pos.y > 0) {
                velo_avoid.x = obs_pos.x > 0 ? -25 : 25;
                velo_avoid.y = obs_pos.y > 0 ? -25 : 25;
                avoid_gain = 2;
            } else if (dist < 650 && obs_pos.y > 0) {
                int magnitude = 25 * ((float)(650 - dist) / (650.0 - 250));
                double theta = atan((double)abs(obs_pos.y) / (obs_pos.x));
                avoid_gain = 1.8 * (0.05 + (float)(650 - dist) / (650.0 - 250));
                velo_avoid.x = -1 * mysign(obs_pos.x) * magnitude * cos(theta);
                velo_avoid.y = -1 * mysign(obs_pos.y) * magnitude * sin(theta);
                if (obs_pos.x >= 0 && obs_pos.x < 100) {
                    velo_avoid.x -= ((100 - obs_pos.x) / 5);
                } else if (obs_pos.x > -100 && obs_pos.x < 0) {
                    velo_avoid.x += ((100 + obs_pos.x) / 5);
                }
                numLimit(velo_avoid.x, velo_avoid.y, 25);
            }
        }

        velo_sum.x = static_cast<int>(velo_goal.x + velo_avoid.x * avoid_gain);
        velo_sum.y = static_cast<int>(velo_goal.y + velo_avoid.y * avoid_gain);
        int velo_sqrt = static_cast<int>(sqrt(velo_sum.x*velo_sum.x+velo_sum.y*velo_sum.y));
        if (dist < 350 && velo_sqrt < 10) {
            velo_sum.x = static_cast<int>(velo_goal.x + velo_avoid.x * avoid_gain * 2.3);
            velo_sum.y = static_cast<int>(velo_goal.y + velo_avoid.y * avoid_gain * 1.8);
        }
        numLimit(velo_sum.x, velo_sum.y, 25);

        sendBuffer[1] = 100 + velo_sum.x;
        sendBuffer[2] = 100 + velo_sum.y; 
        sendBuffer[3] = gt_pos[AGENT_NUM].yaw;
        if (sendBuffer[3] > 180) {
            sendBuffer[3] = 180;
        } else if (sendBuffer[3] < 0) {
            sendBuffer[3] = 0;
        }

        if (print_debug) {
            if (loop_rate_count >= 100) {
                ROS_INFO("Vx = %d, Vy = %d, Yaw = %d",sendBuffer[1]-100,sendBuffer[2]-100,sendBuffer[3]);
                loop_rate_count = 0;
            }
            loop_rate_count++;
        }
        

        sp.write(sendBuffer, 6);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    //关闭串口
    sp.close();
 
    return 0;
}