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
    Ground (int _x, int _y, int _yaw = 90) {
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
        {{0, 700, 0}, {0, 0, 0}, {0, -700, 0}},
        {{0, 350, 0}, {-700, -350, 0}, {700, -350, 0}} 
    };

int AGENT_NUM = 0;
uint8_t sendBuffer[6];
Ground gt_pos[3], obs_pos(1000, 1000), goal_pos(0, 0);
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
    n.getParam("agent_num", AGENT_NUM);
    AGENT_NUM -= 1;

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

    ros::Subscriber subPath_1 = n.subscribe("/vins_1/vins_estimator/odometry", 1000, path1Callback);
    ros::Subscriber subPath_2 = n.subscribe("/vins_2/vins_estimator/odometry", 1000, path2Callback);
    ros::Subscriber subPath_3 = n.subscribe("/vins_3/vins_estimator/odometry", 1000, path3Callback);
    ros::Subscriber subObst = n.subscribe("obs_coor", 1000, obstCallback);
    ros::Subscriber subGoal = n.subscribe("/target_pos", 100, goalCallback);
    ros::Subscriber subOrder = n.subscribe("/key_order", 100, orderCallback);
    ROS_WARN("Node \"controller\" successfully launch (AGENT_NUM: %d)!", AGENT_NUM);

    ros::Rate loop_rate(20);        // 以20Hz频率向下发送更新的数据
    int loop_rate_count = 0;
    while(ros::ok()){
        int dist;
        Ground unit_center(0, 0);
        Ground velo_goal(0, 0), velo_forma(0, 0), velo_avoid(0, 0), velo_sum(0, 0);
        
        // goal vector
        dist = (int)sqrt((goal_pos.x + Forma[forma_type][AGENT_NUM].x - gt_pos[AGENT_NUM].x) * (goal_pos.x + Forma[forma_type][AGENT_NUM].x - gt_pos[AGENT_NUM].x) + 
            (goal_pos.y + Forma[forma_type][AGENT_NUM].y - gt_pos[AGENT_NUM].y) * (goal_pos.y + Forma[forma_type][AGENT_NUM].y - gt_pos[AGENT_NUM].y));
        if (dist > 30) {
            velo_goal.x = (goal_pos.x + Forma[forma_type][AGENT_NUM].x - gt_pos[AGENT_NUM].x) / 8;
            velo_goal.y = (goal_pos.y + Forma[forma_type][AGENT_NUM].y - gt_pos[AGENT_NUM].y) / 8;
            if (dist < 100) {
                float slow_gain = ((float)dist / 100.0) * ((float)dist / 100.0);
                velo_goal.x = (int)((float)velo_goal.x * slow_gain);
                velo_goal.y = (int)((float)velo_goal.y * slow_gain);
            }
            numLimit(velo_goal.x, velo_goal.y, 15);
        }

        // formation vector
        unit_center.x = (gt_pos[0].x + gt_pos[1].x + gt_pos[2].x) / 3;
        unit_center.y = (gt_pos[0].y + gt_pos[1].y + gt_pos[2].y) / 3;
        dist = (int)sqrt((unit_center.x - goal_pos.x) * (unit_center.x - goal_pos.x) + 
            (unit_center.y - goal_pos.y) * (unit_center.y - goal_pos.y));
        if (dist > 30) {
            int delta_x = (goal_pos.x - unit_center.x);
            int delta_y = (goal_pos.y - unit_center.y);
            numLimit(delta_x, delta_y, 150);
            unit_center.x += delta_x;
            unit_center.y += delta_y;
        }
        dist = (int)sqrt((unit_center.x + Forma[forma_type][AGENT_NUM].x - gt_pos[AGENT_NUM].x) * (unit_center.x + Forma[forma_type][AGENT_NUM].x - gt_pos[AGENT_NUM].x) + 
            (unit_center.y + Forma[forma_type][AGENT_NUM].y - gt_pos[AGENT_NUM].y) * (unit_center.y + Forma[forma_type][AGENT_NUM].y - gt_pos[AGENT_NUM].y));
        if (dist > 10) {
            velo_forma.x = (unit_center.x + Forma[forma_type][AGENT_NUM].x - gt_pos[AGENT_NUM].x) / 8;
            velo_forma.y = (unit_center.y + Forma[forma_type][AGENT_NUM].y - gt_pos[AGENT_NUM].y) / 8;
            if (dist < 50) {
                float slow_gain = ((float)dist / 50.0) * ((float)dist / 50.0);
                velo_forma.x = (int)((float)velo_forma.x * slow_gain);
                velo_forma.y = (int)((float)velo_forma.y * slow_gain);
            }
            numLimit(velo_forma.x, velo_forma.y, 15);
        }

        // avoid vector
        if (advoidMode) {
            dist = (int)sqrt(obs_pos.x * obs_pos.x + obs_pos.y * obs_pos.y);
            if (dist < 250 && obs_pos.y > 0) {
                velo_avoid.x = obs_pos.x > 0 ? -15 : 15;
                velo_avoid.y = obs_pos.y > 0 ? -15 : 15;
            } else if (dist < 650 && obs_pos.y > 0) {
                int magnitude = 15 * ((float)(650 - dist) / (650.0 - 250));
                double theta = atan((double)abs(obs_pos.y) / (obs_pos.x));
                velo_avoid.x = -1 * mysign(obs_pos.x) * magnitude * cos(theta);
                velo_avoid.y = -1 * mysign(obs_pos.y) * magnitude * sin(theta);
                if (obs_pos.x >= 0 && obs_pos.x < 100) {
                    velo_avoid.x -= ((100 - obs_pos.x) / 8);
                } else if (obs_pos.x > -150 && obs_pos.x < 0) {
                    velo_avoid.x += ((100 + obs_pos.x) / 8);
                }
                numLimit(velo_avoid.x, velo_avoid.y, 15);
            }
        }

        velo_sum.x = static_cast<int>(velo_goal.x + velo_forma.x + velo_avoid.x * 3);
        velo_sum.y = static_cast<int>(velo_goal.y + velo_forma.y + velo_avoid.y * 3);
        numLimit(velo_sum.x, velo_sum.y, 15);

        sendBuffer[1] = 100 + velo_sum.x;
        sendBuffer[2] = 100 + velo_sum.y; 
        sendBuffer[3] = gt_pos[AGENT_NUM].yaw;
        if (sendBuffer[3] > 180) {
            sendBuffer[3] = 180;
        } else if (sendBuffer[3] < 0) {
            sendBuffer[3] = 0;
        }

        if (print_debug) {
            if (loop_rate_count >= 20) {
                // ROS_INFO("Vx = %d, Vy = %d, Yaw = %d",sendBuffer[1]-100,sendBuffer[2]-100,sendBuffer[3]);
                ROS_INFO("Goal x = %d, y = %d", goal_pos.x, goal_pos.y);
                ROS_INFO("Goal velo x = %d, y = %d", velo_goal.x, velo_goal.y);
                ROS_INFO("Forma velo x = %d, y = %d", velo_forma.x, velo_forma.y);
                ROS_INFO("total velo x = %d, y = %d", velo_sum.x, velo_sum.y);
                ROS_INFO(" ");
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