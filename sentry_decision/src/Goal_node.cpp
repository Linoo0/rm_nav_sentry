#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <serial/serial.h>
#include <iostream>
#include <sstream>
// #include <iomanip>
#include <string>
using namespace std;

// 创建一个serial对象
serial::Serial sp;
geometry_msgs::PoseStamped Goal;
uint8_t mode_num;

void sentry_control(uint8_t mode_num)
{
    float p_x,p_y;
    if(mode_num==1)
    {
        p_x=1.0;
        p_y=1.0;
    }
    else if(mode_num==2)
    {
        p_x=1.0;
        p_y=-1.0;
    }

    if(mode_num>0)
    {
        Goal.header.frame_id = "robot_foot_init";
        Goal.pose.position.x = p_x;
        Goal.pose.position.y = p_y;

        Goal.pose.position.z = 0;
        Goal.pose.orientation.x = 0;
        Goal.pose.orientation.y = 0;
        Goal.pose.orientation.z = 0.1;
        Goal.pose.orientation.w = 0.95;
    }
    
}

int main(int argc, char *argv[])
{
    //节点初始化
    ros::init(argc,argv,"goal_node");
    //创建句柄
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",50);

    // 创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    // 设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    // 设置串口通信的波特率
    sp.setBaudrate(115200);
    // 串口设置timeout
    sp.setTimeout(to);

    try
    {
        // 打开串口
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    // 判断串口是否打开成功
    if (sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

    //串口数据读取
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);
            for (int i = 0; i < n; i++)
            {
                if(buffer[i] == 0xAA && buffer[i+2] == 0xBB)
                {
                    mode_num=buffer[i+1];
                    ROS_INFO("%d",mode_num);
                    
                }
            }
        }

        sentry_control(mode_num);
        goal_pub.publish(Goal);
        loop_rate.sleep();
    }

    sp.close();
    return 0;
}
