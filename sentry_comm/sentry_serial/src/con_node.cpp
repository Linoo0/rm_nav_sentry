#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include "tf/tf.h"
#include "math.h"
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <std_msgs/String.h>  
using namespace std;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
// 创建一个serial对象
serial::Serial sp;

// 联合体
typedef union
{
    float data;
    uint8_t data8[4];
} data_u;

class Point
{
    public:
    float x;
    float y;
    float yaw;
    Point(float x1,float y1,float yaw1):x(x1),y(y1),yaw(yaw1){}
};


bool Navigation_Mode_now = true;

float zx = 0;
float zy = 0;
float goalX = 0;
float goalY = 0;

//串口发送的数据处理
void SpSend(uint8_t &_cnt,data_u _temp,uint8_t data_to_send[100],Point a)
{
    _temp.data = a.x;
    data_to_send[_cnt++] = _temp.data8[0];
    data_to_send[_cnt++] = _temp.data8[1];
    data_to_send[_cnt++] = _temp.data8[2];
    data_to_send[_cnt++] = _temp.data8[3]; // 最高位

    _temp.data = a.y;
    data_to_send[_cnt++] = _temp.data8[0];
    data_to_send[_cnt++] = _temp.data8[1];
    data_to_send[_cnt++] = _temp.data8[2];
    data_to_send[_cnt++] = _temp.data8[3]; // 最高位

    _temp.data = a.yaw;
    data_to_send[_cnt++] = _temp.data8[0];
    data_to_send[_cnt++] = _temp.data8[1];
    data_to_send[_cnt++] = _temp.data8[2];
    data_to_send[_cnt++] = _temp.data8[3]; // 最高位
}

void OdometryAndCmd_VelHandler(const geometry_msgs::Twist& cmd_vel)
{
    ROS_INFO("x1 = %f,y1 = %f,yaw1 = %f",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);
    float x = cmd_vel.linear.x;
    float y = cmd_vel.linear.y;
    float yaw = cmd_vel.angular.z;
    uint8_t _cnt = 0;
    data_u _temp;              // 声明一个联合体实例，使用它将待发送数据转换为字节数组
    uint8_t data_to_send[100]; // 待发送的字节数组
    data_to_send[_cnt++] = 0xEE;

    SpSend(_cnt,_temp,data_to_send,Point(x,y,yaw));
    data_to_send[_cnt++] = 0xFF;
    // 把数据发送回去
    sp.write(data_to_send, _cnt);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //节点初始化
    ros::init(argc,argv,"con_node");
    //创建句柄
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
    nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
    std::string cmd_vel_topic;
    nh.getParam("cmd_vel_topic", cmd_vel_topic);
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

    nh.getParam("Navigation_Mode_now", Navigation_Mode_now);
    //订阅话题
    ros::Subscriber sub = nh.subscribe(cmd_vel_topic, 1000, OdometryAndCmd_VelHandler); 

    ros::spin();
    sp.close();
    return 0;
}
