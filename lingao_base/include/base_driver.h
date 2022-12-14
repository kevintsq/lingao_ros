#ifndef BASE_DRIVER_H
#define BASE_DRIVER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <lingao_msgs/LingAoBmsStatus.h>

#include <cstdio>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "data_format.h"

using namespace std;
class Data_Stream;
class Serial_Async;
class TCP_Async;
class UDP_Async;

class Base_Driver
{
public:
    Base_Driver();
    void base_Loop();
    ~Base_Driver();

private:
    boost::shared_ptr<TCP_Async> tcp;
    boost::shared_ptr<UDP_Async> udp;
    boost::shared_ptr<Serial_Async> serial;
    Data_Stream *stream;

    ros::NodeHandle nh_;
    ros::Subscriber sub_cmd_vel_;
    
    //serial port
    std::string serial_port_;
    int serial_baud_rate;
    bool active;

    void InitParams(); 
    void setCovariance(bool isMove);

private:    //ODOM
    ros::Publisher pub_odom_;
    ros::Time last_odom_vel_time_;
    ros::Time current_time;
    std::string publish_odom_name_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    nav_msgs::Odometry odom_msg;
    Data_Format_Liner liner_rx_;

    tf2_ros::TransformBroadcaster odom_broadcaster_;
    geometry_msgs::TransformStamped odom_tf;

    double x_pos_;
    double y_pos_;
    double th_;
    int loop_rate_;
    bool publish_odom_transform_;

    void init_odom();
    void calc_odom();
    void publish_odom();

private:    //IMU
    ros::Publisher pub_imu_;
    sensor_msgs::Imu imu_msg;

    std::string topic_imu_;
    std::string imu_frame_id_;
    Data_Format_IMU imu_data;
    bool imu_use_;
    bool imu_calibrate_gyro_;
    int imu_cailb_samples_;

    void init_imu();
    void publish_imu();

private:    //Update speed to board
    std::string topic_cmd_vel_name_;
    Data_Format_Liner liner_tx_;
    double cmd_vel_sub_timeout_vel_;
    ros::Timer cmd_vel_cb_timer;

    void cmd_vel_CallBack(const geometry_msgs::Twist& msg);
    void update_liner_speed();
    void subTimeroutCallback(const ros::TimerEvent& event);

private:    //CAILB
    double linear_scale_;
    double angular_scale_;

private:
    ros::Publisher pub_bat_;
    lingao_msgs::LingAoBmsStatus bat_msg;
    Data_Format_BAT rxData_battery;

    void init_sensor_msg();

private:
#define MAX_BUFFER_SIZE 1024
#define ADDR "127.0.0.1"
#define PORT 8080
#define PI 3.14159265358979323846f
    unsigned long long time_stamp = 0;
    char *buffer;
    int socket_fd = -1;
};

#endif // BASE_DRIVER_H
