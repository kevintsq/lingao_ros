#ifndef BASE_DRIVER_H
#define BASE_DRIVER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <lingao_msgs/Imu.h>
#include <lingao_msgs/Battery.h>

#include <stdio.h>
#include <string>
#include "data_format.h"

using namespace std;
class Data_Stream;
class Serial_Async;

class Base_Driver
{
public:
    Base_Driver();
    void base_Loop();

private:

    boost::shared_ptr<Serial_Async> serial;
    Data_Stream *stream;

    ros::NodeHandle nh_;
    ros::Subscriber sub_cmd_vel_;
    
    //serial port
    std::string serial_port_;
    int serial_baud_rate;

    void InitParams(); 
    void setCovariance(bool isMove);

private:    //ODOM
    ros::Publisher pub_odom_;
    ros::Time last_odom_vel_time_;
    ros::Time current_time;
    std::string topic_odom_;
    nav_msgs::Odometry odom_msg;
    Data_Format_Liner liner_rx_;

    tf2_ros::TransformBroadcaster odom_broadcaster_;
    geometry_msgs::TransformStamped odom_trans;

    double x_pos_;
    double y_pos_;
    double th_;
    int loop_rate_;
    bool publish_odom_;

    void init_odom();
    void calc_odom();
    void publish_odom();

private:    //IMU
    ros::Publisher pub_imu_;
    lingao_msgs::Imu imu_msg;

    std::string topic_imu_;
    std::string imu_frame_id_;
    Data_Format_IMU imu_data;
    bool imu_used;

    void init_imu();
    void publish_imu();

private:    //Update speed to board
    std::string topic_cmd_vel_;
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
    lingao_msgs::Battery bat_msg;
    Data_Format_BAT rxData_battery;

    void init_sensor_msg();
};

#endif // BASE_DRIVER_H
