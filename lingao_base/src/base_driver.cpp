/*
 *  Copyright (C) 2020, LingAo Robotics, INC.
 *  author: owen <keaa@keaa.net>
 *  maintainer: owen <keaa@keaa.net>
 *
 *  base driver server
 */
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "base_driver.h"
#include "data_stream.h"
#include "Serial_Async.h"
#include "TCP_Async.h"
#include "UDP_Async.h"

using namespace std;

Base_Driver::Base_Driver() : nh_("~")
{
    InitParams();
    active = false;

    serial = boost::make_shared<Serial_Async>();
    stream = new Data_Stream(serial.get());

    if (serial->init(serial_port_, serial_baud_rate))
    {
        ROS_INFO_STREAM("Main board Serial Port open success, com_port_name= " << serial_port_);
    }
    else
    {
        ROS_ERROR_STREAM("Main board Serial Port open failed... com_port_name= " << serial_port_);
        return;
    }

    if (stream->version_detection())
    {
        Data_Format_VER version = stream->get_data_version();
        ROS_INFO_STREAM("The version matches successfully, current version: [" << (int)version.protocol_ver << "]");
        ROS_INFO_STREAM("GET Equipment Identity: " << version.equipmentIdentity);
    }
    else
    {
        Data_Format_VER version = stream->get_data_version();
        ROS_INFO_STREAM("GET Equipment Identity: " << version.equipmentIdentity);
        ROS_ERROR_STREAM("The driver version does not match,  Main control board driver version:[" << (int)version.protocol_ver << "] Current driver version:["
                                                                                                   << LA_PROTOCOL_VERSION << "]");
        return;
    }

    init_odom();
    init_imu();
    init_sensor_msg();

    x_pos_ = 0;
    y_pos_ = 0;
    th_ = 0;

    liner_tx_.set(.0, .0, .0);
    cmd_vel_cb_timer = nh_.createTimer(ros::Duration(0, cmd_vel_sub_timeout_vel_), &Base_Driver::subTimeroutCallback, this, true);

    active = true;
}

void Base_Driver::InitParams()
{
    // Serial Port Params
    nh_.param("port_name", serial_port_, std::string("/dev/lingao"));
    nh_.param("port_baud", serial_baud_rate, 230400);
    nh_.param("freq", loop_rate_, 100);

    // Topic Params
    nh_.param("topic_cmd_vel_name", topic_cmd_vel_name_, std::string("/cmd_vel"));
    nh_.param("publish_odom_name", publish_odom_name_, std::string("raw_odom"));
    nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    nh_.param("base_frame_id", base_frame_id_, std::string("base_footprint"));
    nh_.param("cmd_vel_sub_timeout", cmd_vel_sub_timeout_vel_, 1.0);
    nh_.param("pub_odom_tf", publish_odom_transform_, false);

    // Scale Params
    nh_.param("linear_scale", linear_scale_, 1.0);
    nh_.param("angular_scale", angular_scale_, 1.0);

    // IMU Params
    nh_.param("topic_imu", topic_imu_, std::string("/imu/onboard_imu"));
    nh_.param("imu_frame_id", imu_frame_id_, std::string("imu_link"));
    nh_.param("imu_use", imu_use, true);
}

void Base_Driver::init_imu()
{
    if (imu_use)
    {
        pub_imu_                = nh_.advertise<sensor_msgs::Imu>(topic_imu_, 50);
        imu_msg.header.frame_id = imu_frame_id_;

        // https://github.com/KristofRobot/razor_imu_9dof/blob/indigo-devel/nodes/imu_node.py
        imu_msg.orientation_covariance[0] = 0.0025;
        imu_msg.orientation_covariance[4] = 0.0025;
        imu_msg.orientation_covariance[8] = 0.0025;

        imu_msg.angular_velocity_covariance[0] = 0.000015;
        imu_msg.angular_velocity_covariance[4] = 0.000015;
        imu_msg.angular_velocity_covariance[8] = 0.000015;

        imu_msg.linear_acceleration_covariance[0] = 0.0001;
        imu_msg.linear_acceleration_covariance[4] = 0.0001;
        imu_msg.linear_acceleration_covariance[8] = 0.0001;
    }
}

void Base_Driver::init_sensor_msg() { pub_bat_ = nh_.advertise<lingao_msgs::LingAoBmsStatus>("battery_state", 50); }

void Base_Driver::init_odom()
{
    ROS_INFO_STREAM("advertise to the odom topic on [" << publish_odom_name_ << "]");
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>(publish_odom_name_, 50);

    ROS_INFO_STREAM("subscribe to the cmd topic on [" << topic_cmd_vel_name_ << "]");
    sub_cmd_vel_ = nh_.subscribe(topic_cmd_vel_name_, 50, &Base_Driver::cmd_vel_CallBack, this);

    // 初始化odom_trans
    odom_tf.header.frame_id         = odom_frame_id_;
    odom_tf.child_frame_id          = base_frame_id_;
    odom_tf.transform.translation.z = 0.0;

    //初始化odom 里程计消息
    odom_msg.header.frame_id      = odom_frame_id_;
    odom_msg.child_frame_id       = base_frame_id_;
    odom_msg.pose.pose.position.z = 0.0;

    setCovariance(false);
}

void Base_Driver::setCovariance(bool isMove)
{
    if (isMove == true)
    {
        odom_msg.pose.covariance[0]   = 1e-3;
        odom_msg.pose.covariance[7]   = 1e-3;
        odom_msg.pose.covariance[14]  = 1e6;
        odom_msg.pose.covariance[21]  = 1e6;
        odom_msg.pose.covariance[28]  = 1e6;
        odom_msg.pose.covariance[35]  = 1e-2;
        
        odom_msg.twist.covariance[0]  = 1e-3;
        odom_msg.twist.covariance[7]  = 1e-3;
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = 1e-2;
    }
    else
    {
        odom_msg.pose.covariance[0]   = 1e-9;
        odom_msg.pose.covariance[7]   = 1e-9;
        odom_msg.pose.covariance[14]  = 1e6;
        odom_msg.pose.covariance[21]  = 1e6;
        odom_msg.pose.covariance[28]  = 1e6;
        odom_msg.pose.covariance[35]  = 1e-9;

        odom_msg.twist.covariance[0]  = 1e-9;
        odom_msg.twist.covariance[7]  = 1e-9;
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = 1e-9;
    }
}

void Base_Driver::subTimeroutCallback(const ros::TimerEvent& event) { liner_tx_.set(.0, .0, .0); }

void Base_Driver::cmd_vel_CallBack(const geometry_msgs::Twist& msg)
{
    liner_tx_.set(msg.linear.x, msg.linear.y, msg.angular.z);
    cmd_vel_cb_timer.setPeriod(ros::Duration(cmd_vel_sub_timeout_vel_), true);
}

//  更新速度消息
void Base_Driver::update_liner_speed()
{
    static Data_Format_Liner linertx;
    linertx.EndianSwapSet(&liner_tx_);

    stream->update_liner_speed(linertx);
}

void Base_Driver::base_Loop()
{
    bool isRead = false;
    if (active == false)
        return;

    ros::Rate loop_rate(loop_rate_); // HZ
    while (ros::ok())
    {

        //判断串口是否正常开启
        if (serial->isOpen() == false)
        {
            ROS_ERROR("Serial closes unexpectedly!");
            return;
        }

        //发送读取请求，并且等待数据读取，读取成功返回true
        isRead = stream->get_Message(MSG_ID_GET_VOLTAGE);
        if (isRead)
        {
            //成功读取后数据处理
            rxData_battery      = stream->get_data_battery();
            bat_msg.header.stamp = ros::Time::now();
            bat_msg.voltage     = rxData_battery.voltage / 100.0;
            bat_msg.current     = rxData_battery.current / 100.0;
            bat_msg.percentage  = rxData_battery.percentage;
            bat_msg.temperature = rxData_battery.temperature /10.0;

            pub_bat_.publish(bat_msg);
        }
        else
            ROS_WARN_STREAM("Get VOLTAGE Data Time Out!");

        if (imu_use)
        {
            isRead = stream->get_Message(MSG_ID_GET_IMU);
            if (isRead)
            {
                imu_data = stream->get_data_imu();
                publish_imu();
            }
            else
                ROS_WARN_STREAM("Get IMU Data Time Out!");
        }

        isRead = stream->get_Message(MSG_ID_GET_VELOCITY);
        if (isRead)
        {
            liner_rx_ = stream->get_data_liner();
            if (liner_rx_.v_liner_x == 0 && liner_rx_.v_angular_z == 0)
            {
                setCovariance(false);
            }
            else
                setCovariance(true);

            calc_odom();
            publish_odom();
        }
        else
            ROS_WARN_STREAM("Get VELOCITY Data Time Out!");

        update_liner_speed();

        ros::spinOnce();
        loop_rate.sleep(); // 等待loop_rate設定的時間
    }
}

/// 函数功能：根据机器人线速度和角度计算机器人里程计
void Base_Driver::calc_odom()
{
    ros::Time current_time = ros::Time::now();

    float linear_velocity_x_  = liner_rx_.v_liner_x * linear_scale_;
    float linear_velocity_y_  = liner_rx_.v_liner_y * linear_scale_;
    float angular_velocity_z_ = liner_rx_.v_angular_z * angular_scale_;

    double vel_dt_      = (current_time - last_odom_vel_time_).toSec();
    last_odom_vel_time_ = current_time;

    double delta_x  = (linear_velocity_x_ * cos(th_) - linear_velocity_y_ * sin(th_)) * vel_dt_; // m
    double delta_y  = (linear_velocity_x_ * sin(th_) + linear_velocity_y_ * cos(th_)) * vel_dt_; // m
    double delta_th = angular_velocity_z_ * vel_dt_;                                             // radians

    //计算机器人的当前位置
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    th_ += delta_th; //实时角度信息,如果这里不使用IMU，也可以通过这种方式计算得出
}

/// 函数功能：上传ODOM信息
void Base_Driver::publish_odom()
{
    current_time = ros::Time::now();

    tf2::Quaternion odom_quat;

    //计算机器人在四元数角下的航向，ROS具有计算四元数角偏航的功能
    odom_quat.setRPY(0, 0, th_);

    // 发布TF
    if (publish_odom_transform_)
    {
        // robot's position in x,y, and z
        odom_tf.transform.translation.x = x_pos_;
        odom_tf.transform.translation.y = y_pos_;
        odom_tf.transform.translation.z = 0.0;

        // robot's heading in quaternion
        odom_tf.transform.rotation.x = odom_quat.x();
        odom_tf.transform.rotation.y = odom_quat.y();
        odom_tf.transform.rotation.z = odom_quat.z();
        odom_tf.transform.rotation.w = odom_quat.w();

        odom_tf.header.stamp = current_time;
        //使用odom_trans对象发布机器人的tf
        odom_broadcaster_.sendTransform(odom_tf);
    }

    //发布里程计消息
    odom_msg.header.stamp         = current_time;
    odom_msg.pose.pose.position.x = x_pos_;
    odom_msg.pose.pose.position.y = y_pos_;
    odom_msg.pose.pose.position.z = 0.0;

    //四元数机器人的航向
    odom_msg.pose.pose.orientation.x = odom_quat.x();
    odom_msg.pose.pose.orientation.y = odom_quat.y();
    odom_msg.pose.pose.orientation.z = odom_quat.z();
    odom_msg.pose.pose.orientation.w = odom_quat.w();

    //编码器的线速度
    odom_msg.twist.twist.linear.x = liner_rx_.v_liner_x * linear_scale_;
    odom_msg.twist.twist.linear.y = liner_rx_.v_liner_y * linear_scale_;
    odom_msg.twist.twist.linear.z = 0.0;

    //编码器的角速度
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = liner_rx_.v_angular_z * angular_scale_;

    pub_odom_.publish(odom_msg);
}

void Base_Driver::publish_imu()
{
    imu_msg.header.stamp          = ros::Time::now();
    imu_msg.linear_acceleration.x = imu_data.accx * 9.80665; // 加速度应以 m/s^2（原单位 g ）
    imu_msg.linear_acceleration.y = imu_data.accy * 9.80665;
    imu_msg.linear_acceleration.z = imu_data.accz * 9.80665;

    imu_msg.angular_velocity.x = imu_data.angx;
    imu_msg.angular_velocity.y = imu_data.angy;
    imu_msg.angular_velocity.z = imu_data.angz;

    tf2::Quaternion goal_quat;
    goal_quat.setRPY(imu_data.roll, imu_data.pitch, imu_data.yaw);
    imu_msg.orientation = tf2::toMsg(goal_quat);

    pub_imu_.publish(imu_msg);
}