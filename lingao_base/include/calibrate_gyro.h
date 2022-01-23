/*
 * Copyright (C) 2021, LingAo Robotics, INC.
 * @Version V1.0
 * @Author owen
 * @Date 2022-01-23 14:34:08
 * @LastEditTime: 2022-01-23 15:14:38
 * @LastEditors: owen
 * @Description 
 * @FilePath: /lingao_ws/src/lingaoRobot/lingao_ros/lingao_base/include/calibrate_gyro.h
 */
#ifndef LINGAO_CALIBRATE_GYRO_H
#define LINGAO_CALIBRATE_GYRO_H

#include <ros/ros.h>

class calibrate_gyro
{
private:
    bool first_calib_;
    uint16_t samples_;
    uint16_t sample_count_;

    double gyro_bias_x_;
    double gyro_bias_y_;
    double gyro_bias_z_;

public:
    double calib_x;
    double calib_y;
    double calib_z;

    calibrate_gyro(uint16_t samples = 300)
    {
        first_calib_ = true;
        gyro_bias_x_ = 0.0;
        gyro_bias_y_ = 0.0;
        gyro_bias_z_ = 0.0;
        sample_count_ = 0;
        samples_ = samples;
    }

    bool calib(double raw_angx, double raw_angy, double raw_angz)
    {
        if (first_calib_)
        {
            ROS_INFO_ONCE("Calibrating IMU Gyros; Do not move the IMU");

            // recursively compute mean gyro measurements
            sample_count_++;
            gyro_bias_x_ = ((sample_count_ - 1) * gyro_bias_x_ + raw_angx) / sample_count_;
            gyro_bias_y_ = ((sample_count_ - 1) * gyro_bias_y_ + raw_angy) / sample_count_;
            gyro_bias_z_ = ((sample_count_ - 1) * gyro_bias_z_ + raw_angz) / sample_count_;

            if (sample_count_ >= samples_)
            {
                ROS_INFO("Gyro calibration complete! (bias = [%.3f, %.3f, %.3f])", gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
                first_calib_ = false;
            }
            
            calib_x = 0;
            calib_y = 0;
            calib_z = 0;

            return false;
        }

        calib_x = raw_angx - gyro_bias_x_;
        calib_y = raw_angy - gyro_bias_y_;
        calib_z = raw_angz - gyro_bias_z_;

        return true;
    }

};

#endif