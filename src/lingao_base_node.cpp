/*
 * Copyright (C) 2021, LingAo Robotics, INC.
 * @Version V1.0
 * @Author owen
 * @Date 2021-08-06 17:08:41
 * @LastEditTime 2021-11-05 13:16:03
 * @LastEditors owen
 * @Description 
 * @FilePath /lingao_ws/src/lingaoRobot/lingao_bringup/src/lingao_base_node.cpp
 */

#include "base_driver.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "lingao_base_node");

    Base_Driver base_driver;
    base_driver.base_Loop();

    ros::spin();
    return 0;
}
