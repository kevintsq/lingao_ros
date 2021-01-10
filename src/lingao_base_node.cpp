// #include "lingao_base.h"
#include "base_driver.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "lingao_base_node");

    Base_Driver base_driver;
    base_driver.base_Loop();

    // LINGAOBASE lingao;
    // while (ros::ok())
    // {
    // }

    ros::spin();
    return 0;
}
