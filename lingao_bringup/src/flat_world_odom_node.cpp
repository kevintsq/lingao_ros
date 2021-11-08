/*
 * Copyright (C) 2021, LingAo Robotics, INC.
 * @Version V1.0
 * @Author owen
 * @Date 2021-11-06 20:26:51
 * @LastEditTime 2021-11-06 22:13:07
 * @LastEditors owen
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class FlatWorldOdomNode {
  public:
    FlatWorldOdomNode()
    {
        publisher_  = nh_.advertise<nav_msgs::Odometry>("odom_out", 10);
        subscriber_ = nh_.subscribe("odom_in", 150, &FlatWorldOdomNode::msgCallback, this);
        ROS_ASSERT(true);
    }

  private:
    ros::NodeHandle nh_;
    ros::Time last_published_time_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    
    void msgCallback(const nav_msgs::Odometry::ConstPtr odom_in)
    {
        if (last_published_time_.isZero() || odom_in->header.stamp > last_published_time_)
        {
            last_published_time_        = odom_in->header.stamp;
            nav_msgs::Odometry odom_out = *odom_in;
            publisher_.publish(odom_out);
        }
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "flat_world_odom_node");

    FlatWorldOdomNode flat_world_odom_node;

    ros::spin();

    return 0;
}
