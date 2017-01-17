/*************************************************************************
	> File Name: learning_from_demonstration_node.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 04 Jan 2017 02:12:46 PM PST
 ************************************************************************/

// system includes 
#include <ros/ros.h>
#include <robot_info/robot_info_init.h>

// local includes 
#include "dmp_behaviors/learning_from_demonstration.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "");
    robot_info::init();
    ros::NodeHandle node_handle("~");
    std::string action_name = "LearningFromDemonstration";
    dmp_behaviors::LearningFromDemonstration learning_from_demonstration(node_handle, action_name);
    learning_from_demonstration.start();
    ros::spin();
    return 0;
}


