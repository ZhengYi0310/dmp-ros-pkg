/*************************************************************************
	> File Name: learning_from_demonstration_client_node.cpp
	> Author:Yi Zheng 
	> Mail: 
	> Created Time: Tue 24 Jan 2017 02:05:53 PM PST
 ************************************************************************/

// system includes 
#include <ros/ros.h>
#include <robot_info/robot_info_init.h>

// local includes 
#include "dmp_behaviors/learning_from_demonstration_client.h"
#include "dmp_behaviors/learning_from_demonstration.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "");
    ros::NodeHandle node_handle("lfd_client");
    std::string action_name = "LearningFromDemonstration";
    std::string joint_states_bag_name = "joint_states_wam.bag";
    dmp_behaviors::LearningFromDemonstrationClient learning_from_demonstration_client(node_handle, action_name, joint_states_bag_name);
    learning_from_demonstration_client.waitForServer();
    learning_from_demonstration_client.sendGoal();
    ros::spin();

    return 0;
}

