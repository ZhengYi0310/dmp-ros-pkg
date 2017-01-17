/*************************************************************************
	> File Name: pr2_task_recorder_manager_node.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 09 Jan 2017 02:20:33 PM PST
 ************************************************************************/

// system includes 
#include <ros/ros.h>

// local includes 
#include "pr2_task_recorder2/pr2_task_recorder_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TaskRecorderManager");
    ros::NodeHandle node_handle("~");

    pr2_task_recorder2::PR2TaskRecorderManager pr2_task_recorder_manager(node_handle);

    if (!pr2_task_recorder_manager.initialize())
    {
        ROS_ERROR("Could not initialize pr2 task recorder manager.");
        return -1;
    }

    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}

