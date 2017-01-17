/*************************************************************************
	> File Name: skill_library_node.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 26 Dec 2016 03:25:21 PM PST
 ************************************************************************/

// system includes 
#include <ros/ros.h>

// local includes 
#include "skill_library/skill_library.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "/SkillLibrary");

    skill_library::SkillLibrary skill_library;
    if (!skill_library.initialize())
    {
        ROS_ERROR("Could not initialize skill library.");
        return -1;
    }

    return skill_library.run();
}

