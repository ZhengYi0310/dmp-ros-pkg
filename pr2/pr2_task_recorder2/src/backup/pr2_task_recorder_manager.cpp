/*************************************************************************
	> File Name: pr2_task_recorder_manager.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 09 Jan 2017 02:02:08 PM PST
 ************************************************************************/

// system includes 
#include <task_recorder2_utilities/task_recorder_specification_utilities.h>

// local includes 
#include "pr2_task_recorder2/pr2_task_recorder2_manager.h"
#include "pr2_task_recorder2/pr2_task_recorder2_factory.h"
using namespace pr2_task_recorder2
{
    PR2TaskRecorderManager::PR2TaskRecorderManager(ros::NodeHandle node_handle) : task_recorder2::TaskRecorderManager(node_handle)
    {
    }

    bool PR2TaskRecorderManager::read(std::vector<boost::shared_ptr<task_recorder2::TaskRecorderBase> > &task_recorders)
    {
        std::vector<task_recorder2_msgs::TaskRecorderSpecification> specifications;
        ROS_VERIFY(task_recorder2_utilities::readTaskRecorderSpecification(specifications, recorder_io_.node_handle_));
        ROS_ASSERT_MSG(specifications.size() > 0, "No task recorders specified. Cannot initialize TaskRecorderManager.");

        for (int i = 0; i < (int)specifications.size(); ++i)
        {
            task_recorder2::TaskRecorderPtr recorder;
            ROS_VERIFY(PR2TaskRecorder::createTaskRecorderByName(specifications[i].class_name, recorder_io_.node_handle_, recorder))
            ROS_INFO("Initializing pr2 task recorder >%s< with topic >%s< and splining method >%s<.", specifications[i].class_name.c_str(), specifications[i].topic_name.c_str(), specifications[i].splining_method.c_str());
            ROS_VERIFY(recorder->initialize(specifications[i].topic_name, specifications[i].splining_method));
            task_recorders.push_back(recorder);
        }
        return true;
    }
}

