/*************************************************************************
	> File Name: pr2_task_recorder_factory.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 09 Jan 2017 01:35:35 PM PST
 ************************************************************************/

#ifndef _PR2_TASK_RECORDER_FACTORY_H
#define _PR2_TASK_RECORDER_FACTORY_H

// system includes 
#include <ros/ros.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <task_recorder2/task_recorder_base.h>

// local includes 
namespace pr2_task_recorder
{
    class PR2TaskRecorderFactory
    {
        public:
            
            static bool createTaskRecorderByName(const std::string& class_name,
                                                 ros::NodeHandle node_handle
                                                 boost::shared_ptr<task_recorder2::TaskRecorderBase> &task_recorder)
    };
}
#endif
