/*************************************************************************
	> File Name: pr2_task_recorder_manager.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 09 Jan 2017 01:43:49 PM PST
 ************************************************************************/

#ifndef _PR2_TASK_RECORDER_MANAGER_H
#define _PR2_TASK_RECORDER_MANAGER_H

// system includes 
#include <vector>
#include <ros/ros.h>

#include <task_recorder2/task_recorder_manager.h>
#include <task_recorder2/task_recorder_base.h>

// local includes 

namespace pr2_task_recorder
{
    class PR2TaskRecorderManager : public task_recorder2::TaskRecorderManager
    {
        public :

            /*! Constructor 
             */
            PR2TaskRecorderManager(ros::NodeHandle node_handle);

            /*! Destructor 
             */
            virtual ~PR2TaskRecorderManager() {};

            /*!
             * @param task_recorders
             * @return
             */
            bool read(std::vector<boost::shared_ptr<task_recorder2::TaskRecorderBase> > &task_recorders);

        private:
    };
}
#endif
