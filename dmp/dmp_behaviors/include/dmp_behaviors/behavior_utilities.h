/*************************************************************************
	> File Name: behavior_utilities.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 04 Jan 2017 12:15:27 PM PST
 ************************************************************************/

#ifndef _BEHAVIOR_UTILITIES_H
#define _BEHAVIOR_UTILITIES_H

// system includes 
#include <string>
#include <ros/ros.h>

// local includes 
namespace dmp_behaviors
{
    template<class Behavior, class ActionServer>
    class BehaviorUtilities
    {
        public:
            
            static void failed(const std::string& reason, ActionServer& action_server);

        private:

            BehaviorUtilities() {};
            virtual ~BehaviorUtilities() {};
    };

    template<class Behavior, class ActionServer>
    void BehaviorUtilities<Behavior, ActionServer>::failed(const std::string& reason, ActionServer& action_server)
    {
        ROS_ERROR("%s", reason.c_str());
        typename Behavior::ActionResult result;
        result.result = Behavior::ActionResult::FAILED;
        action_server.setSucceeded(result);
    }
}
#endif
