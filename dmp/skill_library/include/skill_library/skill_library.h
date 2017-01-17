/*************************************************************************
	> File Name: skill_library.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 23 Dec 2016 07:30:32 PM PST
 ************************************************************************/

#ifndef _SKILL_LIBRARY_H
#define _SKILL_LIBRARY_H

// system includes 
#include <string>
#include <ros/ros.h>

#include <task_msgs/Task.h>
#include <task_msgs/Object.h>

#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h>

#include <inverse_kinematics/inverse_kinematics_with_nullspace_optimization.h>

// local includes 
#include "skill_library/dmp_library_client.h"

#include "skill_library/getAffordance.h"
#include "skill_library/addAffordance.h"

namespace skill_library
{
    class SkillLibrary
    {
        public:
            
            /*! Constructor
             */
            SkillLibrary() : initialized_(false), node_handle_("~") {};
            
            /*! Destructor
             */
            virtual ~SkillLibrary() {};

            /*!
             * @return True on success, otherwise False 
             */
            bool initialize();

            /*!
             * @param request
             * @param response
             * @return 
             */
            bool addAffordance(addAffordance::Request& request,
                               addAffordance::Response& response);

            /*!
             * @param request
             * @param response
             * @return 
             */
            bool getAffordance(getAffordance::Request& request,
                               getAffordance::Response& response);

            /*!
             */
            int run()
            {
                ros::spin();
                return 0;
            }

        private:
            
            /*!
             */
            bool initialized_;

            /*!
             */
            ros::NodeHandle node_handle_;
            ros::ServiceServer add_affordance_service_server_;
            ros::ServiceServer get_affordance_service_server_;

            /*!
             */
            DMPLibraryClient dmp_library_client_;
    };
}
#endif
