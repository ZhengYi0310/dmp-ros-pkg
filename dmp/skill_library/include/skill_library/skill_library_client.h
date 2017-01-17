/*************************************************************************
	> File Name: skill_library_client.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 23 Dec 2016 07:45:10 PM PST
 ************************************************************************/

#ifndef _SKILL_LIBRARY_CLIENT_H
#define _SKILL_LIBRARY_CLIENT_H

// system includes
#include <ros/ros.h>

#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_controller_client.h>

// local includes 

namespace skill_library
{
    class SkillLibraryClient
    {
        public:
            
            /*! Constructor
             * @param node_handle
             */
            SkillLibraryClient(ros::NodeHandle node_handle) {};

            /*! Destructor
             */
            virtual ~SkillLibraryClient() {};

            /*!
             * @param dmp_name
             * @param dmp 
             * @return True on success, otherwise False 
             */
            bool get(const std::string& dmp_name, dmp_lib::DMPBasePtr& dmp);

        private:
            ros::NodeHandle node_handle_;

            ros::ServiceClient get_affordance_service_client_;
            dmp_utilities::DynamicMovementPrimitiveControllerClient right_arm_dmp_controller_client_;
    };
}
#endif
