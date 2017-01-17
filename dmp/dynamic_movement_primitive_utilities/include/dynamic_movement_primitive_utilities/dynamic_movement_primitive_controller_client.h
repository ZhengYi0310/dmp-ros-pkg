/*************************************************************************
	> File Name: dynamic_movement_primitive_controller_client.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 07 Dec 2016 09:24:11 AM PST
 ************************************************************************/

#ifndef _DYNAMIC_MOVEMENT_PRIMITIVE_CONTROLLER_CLIENT_H
#define _DYNAMIC_MOVEMENT_PRIMITIVE_CONTROLLER_CLIENT_H

// system includes 
#include <ros/ros.h>
#include <usc_utilities/assert.h>

#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/ControllerStatusMsg.h>

#include <visualization_utilities/robot_pose_visualizer.h>

// local includes 
#include "dynamic_movement_primitive_utilities/dynamic_movement_primitive_controller_base_client.h"
#include "dynamic_movement_primitive_utilities/dynamic_movement_primitive_controller_client.h"

namespace dmp_utilities 
{
    class DynamicMovementPrimitiveControllerClient
    {
        public:

            /*! Constructor 
             */
            DynamicMovementPrimitiveControllerClient() : initialized_(false), current_controller_("NoController") {};

            /*! Destructor 
             */
            virtual ~DynamicMovementPrimitiveControllerClient() {};

            /*!
             * @param controller_name 
             * @return True on success, otherwise False 
             */
            bool initialize(const std::string& controller_name);

            /*!
             * @param robot_part_name 
             * @param base_frame_id 
             * @param controller_names
             * @param controller_namespace 
             * @return True on success, otherwise False 
             */
            bool initialize(const std::string& robot_part_name,
                            const std::vector<std::string>& controller_names,
                            const std::string& controller_namespace);

            /**
             * Sets this client to single-threaded mode. This means that if the waitForCompletion()
             * function is called, it will spin when in single-threaded mode to listen for status messages,
             * otherwise callbacks are expected to be processed separately.
             */
            void setSingleThreadedMode(bool single_threaded_mode = true);

            /*!
             * @param msg 
             * @param wait_for_success 
             * @return True on success, otherwise False 
             */
            bool sendCommand(const dmp::NC2010DMPMsg& msg,
                             const std::string& controller_name,
                             bool wait_for_success = true);

            /*!
             * @param dmp 
             * @param wait_for_success
             * @return True if command was published successfully, False if not 
             */
            bool sendCommand(const dmp_lib::DMPBasePtr& dmp,
                             const std::string& controller_name,
                             bool wait_for_success = true);

            /*!
             * @return True if controller is active, False is not 
             */
            bool isActive();

            /**
             * Gets the status of the last executed DMP
             *
             * @return False if no DMP has ever been executed through this client
             */
            bool getControllerStatus(dynamic_movement_primitive::ControllerStatusMsg& dmp_status);
            

            /*!
             * Wait for completion of the last sent command.
             * @return Ture on sucess, False if the command was preempted or timed out 
             */
            bool waitForCompletion();

            /*!
             * Halts the currently executing trajectory immediately 
             * @return 
             */
            bool halt(bool wait_for_success = true);

        private:

            /*!
             */
            bool initialized_;

            /*!
             */
            typedef DynamicMovementPrimitiveControllerBaseClient<dmp::NC2010DMP, dmp::NC2010DMPMsg> DMPControllerClient;
            typedef boost::shared_ptr<DMPControllerClient> DMPControllerClientPtr;
            typedef std::pair<std::string, DMPControllerClientPtr> ControllerMapPair;
            typedef std::map<std::string, DMPControllerClientPtr>::iterator ControllerMapIterator;

            /*!
             */
            std::map<std::string, DMPControllerClientPtr> nc2010_controller_clients_;

            /*!
             */
            std::vector<std::string> controller_names_;
            std::string current_controller_;

            /*!
             * @param controller 
             * @return 
             */
            bool switchController(const std::string& controller_name);

            enum Configuration
            {
                START,
                GOAL
            };

            /*!
             * @param configuration 
             * @return 
             */
            bool publishConfiguration(const Configuration configuration);

            /*!
             */
            dynamic_movement_primitive::ControllerStatusMsg last_dmp_status_;

            /*!
             */
            void setRobotVisualizer();      
    };
}
#endif
