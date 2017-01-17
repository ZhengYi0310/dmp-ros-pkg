/*************************************************************************
	> File Name: dmp_joint_position_controller.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 06 Jan 2017 02:48:04 PM PST
 ************************************************************************/

#ifndef _DMP_JOINT_POSITION_CONTROLLER_H
#define _DMP_JOINT_POSITION_CONTROLLER_H

// system includes
#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>
#include <dynamic_movement_primitive/dynamic_movement_primitive.h>

#include <ros/ros.h>
#include <control_toolbox/pid.h>

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>

// local includes 
#include "pr2_dynamic_movement_primitive_controller/joint_position_controller.h"
#include "pr2_dynamic_movement_primitive_controller/dmp_controller.h"

namespace pr2_dynamic_movement_primitive_controller
{
    class DMPJointPositionController : public pr2_controller_interface::Controller 
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            /*! constructor
             *
             */
            DMPJointPositionController() {};

            /*! descructor
             *
             */
            virtual ~DMPJointPositionController() {};

            /*!
             * @param robot_state
             * @param node
             * @return
             */
            bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &node_handle);

            /*!
             * @return
             */
            void starting();

            /*!
             *
             */
            void update();

            /*!
             * REAL-TIME REQUIREMENTS
             */
            void setDesiredState();

            /*!
             * REAL-TIME REQUIREMENTS
             */
            void holdPositions();

            /*!
             */
            void getDesiredPosition();

        private:
            
            pr2_mechanism_model::JointState *joint_state_;
            int num_joints_;

            Eigen::VectorXd desired_positions_;
            Eigen::VectorXd desired_velocities_;
            Eigen::VectorXd desired_accelerations_;

            std::vector<JointPositionController> joint_position_controllers_;

            // DMPController
            boost::shared_ptr<DMPController> dmp_controller_;
    };
}
#endif
