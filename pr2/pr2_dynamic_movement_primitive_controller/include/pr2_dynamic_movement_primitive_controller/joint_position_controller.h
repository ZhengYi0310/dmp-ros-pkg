/*************************************************************************
	> File Name: joint_position_controller.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 06 Jan 2017 11:08:57 AM PST
 ************************************************************************/

#ifndef _JOINT_POSITION_CONTROLLER_H
#define _JOINT_POSITION_CONTROLLER_H

// system includes 
#include <ros/node_handle.h>
#include <rosrt/rosrt.h>

#include <pr2_controller_interface/controller.h>

#include <boost/shared_ptr.hpp>

#include <control_toolbox/pid.h>
#include <control_toolbox/pid_gains_setter.h>

#include <pr2_controllers_msgs/JointControllerState.h>

#include <angles/angles.h>

namespace pr2_dynamic_movement_primitive_controller
{

    class JointPositionController
    {
        public:
            
            /*!
             * @return
             */
            JointPositionController();
            ~JointPositionController();

            bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &node_handle);

            /*!
             * @brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
             * @param command
             */
            void setCommand(const double cmd);

            /*!
             * @brief Get latest position command to the joint: revolute (angle) and prismatic (position).
             */
            void getCommand(double &cmd);

            /*!
             *
             */
            void starting();

            /*!
             * @brief Issues commands to the joint. Should be called at regular intervals
             */
            void update();

            void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
            void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min);
            
            std::string getJointName();

            double getJointPosition() const;
            double getJointVelocity() const;
            
            double getJointPositionError() const;
            double getJointVelocityError() const;

            double getCommandedEffort() const;

            ros::Duration dt_;

        private:

            bool initialized_; 
            pr2_mechanism_model::JointState *joint_state_; /*! Joint we're controlling. */

            double command_;                                      /*! Last commanded position. */
            double commanded_effort_;                             /*! Last commanded effort. */

            int publisher_rate_;
            int publisher_counter_;
            int publisher_buffer_size_;

            double error_;
            double error_dot_;
            double last_error_;

            pr2_mechanism_model::RobotState *robot_;              /**< Pointer to robot structure. */
            control_toolbox::Pid pid_controller_;                 /**< Internal PID controller. */

            ros::NodeHandle node_handle_;
            
            ros::Time last_time_;                                 /**< Last time stamp of update. */

            boost::shared_ptr<rosrt::Publisher<pr2_controllers_msgs::JointControllerState> > controller_state_publisher_;

            void publish();   
    };

    inline double JointPositionController::getJointPosition() const
    {
        if (joint_state_->joint_->type == urdf::Joint::CONTINUOUS)
        {
            return angles::normalize_angle(joint_state_->position_);
        }
        return joint_state_->position_;
    }

    inline double JointPositionController::getJointVelocity() const
    {
        return joint_state_->velocity_;
    }

    inline double JointPositionController::getJointPositionError() const
    {
        return error_;
    }

    inline double JointPositionController::getJointVelocityError() const
    {
        return error_dot_;
    }

    inline double JointPositionController::getCommandedEffort() const
    {
        return commanded_effort_;
    }
}
#endif
