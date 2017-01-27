/*************************************************************************
	> File Name: learning_from_demonstration_client.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 24 Jan 2017 10:41:42 AM PST
 ************************************************************************/
#ifndef _LEARNING_FROM_DEMONSTRATION_CLIENT_H
#define _LEARNING_FROM_DEMONSTRATION_CLIENT_H 
// system includes 
#include <sstream>
#include <ros/ros.h>
#include <ros/package.h>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/file_io.h>
#include <robot_info/robot_info_init.h>

#include <skill_library/getAffordance.h>
#include <skill_library/Affordance.h>

#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h>
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_learner.h>
#include <dynamic_movement_primitive_utilities/trajectory_utilities.h>

#include <Eigen/Eigen>
#include <trajectory_msgs/JointTrajectory.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <dmp_behavior_actions/LearningFromDemonstrationAction.h>

// local includes 
//#include "dmp_behaviors/learning_from_demonstration.h"
//#include "dmp_bahaviors/behavior_utilities.h"

using namespace dmp_behavior_actions;
using namespace skill_library;
using namespace dmp_utilities;
using namespace Eigen;

namespace dmp_behaviors
{
    class LearningFromDemonstrationClient
    {
        public:
            
            typedef actionlib::SimpleActionClient<dmp_behavior_actions::LearningFromDemonstrationAction> ActionClient;
        LearningFromDemonstrationClient(ros::NodeHandle& node_handle, const std::string& action_name, const std::string& joint_states_bag_file_name);
        virtual ~LearningFromDemonstrationClient() {};

        void waitForServer();
        void sendGoal();
        void GoalCallback(const actionlib::SimpleClientGoalState& state,
                          const LearningFromDemonstrationResultConstPtr& result);

        private:
            
            ros::NodeHandle node_handle_;
            
            ActionClient action_client_;
            
            ros::ServiceClient get_affordance_service_client_;

            trajectory_msgs::JointTrajectory roll_out_trajectory_;
            trajectory_msgs::JointTrajectory demonstration_trajectory_;

            std::string joint_states_bag_file_name_;
              
    };

    LearningFromDemonstrationClient::LearningFromDemonstrationClient(ros::NodeHandle& node_handle, const std::string& action_name, const std::string& joint_states_bag_file_name) : node_handle_(node_handle), action_client_(action_name, true), joint_states_bag_file_name_(joint_states_bag_file_name) {}

    void LearningFromDemonstrationClient::waitForServer()
    {
        ROS_INFO("Waiting for action server to start.");
        action_client_.waitForServer();
        ROS_INFO("Action server started, sending goal.");
    }

    void LearningFromDemonstrationClient::sendGoal()
    {
        LearningFromDemonstrationGoal goal;
        
        goal.joint_states_bag_file_name = joint_states_bag_file_name_;
        goal.robot_part_names_from_trajectory.push_back("RIGHT_ARM");
        goal.type.type = 1;

        action_client_.sendGoal(goal, boost::bind(&LearningFromDemonstrationClient::GoalCallback, this, _1, _2), ActionClient::SimpleActiveCallback(), ActionClient::SimpleFeedbackCallback());
    }

    void LearningFromDemonstrationClient::GoalCallback(const actionlib::SimpleClientGoalState& state, const LearningFromDemonstrationResultConstPtr& result)
    {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("Result status: %i", result->result);

        std::string demonstration_data_directory;

        std::string package_path = ros::package::getPath("dmp_behaviors");
        std::string abs_bag_file_name = package_path + "/demonstrations/" + joint_states_bag_file_name_;

        std::vector<std::string> robot_part_names;
        ros::NodeHandle node_handle_tmp("/LearningFromDemonstration");
        //usc_utilities::read(node_handle_tmp, "robot_part_names", robot_part_names);
        robot_part_names.push_back("RIGHT_ARM");

        robot_info::init();
        std::vector<std::string> joint_variable_names;
        robot_info::RobotInfo::getArmJointNames(robot_part_names, joint_variable_names);

        // read the bag file and recreate the trajectory.
        dmp_lib::Trajectory trajectory;
        dmp_utilities::TrajectoryUtilities::createJointStateTrajectory(trajectory, joint_variable_names, abs_bag_file_name, robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);

        std::string demo_joint_file_name = "rollout_joint_demonstration.clmc";
        std::string abs_demo_file_name_joint_states = package_path + "/demonstrations/" + demo_joint_file_name;
        trajectory.writeToCLMCFile(abs_demo_file_name_joint_states);

        // learn a dmp and write it to disc 
        dmp_lib::NC2010DMPPtr nc2010_dmp;
        DynamicMovementPrimitiveLearner<dmp::NC2010DMP>::learnJointSpaceDMP(nc2010_dmp, node_handle_tmp, abs_bag_file_name, robot_part_names);
        std::string dmp_bag_file_name = "joint_space_dmp.bag";
        std::string abs_dmp_bag_file_name = package_path + "/demonstrations/" + dmp_bag_file_name;
        dmp::NC2010DynamicMovementPrimitive::writeToDisc(nc2010_dmp, abs_dmp_bag_file_name);

        // propagate the learned dmp.
        nc2010_dmp->setup();
        double initial_duration = 0;
        nc2010_dmp->getInitialDuration(initial_duration);

        dmp_lib::Trajectory rollout;

        // change the goal 
        VectorXd new_goal = VectorXd::Zero(nc2010_dmp->getNumDimensions());
        nc2010_dmp->getInitialGoal(new_goal);
        for (int i = 0; i < nc2010_dmp->getNumDimensions(); i++)
        {
            new_goal(i) = new_goal(i) + 3.14 / 12;
        }
        nc2010_dmp->changeGoal(new_goal);
        //  
        nc2010_dmp->propagateFull(rollout, initial_duration, trajectory.getNumContainedSamples() * 1.5);
        std::string rollout_joint_file_name = "rollout_joint_reproduction.clmc";
        std::string abs_rollout_joint_file_name = package_path + "/demonstrations/" + rollout_joint_file_name;
        rollout.writeToCLMCFile(abs_rollout_joint_file_name);

        trajectory_msgs::JointTrajectory JointTrajectory;
        JointTrajectory.joint_names.resize(rollout.getDimension());
        for (int i = 0; i < rollout.getDimension(); i++)
        {
            std::ostringstream ss;
            ss << i;
            JointTrajectory.joint_names[i] = "joint_" + ss.str();
        }
        std::cout << std::endl << rollout.getNumContainedSamples() << " " << rollout.getNumTotalCapacity()  << " " << rollout.getSamplingFrequency() << std::endl;
    
        double sampling_frequency = rollout.getSamplingFrequency();
        JointTrajectory.points.resize(rollout.getNumContainedSamples());

        for (int i = 0; i < JointTrajectory.points.size(); i++)
        {
            JointTrajectory.points[i].positions.resize(rollout.getDimension());
            JointTrajectory.points[i].velocities.resize(rollout.getDimension());
            JointTrajectory.points[i].accelerations.resize(rollout.getDimension());
            JointTrajectory.points[i].time_from_start = ros::Duration(i / sampling_frequency);

            for (int j = 0; j < rollout.getDimension(); j++)
            {
                rollout.getTrajectoryPosition(i, j, JointTrajectory.points[i].positions[j]);
                rollout.getTrajectoryVelocity(i, j, JointTrajectory.points[i].velocities[j]);
                rollout.getTrajectoryAcceleration(i, j, JointTrajectory.points[i].accelerations[j]);
            }
        }

        //for (int i = 0; i < JointTrajectory.points.size() ; i++)
        //{
        //    for (int j = 0; j < rollout.getDimension(); j++)
        //    {
        //        std::cout << JointTrajectory.points[i].positions[j] << " ";
        //    }
        //    std::cout << std::endl;
        //}

        std::string topic_name = "/trajectory_msgs";
        std::string msg_bag_file_name = "dmp_rollout_newgoal.bag";
        std::string abs_msg_bag_file_name = package_path + "/demonstrations/" + msg_bag_file_name;

        usc_utilities::FileIO<trajectory_msgs::JointTrajectory>::writeToBagFile(JointTrajectory, topic_name, abs_msg_bag_file_name);

        std::cout << -JointTrajectory.points[0].time_from_start + JointTrajectory.points[JointTrajectory.points.size() - 1].time_from_start << std::endl;



        for (int i = 0; i < trajectory.getDimension(); i++)
        {
            std::cout << JointTrajectory.joint_names[i] << " ";
        }

        std::cout << initial_duration << std::endl;
        
        for (int i = 0; i < robot_part_names.size(); i++)
        {
            std::cout << robot_part_names[i] << std::endl;
        }

        std::cout << abs_bag_file_name << std::endl;
        
        ros::shutdown();
    }


}
#endif


