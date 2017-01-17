/*************************************************************************
	> File Name: robot_pose_visualizer.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 30 Nov 2016 09:18:00 PM PST
 ************************************************************************/

#ifndef _ROBOT_POSE_VISUALIZER_H
#define _ROBOT_POSE_VISUALIZER_H

#include <ros/ros.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/trajectory_monitor.h>
#include <kdl/jntarray.hpp>
#include <robot_info/robot_info.h>

namespace visualization_utilities
{
    class RobotPoseVisualizer
    {
        public:
            RobotPoseVisualizer(const std::string& topic_name, const robot_model::RobotModelConstPtr &robot_model, const boost::shared_ptr<tf::Transformer> &tf);
            virtual ~RobotPoseVisualizer() {};

            void publishPose(const std::string& robot_part_name, const std::string& frame_id, const std::vector<double> joint_angles);
            void publishPose(const std::string& robot_part_name, const std::string& frame_id, const KDL::JntArray& joint_angles);

        private:
            ros::NodeHandle node_handle_;
            ros::Publisher pub_;
            planning_scene_monitor::CurentStateMonitor current_state_monitor_;
    };
}
#endif
