/*************************************************************************
	> File Name: dynamic_movement_primitive_utilities_test_1.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 11 Jan 2017 11:55:19 AM PST
 ************************************************************************/

// system includes 
#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <robot_info/robot_info_init.h>

#include <Eigen/Eigen>

#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>

// local includes 
#include "dynamic_movement_primitive_utilities/dynamic_movement_primitive_learner.h"
#include "dynamic_movement_primitive_utilities/trajectory_utilities.h"

using namespace std;
using namespace dmp_utilities;
using namespace Eigen;

static const std::string rel_joint_space_dmp_bag_file_name = "/test/data/test_joint_space_dmp.bag";
static const std::string rel_cartesian_space_dmp_bag_file_name = "/test/data/test_cartesian_space_dmp.bag";
static const std::string rel_cartesian_and_joint_space_dmp_bag_file_name = "/test/data/test_cartesian_and_joint_space_dmp.bag";

static const std::string rel_demonstration_joint_file_name = "/test/data/test_rollout_joint_demonstration.clmc";
static const std::string rel_rollout_joint_file_name = "/test/data/test_rollout_joint_reproduction.clmc";
static const std::string rel_rollout_joint_file_name_test_1 = "/test/data/test_rollout_joint_reproduction_test_1.clmc";
static const std::string rel_rollout_joint_file_name_test_2 = "/test/data/test_rollout_joint_reproduction_test_2.clmc";

static const std::string rel_rollout_cartesian_file_name = "/test/data/test_rollout_cartesian_reproduction.clmc";
static const std::string rel_rollout_cartesian_file_name_test_1 = "/test/data/test_rollout_cartesian_reproduction_test_1.clmc";
static const std::string rel_rollout_cartesian_file_name_test_2 = "/test/data/test_rollout_cartesian_reproduction_test_2.clmc";

static const std::string rel_rollout_cartesian_and_joint_file_name = "/test/data/test_rollout_cartesian_and_joint_reproduction.clmc";
static const std::string rel_rollout_cartesian_and_joint_file_name_test_1 = "/test/data/test_rollout_cartesian_and_joint_reproduction_test_1.clmc";
static const std::string rel_rollout_cartesian_and_joint_file_name_test_2 = "/test/data/test_rollout_cartesian_and_joint_reproduction_test_2.clmc";

TEST(dmp_learner_tests, learnJointSpaceDMP)
{
    dmp_lib::NC2010DMPPtr nc2010_dmp;
    ros::NodeHandle node_handle("~");
    std::string rel_bag_file_name;
    ROS_VERIFY(usc_utilities::read(node_handle, "rel_bag_file_name", rel_bag_file_name));
    string package_path = ros::package::getPath("dynamic_movement_primitive_utilities");
    string abs_bag_file_name_joint_states = package_path + rel_bag_file_name;
    std::vector<std::string> robot_part_names;

    ROS_VERIFY(usc_utilities::read(node_handle, "robot_part_names", robot_part_names));

    std::vector<std::string> joint_variable_names;
    EXPECT_TRUE(robot_info::RobotInfo::getArmJointNames(robot_part_names, joint_variable_names));
    dmp_lib::Trajectory trajectory;
    EXPECT_TRUE(TrajectoryUtilities::createJointStateTrajectory(trajectory, joint_variable_names, abs_bag_file_name_joint_states, robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY));
    string abs_demo_joint_file_name = package_path + rel_demonstration_joint_file_name;
    EXPECT_TRUE(trajectory.writeToCLMCFile(abs_demo_joint_file_name));

    bool result = DynamicMovementPrimitiveLearner<dmp::NC2010DMP>::learnJointSpaceDMP(nc2010_dmp, node_handle, abs_bag_file_name_joint_states, robot_part_names);
    EXPECT_TRUE(result);

    /*! string abs_bag_file_name_dmp = package_path + rel_joint_space_dmp_bag_file_name;
    EXPECT_TRUE(dmp::NC2010DynamicMovementPrimitive::writeToDisc(nc2010_dmp, abs_bag_file_name_dmp));

    dmp_lib::NC2010DMPPtr nc2010_dmp_copy;
    EXPECT_TRUE(dmp::NC2010DynamicMovementPrimitive::readFromDisc(nc2010_dmp_copy, abs_bag_file_name_dmp));

    double initial_duration = 0;
    EXPECT_TRUE(nc2010_dmp_copy->getInitialDuration(initial_duration));

    // setup dmp, propagate full and write out trajectory containing 1000 samples to file.
    EXPECT_TRUE(nc2010_dmp_copy->setup());
    dmp_lib::Trajectory rollout;
    EXPECT_TRUE(nc2010_dmp_copy->propagateFull(rollout, initial_duration, 1000));
    EXPECT_TRUE(rollout.writeToCLMCFile(package_path + rel_rollout_joint_file_name));

    // setup dmp, propagate full and write output trajectory containing 2000 samples to file.
    dmp_lib::Trajectory rollout_test_1;
    EXPECT_TRUE(nc2010_dmp_copy->setup());

    // write dmp (which is now setup) again to disc 
    EXPECT_TRUE(dmp::NC2010DynamicMovementPrimitive::writeToDisc(nc2010_dmp_copy, abs_bag_file_name_dmp));
    EXPECT_TRUE(nc2010_dmp_copy->propagateFull(rollout_test_1, initial_duration, 2000));
    EXPECT_TRUE(rollout_test_1.writeToCLMCFile(package_path + rel_rollout_joint_file_name_test_1));

    // setup dmp, change goal, propagate full and write output trajectory to file.
    VectorXd new_goal = VectorXd::Zero(nc2010_dmp_copy->getNumDimensions());
    EXPECT_TRUE(nc2010_dmp_copy->setup());
    ROS_VERIFY(nc2010_dmp_copy->getInitialGoal(new_goal));
    for (int i = 0; i < nc2010_dmp_copy->getNumDimensions(); ++i)
    {
        new_goal(i) = new_goal(i) + i;
    }
    EXPECT_TRUE(nc2010_dmp_copy->changeGoal(new_goal));
    dmp_lib::Trajectory rollout_test_2;
    EXPECT_TRUE(nc2010_dmp_copy->propagateFull(rollout_test_2, initial_duration, 2000));
    EXPECT_TRUE(rollout_test_2.writeToCLMCFile(package_path + rel_rollout_joint_file_name_test_2)); */
}

/*! TEST(dmp_learner_utilities, learnCartesianSpaceDMP)
{
    dmp_lib::NC2010DMPPtr nc2010_dmp;
    ros::NodeHandle node_handle("~");
    std::string rel_bag_file_name;
    ROS_VERIFY(usc_utilities::read(node_handle, "rel_bag_file_name", rel_bag_file_name));
    string package_path = ros::package::getPath("dynamic_movement_primitive_utilities");
    string abs_bag_file_name_joint_states = package_path + rel_bag_file_name;
    std::vector<std::string> robot_part_names;
    ROS_VERIFY(usc_utilities::read(node_handle, "robot_part_names", robot_part_names));
    bool result = DynamicMovementPrimitiveLearner<dmp::NC2010DMP>::learnCartesianSpaceDMP(nc2010_dmp, node_handle, abs_bag_file_name_joint_states, robot_part_names);
    EXPECT_TRUE(result);

    string abs_bag_file_name_dmp = package_path + rel_cartesian_space_dmp_bag_file_name;
    EXPECT_TRUE(dmp::NC2010DynamicMovementPrimitive::writeToDisc(nc2010_dmp, abs_bag_file_name_dmp));

    dmp_lib::NC2010DMPPtr nc2010_dmp_copy;
    EXPECT_TRUE(dmp::NC2010DynamicMovementPrimitive::readFromDisc(nc2010_dmp_copy, abs_bag_file_name_dmp));

    double initial_duration = 0;
    EXPECT_TRUE(nc2010_dmp_copy->getInitialDuration(initial_duration));

    // setup dmp, propagate full and write ouput trajectory 
    EXPECT_TRUE(nc2010_dmp_copy->setup());
    dmp_lib::Trajectory rollout;
    EXPECT_TRUE(nc2010_dmp_copy->propagateFull(rollout, initial_duration, 1000));
    EXPECT_TRUE(rollout.writeToCLMCFile(package_path + rel_rollout_cartesian_file_name));

}

TEST(dmp_learner_test, learnCartesianAndJointSpaceDMP)
{
    dmp_lib::NC2010DMPPtr nc2010_dmp;
    ros::NodeHandle node_handle("~");
    std::string rel_bag_file_name;
    ROS_VERIFY(usc_utilities::read(node_handle, "rel_bag_file_name", rel_bag_file_name));
    string package_path = ros::package::getPath("dynamic_movement_primitive_utilities");
    string abs_bag_file_name_joint_states = package_path + rel_bag_file_name;
    std::vector<std::string> robot_part_names;
    ROS_VERIFY(usc_utilities::read(node_handle, "robot_part_names", robot_part_names));
    bool result = DynamicMovementPrimitiveLearner<dmp::NC2010DMP>::learnCartesianAndJointSpaceDMP(nc2010_dmp, node_handle, abs_bag_file_name_joint_states, robot_part_names);
    EXPECT_TRUE(result);

    string abs_bag_file_name_dmp = package_path + rel_cartesian_and_joint_space_dmp_bag_file_name;
    EXPECT_TRUE(dmp::NC2010DynamicMovementPrimitive::writeToDisc(nc2010_dmp, abs_bag_file_name_dmp));

    dmp_lib::NC2010DMPPtr nc2010_dmp_copy;
    EXPECT_TRUE(dmp::NC2010DynamicMovementPrimitive::readFromDisc(nc2010_dmp_copy, abs_bag_file_name_dmp));

    double initial_duration = 0;
    EXPECT_TRUE(nc2010_dmp_copy->getInitialDuration(initial_duration));

    // setup dmp, propagate full and write ouput trajectory 
    EXPECT_TRUE(nc2010_dmp_copy->setup());

    // write dmp (which is now setup up) again to disc 
    EXPECT_TRUE(dmp::NC2010DynamicMovementPrimitive::writeToDisc(nc2010_dmp_copy, abs_bag_file_name_dmp));

    dmp_lib::Trajectory rollout;
    EXPECT_TRUE(nc2010_dmp_copy->propagateFull(rollout, initial_duration, 1000));
    EXPECT_TRUE(rollout.writeToCLMCFile(package_path + rel_rollout_cartesian_and_joint_file_name));
} */

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "dmp_learner_tests");
    robot_info::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}




