/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		icra2009_transformation_system.cpp

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 9, 2010

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes
#include <dynamic_movement_primitive/icra2009_transformation_system.h>
#include <dynamic_movement_primitive/state.h>

using namespace std;

namespace dmp
{

bool ICRA2009TransformationSystem::initOneDimensionalTransformationSystemHelper(std::vector<dmp_lib::ICRA2009TSPtr>& transformation_systems,
                                                                                XmlRpc::XmlRpcValue transformation_systems_parameters_xml,
                                                                                ros::NodeHandle& node_handle,
                                                                                dmp_lib::TransformationSystem::IntegrationMethod integration_method)
{
  ROS_ASSERT(transformation_systems_parameters_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int j = 0; j < transformation_systems_parameters_xml.size(); ++j)
  {
    ROS_DEBUG("Initializing ICRA2009 transformation system number >%i< from node handle.",j);

    // create transformation system
    dmp_lib::ICRA2009TSPtr transformation_system(new dmp_lib::ICRA2009TransformationSystem());

    XmlRpc::XmlRpcValue ts_xml = transformation_systems_parameters_xml[j];
    ROS_ASSERT(ts_xml.hasMember(transformation_system->getVersionString()));
    XmlRpc::XmlRpcValue icra2009_ts_xml = ts_xml[transformation_system->getVersionString()];

    double k_gain = 0;
    if (!usc_utilities::getParam(icra2009_ts_xml, "k_gain", k_gain))
    {
      return false;
    }
    double d_gain = dmp_lib::ICRA2009TransformationSystem::getDGain(k_gain);

    // create parameters
    dmp_lib::ICRA2009TSParamPtr parameters(new dmp_lib::ICRA2009TransformationSystemParameters());

    // initialize parameters
    ROS_VERIFY(parameters->initialize(k_gain, d_gain));

    // initialize base class
    ROS_VERIFY(TransformationSystem::initFromNodeHandle(parameters, ts_xml, node_handle));

    // create state
    dmp_lib::ICRA2009TSStatePtr state(new dmp_lib::ICRA2009TransformationSystemState());

    // initialize transformation system with parameters and state
    ROS_VERIFY(transformation_system->initialize(parameters, state, integration_method));
    ROS_VERIFY(transformation_system->setIntegrationMethod(integration_method));

    transformation_systems.push_back(transformation_system);
  }
  return true;
}

bool ICRA2009TransformationSystem::initMultiDimensionalTransformationSystemHelper(std::vector<dmp_lib::ICRA2009TSPtr>& transformation_systems,
                                                                                  XmlRpc::XmlRpcValue transformation_systems_parameters_xml,
                                                                                  ros::NodeHandle& node_handle,
                                                                                  dmp_lib::TransformationSystem::IntegrationMethod integration_method)
{
  ROS_INFO("Initializing ICRA2009 orientation transformation system from node handle.");
  std::vector<dmp_lib::ICRA2009TSParamPtr> parameters_vector;
  std::vector<dmp_lib::ICRA2009TSStatePtr> state_vector;

  // create one transformation system
  dmp_lib::ICRA2009TSPtr transformation_system(new dmp_lib::ICRA2009TransformationSystem());

  ROS_ASSERT(transformation_systems_parameters_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int j = 0; j < transformation_systems_parameters_xml.size(); ++j)
  {
    XmlRpc::XmlRpcValue ts_xml = transformation_systems_parameters_xml[j];
    ROS_ASSERT(ts_xml.hasMember(transformation_system->getVersionString()));
    XmlRpc::XmlRpcValue icra2009_ts_xml = ts_xml[transformation_system->getVersionString()];
    double k_gain = 0;
    if (!usc_utilities::getParam(icra2009_ts_xml, "k_gain", k_gain))
    {
      return false;
    }
    double d_gain = dmp_lib::ICRA2009TransformationSystem::getDGain(k_gain);

    // create parameters
    dmp_lib::ICRA2009TSParamPtr parameters(new dmp_lib::ICRA2009TransformationSystemParameters());

    // initialize transformation system parameters
    ROS_VERIFY(parameters->initialize(k_gain, d_gain));

    // initialize base class
    ROS_VERIFY(TransformationSystem::initFromNodeHandle(parameters, ts_xml, node_handle));

    // create state
    dmp_lib::ICRA2009TSStatePtr state(new dmp_lib::ICRA2009TransformationSystemState());

    state_vector.push_back(state);
    parameters_vector.push_back(parameters);
  }

  // initialize transformation system with multiple parameters and states
  ROS_VERIFY(transformation_system->initialize(parameters_vector, state_vector, integration_method));
  ROS_VERIFY(transformation_system->setIntegrationMethod(integration_method));
  transformation_systems.push_back(transformation_system);

  return true;
}

bool ICRA2009TransformationSystem::initFromNodeHandle(std::vector<dmp_lib::ICRA2009TSPtr>& transformation_systems,
                                                      ros::NodeHandle& node_handle)
{
  ROS_DEBUG("Initializing ICRA2009 transformation system from node handle in namespace >%s<.", node_handle.getNamespace().c_str());

  // read one dimensional transformation systems from node handle
  XmlRpc::XmlRpcValue transformation_systems_parameters_xml;
  if (!node_handle.getParam("transformation_systems_parameters", transformation_systems_parameters_xml))
  {
    ROS_ERROR("Node handle in namespace >%s< must contain the >transformation_systems_parameters< array.", node_handle.getNamespace().c_str());
    return false;
  }
  transformation_systems.clear();
  if (!ICRA2009TransformationSystem::initOneDimensionalTransformationSystemHelper(transformation_systems, transformation_systems_parameters_xml, node_handle))
  {
    return false;
  }
  return true;
}

bool ICRA2009TransformationSystem::initFromNodeHandle(std::vector<dmp_lib::ICRA2009TSPtr>& transformation_systems,
                                                      const std::vector<std::string>& robot_part_names,
                                                      ros::NodeHandle& node_handle)
{
  if(robot_part_names.empty())
  {
    return ICRA2009TransformationSystem::initFromNodeHandle(transformation_systems, node_handle);
  }

  ROS_DEBUG("Initializing ICRA2009 transformation system from node handle in namespace >%s<.", node_handle.getNamespace().c_str());
  transformation_systems.clear();
  for (int i = 0; i < (int)robot_part_names.size(); ++i)
  {
    ROS_DEBUG("Using robot part: >%s<.", robot_part_names[i].c_str());

    // get transformation system for each robot part
    ros::NodeHandle robot_part_node_handle(node_handle, robot_part_names[i]);

    // read one dimensional transformation systems from node handle
    XmlRpc::XmlRpcValue transformation_systems_parameters_xml;
    bool initialized = false;
    if (robot_part_node_handle.getParam("transformation_systems_parameters", transformation_systems_parameters_xml))
    {
      if (!ICRA2009TransformationSystem::initOneDimensionalTransformationSystemHelper(transformation_systems, transformation_systems_parameters_xml, node_handle))
      {
        return false;
      }
      initialized = true;
    }

    // read orientation transformation system from node handle
    if (robot_part_node_handle.getParam("orientation_transformation_system", transformation_systems_parameters_xml))
    {
      if (!ICRA2009TransformationSystem::initMultiDimensionalTransformationSystemHelper(transformation_systems, transformation_systems_parameters_xml, node_handle, dmp_lib::TransformationSystem::QUATERNION))
      {
        return false;
      }
      initialized = true;
    }

    if(!initialized)
    {
      ROS_ERROR("Namespace >%s< must either contain >transformation_systems_parameters< or >orientation_transformation_system<. Could not initialize ICRA2009 DMP from node handle.", robot_part_node_handle.getNamespace().c_str());
      return false;
    }
  }
  ROS_DEBUG("Done learning ICRA2009 transformation system.");
  return true;
}

bool ICRA2009TransformationSystem::initFromMessage(std::vector<dmp_lib::ICRA2009TSPtr>& transformation_systems,
                                                   const vector<ICRA2009TSMsg>& ts_msgs)
{
  ROS_DEBUG("Initializing ICRA2009 transformation system from message.");

  transformation_systems.clear();
  for (int i = 0; i < (int)ts_msgs.size(); ++i)
  {

    if(ts_msgs[i].parameters.size() != ts_msgs[i].states.size())
    {
      ROS_ERROR("Cannot initialize transformation system with >%i< parameters and >%i< states.", (int)ts_msgs[i].parameters.size(), (int)ts_msgs[i].states.size());
      return false;
    }

    vector<dmp_lib::ICRA2009TSParamPtr> parameters_vector;
    vector<dmp_lib::ICRA2009TSStatePtr> state_vector;
    for (int j = 0; j < (int)ts_msgs[i].parameters.size(); ++j)
    {
      // create parameters
      dmp_lib::ICRA2009TSParamPtr parameters(new dmp_lib::ICRA2009TransformationSystemParameters());

      // initialize transformation system parameters from message
      ROS_VERIFY(parameters->initialize(ts_msgs[i].parameters[j].k_gain, ts_msgs[i].parameters[j].d_gain));

      // create state
      dmp_lib::ICRA2009TSStatePtr state(new dmp_lib::ICRA2009TransformationSystemState());

      // initialize base class
      ROS_VERIFY(TransformationSystem::initFromMessage(parameters, state,
                                                       ts_msgs[i].transformation_system.parameters[j],
                                                       ts_msgs[i].transformation_system.states[j]));

      parameters_vector.push_back(parameters);
      state_vector.push_back(state);
    }

    // create transformation system
    dmp_lib::ICRA2009TSPtr transformation_system(new dmp_lib::ICRA2009TransformationSystem());
    dmp_lib::TransformationSystem::IntegrationMethod integration_method = dmp_lib::TransformationSystem::NORMAL;
    if(ts_msgs[i].transformation_system.integration_method == (int)dmp_lib::TransformationSystem::NORMAL)
    {
      integration_method = dmp_lib::TransformationSystem::NORMAL;
    }
    else if(ts_msgs[i].transformation_system.integration_method == (int)dmp_lib::TransformationSystem::QUATERNION)
    {
      integration_method = dmp_lib::TransformationSystem::QUATERNION;
    }
    else
    {
      ROS_ERROR("Invalid integration method >%i<. Cannot initialize transformation system.", ts_msgs[i].transformation_system.integration_method);
    }
    ROS_VERIFY(transformation_system->initialize(parameters_vector, state_vector, integration_method));
    transformation_systems.push_back(transformation_system);
  }
  return true;
}

bool ICRA2009TransformationSystem::writeToMessage(const std::vector<dmp_lib::ICRA2009TSConstPtr> transformation_systems,
                                                  std::vector<ICRA2009TSMsg>& ts_msgs)
{
  ts_msgs.clear();
  for (int i = 0; i < (int)transformation_systems.size(); ++i)
  {
    ROS_DEBUG("Writing ICRA2009 transformation system >%i< to message.", i);

    vector<dmp_lib::ICRA2009TSParamConstPtr> parameters_vector;
    vector<dmp_lib::ICRA2009TSStateConstPtr> state_vector;
    ROS_VERIFY(transformation_systems[i]->get(parameters_vector, state_vector));

    ICRA2009TSMsg ts_msg;
    ts_msg.parameters.resize(parameters_vector.size());
    ts_msg.states.resize(state_vector.size());
    for (int j = 0; j < (int)parameters_vector.size(); ++j)
    {
      // set parameters
      ROS_VERIFY(parameters_vector[j]->get(ts_msg.parameters[j].k_gain, ts_msg.parameters[j].d_gain));
      // state is empty
    }
    // write base class
    ROS_VERIFY(TransformationSystem::writeToMessage(transformation_systems[i], ts_msg.transformation_system));

    // add to transformation system messages
    ts_msgs.push_back(ts_msg);
  }
  return true;
}

}
