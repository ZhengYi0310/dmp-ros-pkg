/*************************************************************************
	> File Name: transformation_system.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 15 Nov 2016 07:05:42 PM PST
 ************************************************************************/
// system includes 
#include <stdio.h>
#include <dynamic_movement_primitive/locally_weighted_regression.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes 
#include "dynamic_movement_primitive/transformation_system.h"
#include "dynamic_movement_primitive/state.h"

using namespace std;

namespace dmp
{
    bool TransformationSystem::initFromNodeHandle(dmp_lib::TSParamBasePtr parameters,
                                                  XmlRpc::XmlRpcValue& ts_xml,
                                                  ros::NodeHandle& node_handle)
    {
        double cutoff = 0;
        if (!usc_utilities::read(node_handle, "cutoff", cutoff))
        {
            ROS_ERROR("Could not read transformation system parameter >cutoff< from param server in namespace >%s<.", node_handle.getNamespace().c_str());
            return false;
        }

        std::string name;
        if (!usc_utilities::getParam(ts_xml, "name", name))
        {
            ROS_ERROR("Could not read transformation system parameter >name< from param server in namespace >%s<.", node_handle.getNamespace().c_str());
            return false;   
        }

        // create lwr model and initialize lwr model from node handle 
        lwr::LWRPtr lwr_model(new lwr::LocallyWeightedRegression());
        if (!lwr_model->initFromNodeHandle(node_handle, cutoff))
        {
            return false;
        }

        // initialize transformation system parameters from message 
        if (!parameters->initialize(lwr_model->getModel(), name))
        {
            return false;
        }
        return true;
    }

    bool TransformationSystem::initFromMessage(dmp_lib::TSParamBasePtr parameters,
                                               dmp_lib::TSStateBasePtr state,
                                               const TSParamMsg& ts_param_msg,
                                               const TSStateMsg& ts_state_msg)
    {
        // create lwr model and ininitial lwr model from message  
        lwr::LWRPtr lwr_model(new lwr::LocallyWeightedRegression());
        if (!lwr_model->initFromMessage(ts_param_msg.lwr_model))
        {
            return false;
        }

        // initialize transformation system parameters from message
        if (!parameters->initialize(lwr_model->getModel(), ts_param_msg.name, ts_param_msg.initial_start, ts_param_msg.initial_goal))
        {
            return false;
        }

        // initialize transformation system state from message 
        State internal(ts_state_msg.internal);
        State target(ts_state_msg.target);
        State current(ts_state_msg.current);
        if (!state->set(internal.getState(), target.getState(), current.getState(), ts_state_msg.start, ts_state_msg.goal, ts_state_msg.f, ts_state_msg.ft))
        {
            return false;
        }
        return true;
    }

    bool TransformationSystem::writeToMessage(const dmp_lib::TSBaseConstPtr transformation_system, TSMsg& ts_msg)
    {
        ROS_DEBUG("Writing transformation system to message.");
        vector<dmp_lib::TSParamBaseConstPtr> parameters;
        vector<dmp_lib::TSStateBaseConstPtr> states;

        dmp_lib::TransformationSystemBase::IntegrationMethod integration_method;
        if (!transformation_system->get(parameters, states, integration_method))
        {
            return false;
        }
        
        ts_msg.integration_method = static_cast<int>(integration_method);
        ts_msg.parameters.resize(parameters.size());

        for (int i = 0; i < (int)parameters.size(); ++i)
        {
            // set parameters
            lwr_lib::LWRConstPtr lwr_model;
            if (!parameters[i]->get(lwr_model, ts_msg.parameters[i].name, ts_msg.parameters[i].initial_start, ts_msg.parameters[i].initial_goal))
            {
                return false;
            }
            ROS_VERIFY(lwr::writeToMessage(lwr_model, ts_msg.parameters[i].lwr_model));
        }

        ts_msg.states.resize(states.size());
        for (int i = 0; i < (int)states.size(); ++i)
        {
            // set states
            dmp_lib::State internal, target, current;
            if (!states[i]->get(internal, target, current,
                                ts_msg.states[i].start, ts_msg.states[i].goal,
                                ts_msg.states[i].f, ts_msg.states[i].ft))
            {
                return false;
            }

            internal.get(ts_msg.states[i].internal.x, ts_msg.states[i].internal.xd, ts_msg.states[i].internal.xdd);
            target.get(ts_msg.states[i].target.x, ts_msg.states[i].target.xd, ts_msg.states[i].target.xdd);
            current.get(ts_msg.states[i].current.x, ts_msg.states[i].current.xd, ts_msg.states[i].current.xdd);
        }

        ROS_DEBUG("Done writing transformation system to message.");
        return true;


    }

}

