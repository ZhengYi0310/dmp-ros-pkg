/*************************************************************************
	> File Name: transformation_system_base.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 27 Oct 2016 12:27:11 PM PDT
 ************************************************************************/

// system includes 
#include <stdio.h>

// local includes 
#include "dmp_lib/transformation_system_base.h"
#include "dmp_lib/logger.h"

using namespace std;
namespace dmp_lib
{
    static const std::string invalid_name = "null";

    bool TransformationSystemBase::initialize(const std::vector<TSParamBasePtr> parameters,
                                              const std::vector<TSStateBasePtr> states,
                                              const IntegrationMethod integration_method)
    {
        Logger::logPrintf("Initializing transfomration system base with >%i< dimensions,", Logger::DEBUG, (int)parameters.size());

        if (parameters.size() != states.size())
        {
            Logger::logPrintf("Number of parameters >%i< and number of states >%i< must be equal. Cannot initialize transfomration system.", Logger::FATAL, (int)parameters.size(), (int)states.size());
            return (initialized_ = false);
        }

        if (parameters.empty())
        {
            Logger::logPrintf("Cannot initialize transformation system from empty parameters.", Logger::FATAL);
            return (initialized_ = false);
        }

        for (int i = 0; i < (int)parameters.size(); ++i)
        {
            if (!parameters[i].get() || !states[i].get())
            {
                Logger::logPrintf("Cannot initialize transfomration system from unset pointers.", Logger::FATAL);
                return (initialized_ = false);
            }
        }

        Logger::logPrintf(initialized_, "Transfomration system already initialized. Re-Initializing...", Logger::WARN);
        parameters_ = parameters;
        states_ = states;
        integration_method_ = integration_method;
        return (initialized_ = true);
    }

    bool TransformationSystemBase::get(std::vector<TSParamBaseConstPtr>& parameters, 
                                       std::vector<TSStateBaseConstPtr>& states, 
                                       IntegrationMethod& integration_method) const 
    {
        if (!initialized_)
        {
            Logger::logPrintf("Transfomration system is not initialized, cannot return parameters.", Logger::ERROR);
            return false;
        }
        parameters.clear();
        states.clear();
        for (int i = 0; i < getNumDimensions(); ++i)
        {
            parameters.push_back(parameters_[i]);
            states.push_back(states_[i]);
        }
        integration_method = integration_method_;
        return true;
    }

    // REAL-TIME REQUIREMENTS
    bool TransformationSystemBase::setCurrentState(const int index, const State& current_state)
    {
        if (index < 0 || index >= getNumDimensions())
        {
            Logger::logPrintf("Cannot set current state of transfomration system with index >%i< (REAL-TIME VIOLATION).", Logger::ERROR, index);
            return false;
        }
        states_[index]->current_ = current_state;
        return true;
    }

    // REAL-TIME REQUIREMENTS
    bool TransformationSystemBase::setCurrentStates(const std::vector<State>& current_states)
    {
        assert(initialized_);
        if ((int)current_states.size() != getNumDimensions())
        {
            Logger::logPrintf("Number of states >%i< does not match the number of states in the transfomration system (REAL-TIME VIOLATION).", Logger::ERROR, (int)current_states.size(), getNumDimensions());
            return false;
        }

        for (int i = 0; i < getNumDimensions(); ++i)
        {
            states_[i]->current_ = current_states[i];
        }
        return true;
    }

    // REAL-TIME REQUIREMENTS
    bool TransformationSystemBase::getCurrentState(const int index, State& state) const
    {
        assert(initialized_);
        if (index < 0 || index >= getNumDimensions())
        {
            Logger::logPrintf("Cannot get current state of transformation system with index >%i< (REAL-TIME VIOLATION).", Logger::ERROR, index);
            return false;
        }
        state = states_[index]->current_;
        return true;
    }

    void TransformationSystemBase::getCurrentStates(std::vector<State>& states) const
    {
        assert(initialized_);
        if ((int)states.size() != getNumDimensions())
        {
            states.clear();
            states.resize(getNumDimensions());
        }
        for (int i = 0; i < getNumDimensions(); ++i)
        {
            states[i] = states_[i]->current_;
        }
    }

    // REAL-TIME REQUIREMENTS
    bool TransformationSystemBase::setStart(const int index, const double start, bool initial)
    {
        assert(initialized_);
        if (index < 0 || index >= getNumDimensions())
        {
            Logger::logPrintf("Cannot set start of transformation system with index >%i< (Real-time violation).", Logger::ERROR, index);
            return false;
        }
        if (initial)
        {
            parameters_[index]->initial_start_ = start;
        }
        else
        {
            states_[index]->start_ = start;
        }
        return true;
    }

    // REAL-TIME REQUIREMENTS
    bool TransformationSystemBase::setStart(const std::vector<double> start, bool initial)
    {
        if ((int)start.size() != getNumDimensions())
        {
            Logger::logPrintf("Start vector dimension >%i< does not match number of dimensions >%i< of the transformation system (Real-time violation).", Logger::ERROR, (int)start.size(), getNumDimensions());
            return false;
        }
        for (int i = 0; i < getNumDimensions(); ++i)
        {
            if (initial)
            {
                parameters_[i]->initial_start_ = start[i];
            }
            else
            {
                states_[i]->start_ = start[i];
            }
        }
        return true;
    }

    // REAL-TIME REQUIREMENTS
    bool TransformationSystemBase::getStart(const int index, double& start, bool initial) const
    {
        assert(initialized_);
        if (index < 0 || index >= getNumDimensions())
        {
            Logger::logPrintf("Cannot get start of transformation system with index >%i< (Real-time violation).", Logger::ERROR, index);
            return false;
        }
        if (initial)
        {
            start = parameters_[index]->initial_start_;
        }
        else
        {
            start = states_[index]->start_;
        }
        return true;
    }

    void TransformationSystemBase::getStart(std::vector<double>& start, bool initial) const
    {
        assert(initialized_);
        if ((int)start.size() != getNumDimensions())
        {
            start.clear();
            start.resize(getNumDimensions());
        }
        for (int i = 0; i < getNumDimensions(); ++i)
        {
            if (initial)
            {
                start[i] = parameters_[i]->initial_start_;
            }
            else
            {
                start[i] = states_[i]->start_;
            }
        }
    }

    // REAL-TIME REQUIREMENTS
    bool TransformationSystemBase::setGoal(const int index, const double goal, bool initial)
    {
        assert(initialized_);
        if (index < 0 || index >= getNumDimensions())
        {
            Logger::logPrintf("Cannot set goal of transformation system with index >%i< (Real-time violation).", Logger::ERROR, index);
            return false;
        }
        if (initial)
        {
            parameters_[index]->initial_goal_ = goal;
        }
        else
        {
            states_[index]->goal_ = goal;
        }
        return true;
    }

    // REAL-TIME REQUIREMENTS
    bool TransformationSystemBase::setGoal(const std::vector<double> goal, bool initial)
    {
        if ((int)goal.size() != getNumDimensions())
        {
            Logger::logPrintf("Goal vector dimension >%i< does not match number of dimensions >%i< of the transformation system (Real-time violation).", Logger::ERROR, (int)goal.size(), getNumDimensions());
            return false;
        }
        for (int i = 0; i < getNumDimensions(); ++i)
        {
            if (initial)
            {
                parameters_[i]->initial_goal_ = goal[i];
            }
            else
            {
                states_[i]->goal_ = goal[i];
            }
        }
        return true;
    }

    // REAL-TIME REQUIREMENTS
    bool TransformationSystemBase::getGoal(const int index, double& goal, bool initial) const
    {
        assert(initialized_);
        if (index < 0 || index >= getNumDimensions())
        {
            Logger::logPrintf("Cannot get goal of transformation system with index >%i< (Real-time violation).", Logger::ERROR, index);
            return false;
        }
        if (initial)
        {
            goal = parameters_[index]->initial_goal_;
        }
        else
        {
            goal = states_[index]->goal_;
        }
        return true;
    }

    void TransformationSystemBase::getGoal(std::vector<double>& goal, bool initial) const
    {
        assert(initialized_);
        if ((int)goal.size() != getNumDimensions())
        {
            goal.clear();
            goal.resize(getNumDimensions());
        }
        for (int i = 0; i < getNumDimensions(); ++i)
        {
            if (initial)
            {
                goal[i] = parameters_[i]->initial_goal_;
            }
            else
            {
                goal[i] = states_[i]->goal_;
            }
        }
    }

    bool TransformationSystemBase::setInitialStart(const int index, const double initial_start)
    {
        return setStart(index, initial_start, true);
    }
    
    bool TransformationSystemBase::setInitialStart(const std::vector<double> initial_start)
    {
        return setStart(initial_start, true);
    }

    bool TransformationSystemBase::getInitialStart(const int index, double& initial_start) const
    {
        return getStart(index, initial_start, true);
    }

    void TransformationSystemBase::getInitialStart(std::vector<double>& initial_start) const
    {
        getStart(initial_start, true);
    }

    bool TransformationSystemBase::setInitialGoal(const int index, const double initial_goal)
    {
        return setGoal(index, initial_goal, true);
    }

    bool TransformationSystemBase::setInitialGoal(const std::vector<double> initial_goal)
    {
        return setGoal(initial_goal, true);
    }

    bool TransformationSystemBase::getInitialGoal(const int index, double& initial_goal) const
    {
        return getGoal(index, initial_goal, true);
    }

    void TransformationSystemBase::getInitialGoal(std::vector<double>& initial_goal) const
    {
        getGoal(initial_goal, true);
    }
    
    bool TransformationSystemBase::getName(const int index, string& name) const
    {
        assert(initialized_);
        if (index < 0 || index >= getNumDimensions())
        {
            Logger::logPrintf("Cannot get name of transformation system with index >%i< (Real-time violation).", Logger::ERROR, index);
            return false;
        }
        name.assign(parameters_[index]->name_);
        return true;
    }

    // REAL-TIME REQUIREMENTS
    const std::string& TransformationSystemBase::getName(const int index) const
    {
        if (index < 0 || index >= getNumDimensions())
        {
            Logger::logPrintf("Cannot get name of transformation system with index >%i< (Real-time violation).", Logger::ERROR, index);
            return invalid_name;
        }
        return parameters_[index]->name_;
    }

    void TransformationSystemBase::getNames(vector<string>& names) const
    {
        assert(initialized_);
        if ((int)names.size() != getNumDimensions())
        {
            names.clear();
            names.resize(getNumDimensions());
        }
        for (int i = 0; i < getNumDimensions(); ++i)
        {
            names[i] = parameters_[i]->name_;
        }
    }

    bool TransformationSystemBase::setIntegrationMethod(IntegrationMethod integration_method)
    {
        if((integration_method == QUATERNION) && (getNumDimensions() != 4))
        {
            Logger::logPrintf("Cannot set integration method to >QUATERNION< because transformation system has incorrect number of dimension >%i<.", Logger::ERROR, getNumDimensions());
            return false;
        }
        integration_method_ = integration_method;
        return true;
    }
}

