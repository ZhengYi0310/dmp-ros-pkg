/*************************************************************************
	> File Name: canonical_system_base.cpp
	> Author: 
	> Mail: 
	> Created Time: Wed 26 Oct 2016 12:17:10 PM PDT
 ************************************************************************/
// system includes
#include<stdio.h>

// local includes 
#include "dmp_lib/canonical_system_base.h"
#include "dmp_lib/logger.h"
#include "dmp_lib/utilities.h"

namespace dmp_lib
{
    bool CanonicalSystemBase::initialize(const CSParamBasePtr parameters, const CSStateBasePtr state)
    {
        Logger::logPrintf("Initializing canonical system.", Logger::DEBUG);
        if (!parameters.get())
        {
            Logger::logPrintf("Cannot initialize canonical system from empty parameters.", Logger::ERROR);
            return false;
        }
        if (!state.get())
        {
            Logger::logPrintf("Cannot initialize canonical system for empty state.", Logger::ERROR);
            return false;
        }
        Logger::logPrintf(initialized_, "Canonical system already initialized. Re-initializing...", Logger::WARN);
        parameters_ = parameters;
        state_ = state;
        return (initialized_ = true);
    }

    bool CanonicalSystemBase::isCompatible(const CanonicalSystemBase& other_cs_base) const
    {
        if (!initialized_)
        {
            Logger::logPrintf("Canonical system not initialized, not compatible.", Logger::ERROR);
            return false;
        }
        if (other_cs_base.initialized_)
        {
            Logger::logPrintf("Other canonical system not initialized, not compatible.", Logger::ERROR);
            return false; 
        }
        return (parameters_->isCompatible(*other_cs_base.parameters_) && state_->isCompatible(*other_cs_base.state_));
    }

    bool CanonicalSystemBase::get(CSParamBaseConstPtr& parameters, CSStateBaseConstPtr& state) const
    {
        if (!initialized_)
        {
            Logger::logPrintf("Canonical system is not initialized, cannot return parameters.", Logger::ERROR);
            return false;
        }
        parameters = parameters_;
        state = state_;
        return true;
    }

}


