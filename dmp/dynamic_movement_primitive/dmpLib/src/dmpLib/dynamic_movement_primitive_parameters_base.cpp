/*************************************************************************
	> File Name: dynamic_movement_primitive_parameters_base.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 18 Oct 2016 05:41:07 PM PDT
 ************************************************************************/

// system includes
#include <stdio.h>

// local includes 
#include "dmp_lib/dynamic_movement_primitive_parameters_base.h"
#include "dmp_lib/logger.h"

namespace dmp_lib
{
    bool DynamicMovementPrimitiveParametersBase::initialize(const Time& initial_time, const double teaching_duration, const double execution_duration, const double cutoff, const int type, const int id)
    {
        // TODO: think whether to check if initial_time has a non-zero value 
        initial_time_ = initial_time;

        if(teaching_duration < 0)
        {
            Logger::logPrintf("Teaching duration >%f< is invalid", Logger::ERROR, teaching_duration);
            return false;
        }
        teaching_duration_ = teaching_duration;

        if (execution_duration < 0)
        {
            Logger::logPrintf("Execution duration >%f< is invalid", Logger::ERROR, execution_duration);
            return false;
        }
        execution_duration_ = execution_duration;
        type_ = type;
        id_ = id;
        return setCutoff(cutoff);
    } 

    bool DynamicMovementPrimitiveParametersBase::setCutoff(const double cutoff)
    {
        if (cutoff < 1e-10)
        {
            Logger::logPrintf("Invalid cutoff specified >%f<.", Logger::ERROR, cutoff);
            return false;
        }
        cutoff_ = cutoff;
        return true;
    }

    bool DynamicMovementPrimitiveParametersBase::isCompatible(const DynamicMovementPrimitiveParametersBase& other_parameters_base) const
    {
        if (initial_time_ != other_parameters_base.initial_time_)
        {
            Logger::logPrintf("Initial time is not compatible (dt=>%.4f<,tau=>%.4f<) vs (dt=>%.4f<,tau=>%.4f<)).", Logger::ERROR, initial_time_.getDeltaT(), initial_time_.getTau(), other_parameters_base.initial_time_.getDeltaT(), other_parameters_base.initial_time_.getTau());
            return false; 
        }

        if (cutoff_ != other_parameters_base.cutoff_)
        {
            Logger::logPrintf("Cutoffs >%f< and >%f< are not compatible.", Logger::ERROR, cutoff_, other_parameters_base.cutoff_);
            return false;
        }
        return true;
    }

    bool DynamicMovementPrimitiveParametersBase::changeType(const DynamicMovementPrimitiveParametersBase& other_parameters_base)
    {
        // TODO: change this !!!
        // only change type if they are different and not greater than 3
        if((type_ != other_parameters_base.type_) && (type_ < 3) && (other_parameters_base.type_ < 3))
        {
            type_ += other_parameters_base.type_;
        }
        return true;
    }

    bool DynamicMovementPrimitiveParametersBase::get(Time& initial_time, double& teaching_duration, double& execution_duration, double& cutoff, int& type, int &id) const
    {
        initial_time = initial_time_;
        teaching_duration = teaching_duration_;
        execution_duration = execution_duration_;
        cutoff = cutoff_;
        type = type_;
        id = id_;
        return true;
    }

}


