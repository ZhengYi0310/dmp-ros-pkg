/*************************************************************************
	> File Name: transformation_system_parameters_base.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 26 Oct 2016 04:31:10 PM PDT
 ************************************************************************/

// system include
#include <cassert>
#include <stdio.h>

// local include
#include "dmp_lib/transformation_system_parameters_base.h"
#include "dmp_lib/logger.h"
#include "dmp_lib/utilities.h"

namespace dmp_lib
{
    TransformationSystemParametersBase& TransformationSystemParametersBase::operator=(const TransformationSystemParametersBase& parameters)
    {
        Logger::logPrintf("TransformationSystemParametersBase assignment.", Logger::DEBUG);
        assert(Utilities<lwr_lib::LWR>::assign(lwr_model_, parameters.lwr_model_));
        initial_start_ = parameters.initial_start_;
        initial_goal_ = parameters.initial_goal_;
        name_.assign(parameters.name_);
        initialized_ = parameters.initialized_;
        return *this;
    }

    bool TransformationSystemParametersBase::initialize(const lwr_lib::LWRPtr lwr_model, const std::string& name, const double initial_start, const double initial_goal)
    {
        Logger::logPrintf("Initializing transformation system parameters.", Logger::DEBUG);
        assert(lwr_model.get());
        if (!lwr_model->isInitialized())
        {
            Logger::logPrintf("Cannot initialize transformation system parameters from uninitialized LWR model.", Logger::ERROR);
            return (initialized_ = false);
        }
        assert(Utilities<lwr_lib::LWR>::assign(lwr_model_, lwr_model));
        name_.assign(name);
        initial_start_ = initial_start;
        initial_goal_ = initial_goal;
        return (initialized_ = true);
    }

    bool TransformationSystemParametersBase::get(lwr_lib::LWRConstPtr& lwr_model,
                                                 std::string& name,
                                                 double& initial_start,
                                                 double& initial_goal) const
    {
        if (!initialized_)
        {
            Logger::logPrintf("Transformation system parameters not initialized. Cannot return parameters.", Logger::ERROR);
            return false;
        }
        lwr_model = lwr_model_;
        name.assign(name_);
        initial_start = initial_start_;
        initial_goal = initial_goal_;
        return true;
    }

    void TransformationSystemParametersBase::setInitialGoal(const double initial_goal)
    {
        initial_goal_ = initial_goal;
    }

    double TransformationSystemParametersBase::getInitialGoal() const 
    {
        return initial_goal_;
    }

    void TransformationSystemParametersBase::setInitialStart(const double initial_start)
    {
        initial_start_ = initial_start_;
    }

    double TransformationSystemParametersBase::getInitialStart() const 
    {
        return initial_start_;
    }
}

