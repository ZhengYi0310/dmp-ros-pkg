/*************************************************************************
	> File Name: transformation_system_state_base.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 26 Oct 2016 03:07:07 PM PDT
 ************************************************************************/
// system includes
#include <stdio.h>

// local includes
#include"dmp_lib/transformation_system_state_base.h"

namespace dmp_lib
{
    bool TransformationSystemStateBase::set(const State& internal,
                                        const State& target,
                                        const State& current,
                                        const double start,
                                        const double goal,
                                        const double f,
                                        const double ft/*,
                                        const std::vector<double>& function_input,
                                        const std::vector<double>& function_target*/)
    {
        internal_ = internal;
        target_ = target;
        current_ = current;
        start_ = start;
        goal_ = goal;
        f_ = f;
        ft_ = ft;
        // function_input_ = function_input;
        // function_target_ = function_target;
        return true;
    }

    bool TransformationSystemStateBase::get(State& internal,
                                        State& target,
                                        State& current,
                                        double& start,
                                        double& goal,
                                        double& f,
                                        double& ft/*,
                                        std::vector<double>& function_input,
                                        std::vector<double>& function_target*/) const
    {
        internal = internal_;
        target = target_;
        current = current_;
        start = start_;
        goal = goal_;
        f = f_;
        ft = ft_;
        // function_input = function_input_;
        // function_target = function_target_;
        return true;
    }

    void TransformationSystemStateBase::reset()
    {
        internal_.reset();
    }
}

