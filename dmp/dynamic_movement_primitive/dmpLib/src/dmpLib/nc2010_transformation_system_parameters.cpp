/*************************************************************************
	> File Name: nc2010_transformation_system_parameters.cpp
	> Author: 
	> Mail: 
	> Created Time: Mon 31 Oct 2016 07:53:45 PM PDT
 ************************************************************************/

// system includes
#include <stdio.h>

// local includes
#include "dmp_lib/nc2010_transformation_system_parameters.h"
#include "dmp_lib/logger.h"

namespace dmp_lib
{
    bool NC2010TransformationSystemParameters::initialize(const double k_gain, const double d_gain)
    {
        if (k_gain < 0)
        {
            Logger::logPrintf("Invalid K gain >%f<. Cannot initialize NC2010 transformation system.", Logger::ERROR, k_gain);
            return false;
        }
  
        k_gain_ = k_gain;
        if (d_gain < 0)
        {
            Logger::logPrintf("Invalid D gain >%f<. Cannot initialize NC2010 transformation system.", Logger::ERROR, d_gain);
            return false;
        }
        
        d_gain_ = d_gain;
        return true;
    }

    bool NC2010TransformationSystemParameters::get(double& k_gain, double& d_gain) const
    {
        // TODO: think about error/init checking...
        k_gain = k_gain_;
        d_gain = d_gain_;
        return true;
    }

    bool NC2010TransformationSystemParameters::initialize(const lwr_lib::LWRPtr lwr_model,
                                                          const std::string& name,
                                                          const double k_gain, const double d_gain,
                                                          const double initial_start, const double initial_goal)
    {
        if (!TransformationSystemParametersBase::initialize(lwr_model, name, initial_start, initial_goal))
        {
            Logger::logPrintf("Could not initialize NC2010 transformation system.", Logger::ERROR);
            return false;
        }
  
        if (!initialize(k_gain, d_gain))
        {
            return false;
        }
        return true;
    }
}

