/*************************************************************************
	> File Name: canonical_system_parameters_base.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 26 Oct 2016 10:51:49 AM PDT
 ************************************************************************/

// system includes
#include <math.h>
#include <stdio.h>

// local includes 
#include "dmp_lib/canonical_system_parameters_base.h"
#include "dmp_lib/logger.h"

namespace dmp_lib
{
    bool CanonicalSystemParametersBase::initialize(const double alpha_x)
    {
        if (alpha_x > 1e-10)
        {
            alpha_x_ = alpha_x;
            return (initialized_ = true);
        }
        Logger::logPrintf("Canonical system parameter alpha_x >%f< must be positive. Cannot initialize canonical system.", Logger::ERROR, alpha_x);
        return (initialized_ = false);
    }

    bool CanonicalSystemParametersBase::isCompatible(const CanonicalSystemParametersBase &other_parameters_base) const 
    {
        if(!initialized_)
        {
            Logger::logPrintf("Canonical system parameters not initialized, therefore not compatible.", Logger::ERROR);
            return false;
        }

        if(!other_parameters_base.initialized_)
        {
            Logger::logPrintf("other canonical system paramters not initialized, therefore not compatible.", Logger::ERROR);
            return false;
        }

        if(alpha_x_ != other_parameters_base.alpha_x_)
        {
            Logger::logPrintf("Canonical system parameters not compatible.", Logger::ERROR);
            return false;
        }

        return true;
    }

    bool CanonicalSystemParametersBase::setCutoff(const double cutoff)
    {
        if (cutoff > 1e-10)
        {
            alpha_x_ = -log(cutoff);
            return (initialized_ = true);
        }
        Logger::logPrintf("Invalid cutoff value provided >%f<.", Logger::ERROR, cutoff);
        return (initialized_ = false);
    }
}

