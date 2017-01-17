/*************************************************************************
	> File Name: nc2010_dynamic_movement_primitive.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 01 Nov 2016 01:18:46 PM PDT
 ************************************************************************/

// system includes
#include <stdio.h>

// local includes 
#include "dmp_lib/nc2010_dynamic_movement_primitive.h"
#include "dmp_lib/nc2010_transformation_system.h"
#include "dmp_lib/nc2010_canonical_system.h"
#include "dmp_lib/logger.h"
#include "dmp_lib/utilities.h"

using namespace std;

namespace dmp_lib
{
    NC2010DynamicMovementPrimitive& NC2010DynamicMovementPrimitive::operator=(const NC2010DynamicMovementPrimitive& nc2010dmp)
    {
        Logger::logPrintf("NC2010DynamicMovementPrimitive assignment.", Logger::DEBUG);

        // first assign all member variables 
        assert(Utilities<NC2010DMPParam>::assign(parameters_, nc2010dmp.parameters_));
        assert(Utilities<NC2010DMPState>::assign(state_, nc2010dmp.state_));
        assert(Utilities<NC2010TS>::assign(transformation_systems_, nc2010dmp.transformation_systems_));
        assert(Utilities<NC2010CS>::assign(canonical_system_, nc2010dmp.canonical_system_));

        // then assign base class variables 
        DynamicMovementPrimitiveBase::parameters_ = parameters_;
        DynamicMovementPrimitiveBase::state_ = state_;
        DynamicMovementPrimitiveBase::transformation_systems_.clear();
        for (int i = 0; i < (int)transformation_systems_.size(); ++i)
        {
            DynamicMovementPrimitiveBase::transformation_systems_.push_back(transformation_systems_[i]);
        }
        DynamicMovementPrimitiveBase::canonical_system_ = canonical_system_;

        indices_ = nc2010dmp.indices_;
        initialized_ = nc2010dmp.initialized_;
        return *this;
    }

    bool NC2010DynamicMovementPrimitive::initialize(NC2010DMPParamPtr& parameters,
                                                    NC2010DMPStatePtr& state,
                                                    vector<NC2010TSPtr>& transformation_systems,
                                                    NC2010CSPtr& canonical_system)
    {
        // first set all member variables 
        assert(Utilities<NC2010DMPParam>::assign(parameters_, parameters));
        assert(Utilities<NC2010DMPState>::assign(state_, state));
        assert(Utilities<NC2010TS>::assign(transformation_systems_, transformation_systems));
        assert(Utilities<NC2010CS>::assign(canonical_system_, canonical_system));

        // then initialize base class variables 
        vector<TSBasePtr> base_transformation_systems;
        for (vector<NC2010TSPtr>::const_iterator vi = transformation_systems_.begin(); vi != transformation_systems_.end(); ++vi)
        {
            base_transformation_systems.push_back(*vi);
        }
        return DynamicMovementPrimitiveBase::initialize(parameters_, state_, base_transformation_systems, canonical_system_);
    }

    bool NC2010DynamicMovementPrimitive::add(const NC2010DynamicMovementPrimitive& nc2010dmp, bool check_for_compatibility)
    {
        if (check_for_compatibility && !isCompatible(nc2010dmp))
        {
            return false;
        }
        transformation_systems_.insert(transformation_systems_.end(), nc2010dmp.transformation_systems_.begin(), nc2010dmp.transformation_systems_.end());
        return parameters_->changeType(*nc2010dmp.parameters_);
    }

    bool NC2010DynamicMovementPrimitive::get(NC2010DMPParamConstPtr& parameters,
                                             NC2010DMPStateConstPtr& state,
                                             vector<NC2010TSConstPtr>& transformation_systems,
                                             NC2010CSConstPtr& canonical_system) const 
    {
        if (!initialized_)
        {
            Logger::logPrintf("NC2010 DMP is not initialized, cannot return parameters.", Logger::ERROR);
            return false;
        }
        parameters = parameters_;
        state = state_;
        transformation_systems.clear();
        for (int i = 0; i < (int)transformation_systems_.size(); ++i)
        {
            transformation_systems.push_back(transformation_systems_[i]);
        }
        canonical_system = canonical_system_;
        return true;
    }

    bool NC2010DynamicMovementPrimitive::initialize(const vector<string>& variable_names, lwr_lib::LWRParamPtr lwr_parameters, const double k_gain, const double d_gain)
    {
        (!variable_names.empty());
        (lwr_parameters->isInitialized());

        vector<TSBasePtr> nc2010_transformation_systems;
        for (int i = 0; i < (int)variable_names.size(); ++i)
        {
            NC2010TSParamPtr nc2010_transformation_system_parameters(new NC2010TransformationSystemParameters());

            lwr_lib::LWRPtr lwr_model(new::lwr_lib::LWR());
            if (!lwr_model->initialize(lwr_parameters))
            {
                Logger::logPrintf("Could not initialize LWR model with provided parameters.", Logger::ERROR);
                return false;
            }

            if (!nc2010_transformation_system_parameters->initialize(lwr_model, variable_names[i], k_gain, d_gain))
            {
                Logger::logPrintf("Could not initialize transformation system parameters.", Logger::ERROR);
                return false;
            }

            NC2010TSStatePtr nc2010_transformation_system_state(new NC2010TransformationSystemState());
            NC2010TSPtr nc2010_transformation_system(new NC2010TransformationSystem());
            if (!nc2010_transformation_system->initialize(nc2010_transformation_system_parameters,
                                                          nc2010_transformation_system_state,
                                                          TransformationSystemBase::NORMAL))
            {
                Logger::logPrintf("Could not initialize transformation system.", Logger::ERROR);
                return false;
            }
            nc2010_transformation_systems.push_back(nc2010_transformation_system);
        }

        NC2010CSParamPtr nc2010_canonical_system_parameters(new NC2010CanonicalSystemParameters());
        NC2010CSStatePtr nc2010_canonical_system_state(new NC2010CanonicalSystemState());
        NC2010CSPtr nc2010_canonical_system(new NC2010CanonicalSystem());
        if (!nc2010_canonical_system->initialize(nc2010_canonical_system_parameters, nc2010_canonical_system_state))
        {
            Logger::logPrintf("Could not initialize canonical system.", Logger::ERROR);
            return false;
        }

        DMPParamBasePtr dmp_parameters(new DynamicMovementPrimitiveParametersBase());
        if (!dmp_parameters->setCutoff(0.001))
        {
            Logger::logPrintf("Could not set cutoff.", Logger::ERROR);
            return false;
        }

        if (!DynamicMovementPrimitiveBase::initialize(dmp_parameters, nc2010_transformation_systems, nc2010_canonical_system))
        {
            Logger::logPrintf("Could not initialize dmp.", Logger::ERROR);
            return false;
        }
        return (initialized_ = true);
    }
}

