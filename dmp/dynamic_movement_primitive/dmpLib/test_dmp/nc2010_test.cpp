/*************************************************************************
	> File Name: nc2010_test.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 03 Nov 2016 11:48:48 AM PDT
 ************************************************************************/

// system includes 
#include <string>
#include <vector>
#include <stdio.h>

// local includes 
#include <dmp_lib/nc2010_dynamic_movement_primitive.h>
#include <dmp_lib/logger.h>
#include "nc2010_test.h"

using namespace dmp_lib;
using namespace std;

// import most common eigen types 
using namespace Eigen;

namespace test_dmp
{
    bool NC2010Test::initialize(NC2010DMP& dmp, const TestData& testdata)
    {

        lwr_lib::LWRParamPtr lwr_parameters(new lwr_lib::LWRParameters());
        if (!lwr_parameters->initialize(testdata.getNumRFS(), testdata.getActivition()))
        {
            dmp_lib::Logger::logPrintf("Could not initialize LWR parameters.", dmp_lib::Logger::ERROR);
            return false;
        }

        lwr_lib::LWRPtr lwr_model(new lwr_lib::LWR());
        if (!lwr_model->initialize(lwr_parameters))
        {
            dmp_lib::Logger::logPrintf("Could not initialize LWR model with provided parameters.", Logger::ERROR);
            return false;
        }

        vector<NC2010TSPtr> nc2010_transformation_systems;

        vector<string> variable_names = testdata.getTestVariableNames();
        for (int i = 0; i < (int)variable_names.size(); ++i)
        {
            vector<NC2010TSStatePtr> nc2010_transformation_system_state_vector;
            vector<NC2010TSParamPtr> nc2010_transformation_system_parameters_vector;

            NC2010TSStatePtr nc2010_transformation_system_state;
            NC2010TSParamPtr nc2010_transformation_system_parameters;

            if (testdata.getTestCase() == TestData::QUAT_TEST && testdata.getQuatTSIndex() == i)
            {
                if (i+4 > (int)variable_names.size())
                {
                    Logger::logPrintf("Number of variable names is in valid.", Logger::ERROR);
                    return false;
                }
                for (int j = 0; j < 4; j++)
                {
                    nc2010_transformation_system_parameters.reset(new NC2010TransformationSystemParameters());
                    if (!nc2010_transformation_system_parameters->initialize(lwr_model, variable_names[i+j], testdata.getKGain(), testdata.getDGain()))
                    {
                        Logger::logPrintf("Could not initialize transformation system parameters.", Logger::ERROR);
                        return false;
                    }
                    nc2010_transformation_system_parameters_vector.push_back(nc2010_transformation_system_parameters);
                    nc2010_transformation_system_state.reset(new NC2010TransformationSystemState());
                    nc2010_transformation_system_state_vector.push_back(nc2010_transformation_system_state);
                }
                i += 4;
            }
            
            else
            {
                nc2010_transformation_system_parameters.reset(new NC2010TransformationSystemParameters());
                if (!nc2010_transformation_system_parameters->initialize(lwr_model, variable_names[i], testdata.getKGain(), testdata.getDGain()))
                {
                    Logger::logPrintf("Could not initialize transfomration system parameters.", Logger::ERROR);
                    return false;
                }
                nc2010_transformation_system_parameters_vector.push_back(nc2010_transformation_system_parameters);
                nc2010_transformation_system_state.reset(new NC2010TransformationSystemState());
                nc2010_transformation_system_state_vector.push_back(nc2010_transformation_system_state);
            }

            // TODO: fix the integration method 
            NC2010TSPtr nc2010_transformation_system(new NC2010TransformationSystem());
            if (!nc2010_transformation_system->initialize(nc2010_transformation_system_parameters_vector,
                                                          nc2010_transformation_system_state_vector,
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

        NC2010DMPParamPtr nc2010_dmp_parameters(new NC2010DynamicMovementPrimitiveParameters());
        if (!nc2010_dmp_parameters->setCutoff(testdata.getCutoff()))
        {
            Logger::logPrintf("Could not set cutoff.", Logger::ERROR);
            return false;
        }

        NC2010DMPStatePtr nc2010_dmp_state(new NC2010DynamicMovementPrimitiveState());
        if (!dmp.initialize(nc2010_dmp_parameters, nc2010_dmp_state, nc2010_transformation_systems, nc2010_canonical_system))
        {
            Logger::logPrintf("Could not initialize nc2010 dmp.", Logger::ERROR);
            return false;
        }

        if (testdata.getTestCase() == TestData::QUAT_TEST)
        {
            dmp.getTransformationSystem(testdata.getQuatTSIndex())->setIntegrationMethod(TransformationSystemBase::QUATERNION);
        }
        return true;
    }
}
