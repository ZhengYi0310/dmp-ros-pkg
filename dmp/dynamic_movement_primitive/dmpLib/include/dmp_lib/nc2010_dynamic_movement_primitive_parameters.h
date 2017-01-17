/*************************************************************************
	> File Name: nc2010_dynamic_movement_primitive_parameters.h
	> Author: Yi Zheng
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 01 Nov 2016 10:44:49 AM PDT
 ************************************************************************/

#ifndef _NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_PARAMETERS_H
#define _NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_PARAMETERS_H

// system includes

// local includes
#include "dmp_lib/dynamic_movement_primitive_parameters_base.h"

namespace dmp_lib
{
    class NC2010DynamicMovementPrimitiveParameters : public DynamicMovementPrimitiveParametersBase
    {
        public:
            
            /*! Constructor
             */
            NC2010DynamicMovementPrimitiveParameters() {};

            /*! Destructor
             */
            virtual ~NC2010DynamicMovementPrimitiveParameters() {};

        private:
    };

    /*! Abbreviation for convinience
     */
    typedef NC2010DynamicMovementPrimitiveParameters NC2010DMPParam;
    typedef boost::shared_ptr<NC2010DMPParam> NC2010DMPParamPtr;
    typedef boost::shared_ptr<NC2010DMPParam const> NC2010DMPParamConstPtr;
}
#endif
