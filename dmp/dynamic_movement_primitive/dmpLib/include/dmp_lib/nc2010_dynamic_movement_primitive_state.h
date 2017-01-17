/*************************************************************************
	> File Name: nc2010_dynamic_movement_primitive_state.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 01 Nov 2016 10:55:34 AM PDT
 ************************************************************************/

#ifndef _NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_STATE_H
#define _NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_STATE_H

// system includes

// local includes
#include <dmp_lib/dynamic_movement_primitive_state_base.h>

namespace dmp_lib
{
    class NC2010DynamicMovementPrimitiveState : public DynamicMovementPrimitiveStateBase
    {

        public:
            
            /*! Constructor
             */
            NC2010DynamicMovementPrimitiveState() {};

            /*! Destructor
             */
            virtual ~NC2010DynamicMovementPrimitiveState () {};

        private:
    };

    /*! Abbreviation for convinience
     */
    typedef NC2010DynamicMovementPrimitiveState NC2010DMPState;
    typedef boost::shared_ptr<NC2010DMPState> NC2010DMPStatePtr;
    typedef boost::shared_ptr<NC2010DMPState const> NC2010DMPStateConstPtr;
}
#endif
