/*************************************************************************
	> File Name: nc2010_transformation_system_state.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 31 Oct 2016 04:27:48 PM PDT
 ************************************************************************/

#ifndef _NC2010_TRANSFORMATION_SYSTEM_STATE_H
#define _NC2010_TRANSFORMATION_SYSTEM_STATE_H

// system includes

// local includes
#include "dmp_lib/transformation_system_state_base.h"

namespace dmp_lib
{ 
    class NC2010TransformationSystemState : public TransformationSystemStateBase
    {

        /*! Allow the NC2010TransformationSystem class to access private member variables directly
         */
        friend class NC2010TransformationSystem;

        public:
            
            /*! Constructor
             */
            NC2010TransformationSystemState() {};

            /*! Destructor
             */
            virtual ~NC2010TransformationSystemState() {};

        private:
    };

    /*! Abbreviation for convinience
     */
    typedef NC2010TransformationSystemState NC2010TSState;
    typedef boost::shared_ptr<NC2010TSState> NC2010TSStatePtr;
    typedef boost::shared_ptr<NC2010TSState const> NC2010TSStateConstPtr;
}
#endif /* _NC2010_TRANSFORMATION_SYSTEM_STATE_H */
