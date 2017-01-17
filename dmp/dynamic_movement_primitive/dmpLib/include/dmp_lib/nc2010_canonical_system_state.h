/*************************************************************************
	> File Name: nc2010_canonical_system_state.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 31 Oct 2016 03:51:32 PM PDT
 ************************************************************************/

#ifndef _NC2010_CANONICAL_SYSTEM_STATE_H
#define _NC2010_CANONICAL_SYSTEM_STATE_H

// system includes

// local includes
#include "dmp_lib/canonical_system_state_base.h"

namespace dmp_lib
{
    class NC2010CanonicalSystemState : public CanonicalSystemStateBase
    {
        /*! Allow the CanonicalSystem class and TransformationSystem class to access private member variables directly
         */
        friend class NC2010TransformationSystem;
        friend class NC2010CanonicalSystem;

        public:
            /*! Constructor
             */
            NC2010CanonicalSystemState() {};

            /*! Destructor
             */
            virtual ~NC2010CanonicalSystemState() {};

        private:
    };

    /*! Abbreviation for convinience
     */
    typedef NC2010CanonicalSystemState NC2010CSState;
    typedef boost::shared_ptr<NC2010CSState> NC2010CSStatePtr;
    typedef boost::shared_ptr<NC2010CSState const> NC2010CSStateConstPtr;
}
#endif /* _NC2010_CANONICAL_SYSTEM_STATE_H */ 
