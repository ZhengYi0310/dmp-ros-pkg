/*************************************************************************
	> File Name: nc2010_canonical_system_parameters.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 31 Oct 2016 03:42:43 PM PDT
 ************************************************************************/

#ifndef _NC2010_CANONICAL_SYSTEM_PARAMETERS_H
#define _NC2010_CANONICAL_SYSTEM_PARAMETERS_H

// system includes

// local includes 
#include "dmp_lib/canonical_system_parameters_base.h"

namespace dmp_lib
{
    /*! This class implements the parameters for a simple first order dynamical system
     */
    class NC2010CanonicalSystemParameters : public CanonicalSystemParametersBase
    {
        /*! Allow the CanonicalSystem class to access private member variables directly
         */
        friend class CanonicalSystemBase;

        public:
            /*! Constructor
             */
            NC2010CanonicalSystemParameters() {};

            /*! Destructor
             */
            virtual ~NC2010CanonicalSystemParameters() {};
        private:    
    };

    /*! Abbreviation for convinience
     */
    typedef NC2010CanonicalSystemParameters NC2010CSParam;
    typedef boost::shared_ptr<NC2010CSParam> NC2010CSParamPtr;
    typedef boost::shared_ptr<NC2010CSParam const> NC2010CSParamConstPtr;
}

#endif /* _NC2010_CANONICAL_SYSTEM_PARAMETERS_H */
