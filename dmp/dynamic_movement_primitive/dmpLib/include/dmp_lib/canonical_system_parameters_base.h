/*************************************************************************
	> File Name: canonical_system_parameters_base.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 26 Oct 2016 10:10:27 AM PDT
 ************************************************************************/

#ifndef _CANONICAL_SYSTEM_PARAMETERS_BASE_H
#define _CANONICAL_SYSTEM_PARAMETERS_BASE_H

// system includes
#include <boost/shared_ptr.hpp>
#include <math.h>

// local includes
#include "dmp_lib/status.h"

namespace dmp_lib
{
    /*!
     */
    class CanonicalSystemParametersBase : public Status 
    {
        /*! Allow the CanonicalSystemBase class to access private memeber variables directly
         */
        friend class CanonicalSystemBase;

        public:
            
            /*! Constructor
             */
            CanonicalSystemParametersBase() : alpha_x_(0) {};

            /*! Destructor
             */
            virtual ~CanonicalSystemParametersBase() {};

            /*!
             * @param params 
             * @return True if equal, otherwise False
             */
            bool operator==(const CanonicalSystemParametersBase &params_base) const 
            {
                return ((isInitialized() && params_base.isInitialized()) && (fabs(alpha_x_ - params_base.alpha_x_) < EQUALITY_PRECISION));
            }

            bool operator!=(const CanonicalSystemParametersBase &params_base) const 
            {
                return !(*this == params_base);
            }

            /*!
             * @param alpha_x
             * @return True on success, otherwise False 
             */
            bool initialize(const double alpha_x);

            /*!
             * @param other_params_base
             * @return 
             */
            bool isCompatible(const CanonicalSystemParametersBase &other_params_base) const;

            /*!
             * @params alpha_x
             * @return True on success, otherwise False 
             */
            bool get(double &alpha_x) const;

            /*! Compute alpha_x such that canonical system drops below the cutoff when the trajectory has finished
             * @param cutoff
             * @return True on success, otherwise False 
             */
            bool setCutoff(const double cutoff);

            /*!
             * @return 
             */
            double getAlphaX() const;

        protected:
            static const double EQUALITY_PRECISION = 1e-6;

            /*! Time constant 
             */
            double alpha_x_;
    };

    /*! Abbreviation for convinence 
     */
    typedef CanonicalSystemParametersBase CSParamBase;
    typedef boost::shared_ptr<CSParamBase> CSParamBasePtr;
    typedef boost::shared_ptr<CSParamBase> CSParamBaseConstPtr;

    // Initialize 
    inline double CanonicalSystemParametersBase::getAlphaX() const 
    {
        assert(initialized_);
        return alpha_x_;
    }

    inline bool CanonicalSystemParametersBase::get(double &alpha_x) const 
    {
        assert(initialized_);
        alpha_x = alpha_x_;
        return true;
    }
}
#endif /* _CANONICAL_SYSTEM_PARAMETERS_BASE_H */
