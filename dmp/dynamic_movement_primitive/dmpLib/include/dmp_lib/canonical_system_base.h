/*************************************************************************
	> File Name: canonical_system_base.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 26 Oct 2016 11:34:54 AM PDT
 ************************************************************************/

#ifndef _CANONICAL_SYSTEM_BASE_H
#define _CANONICAL_SYSTEM_BASE_H

// system includes
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Eigen>

// local includes 
#include "dmp_lib/canonical_system_parameters_base.h"
#include "dmp_lib/canonical_system_state_base.h"
#include "dmp_lib/time.h"
#include "dmp_lib/status.h"

namespace dmp_lib
{
    /*!
     */
    class CanonicalSystemBase : public Status 
    {
        /*! Allow the DynamicMovementPrimitiveBase class to access private member variables directly
         */
        friend class DynamicMovementPrimitiveBase;
        
        public:

            /*! Constructor
             */
            CanonicalSystemBase() {};

            /*! Destructor
             */ 
            virtual ~CanonicalSystemBase() {};

            /*!
             * @param canonical_system_base
             * @return true if equal, otherwise false 
             */
            bool operator==(const CanonicalSystemBase &canonical_system_base) const 
            {
                return ((isInitialized() && canonical_system_base.isInitialized()) && (*parameters_ == *(canonical_system_base.parameters_)) && (*state_ == *(canonical_system_base.state_)));
            }

            bool operator!=(const CanonicalSystemBase &canonical_system_base) const 
            {
                return !(*this == canonical_system_base);
            }

            /*!
             * @param params
             * @param state 
             * @return true on success, otherwise false 
             */
            bool initialize(const CSParamBasePtr params, const CSStateBasePtr state);

            /*!
             * @param other_cs_base
             * @return True on success, otherwise False
             */
            bool isCompatible(const CanonicalSystemBase& other_cs_base) const;

            /*!
             * @param parameters
             * @param state
             * @return True on success, otherwise False
             */
            bool get(CSParamBaseConstPtr& parameters, CSStateBaseConstPtr& state) const;

            /*! Reset the canonical system 
             */
            virtual void reset() = 0;

            /*! Integrate the canonical system 
             * @param dmp_time 
             * @return true on success, otherwise false 
             * REAL-TIME REQUIREMENTS
             */
            virtual bool integrate(const Time& dmp_time) = 0;

            /*! Returns the time (in sec) of the movement.
             * Will stop when the movement duration is reached
             * @return
             * REAL-TIME REQUIREMENTS
             */
            virtual double getTime() const = 0;

            /*! Updates the time (in sec) of the movement.
             * Will NOT stop when the movement duration is reached
             * @param dmp_time
             * REAL-TIME REQUIREMENTS
             */
            void integrateProgress(const Time& dmp_time)
            {
                state_->addProgressTime(dmp_time.getDeltaT());
            }
    

            /*! Returns the time (in sec) of the movement 
             * Will NOT stop when the movement duration is reached 
             * @return 
             * REAL-TIME REQUIREMENT 
             */
            double getProgressTime() const 
            {
                return state_->getProgressTime();
            }

            /*! Sets the rollout with num_time_steps time steps 
             * @param num_time_steps
             * @param cutoff 
             * @return true on success, otherwise false 
             */
            virtual bool getRollout(const int num_time_steps, const double cutoff, Eigen::VectorXd& rollout) const = 0;

            /*!
             * @return 
             */
            CSParamBasePtr getParameters() const;

            /*!
             * @return 
             */
            CSStateBasePtr getState() const;

        protected:

            /*!
             */
            CSParamBasePtr parameters_;

            /*!
             */
            CSStateBasePtr state_;
    };

    /*! Abbreviation for convenience
     */
    typedef boost::shared_ptr<CanonicalSystemBase> CSBasePtr;
    typedef boost::shared_ptr<CanonicalSystemBase const> CSBaseConstPtr;

    // inline function follows 
    inline CSParamBasePtr CanonicalSystemBase::getParameters() const 
    {
        assert(initialized_);
        return parameters_;
    }

    inline CSStateBasePtr CanonicalSystemBase::getState() const 
    {
        assert(initialized_);
        return state_;
    }
}
#endif /* _CANONICAL_SYSTEM_BASE_H */
