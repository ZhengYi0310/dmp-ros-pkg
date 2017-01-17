/*************************************************************************
	> File Name: nc2010_canonical_system.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 31 Oct 2016 03:57:30 PM PDT
 ************************************************************************/

#ifndef _NC2010_CANONICAL_SYSTEM_H
#define _NC2010_CANONICAL_SYSTEM_H

// system include
#include <string>
#include <boost/shared_ptr.hpp>

// local include
#include "dmp_lib/canonical_system_base.h"
#include "dmp_lib/nc2010_canonical_system_parameters.h"
#include "dmp_lib/nc2010_canonical_system_state.h"

namespace dmp_lib
{
    /*!
     */
    class NC2010CanonicalSystem : public CanonicalSystemBase
    {

        public:
            friend class NC2010TransformationSystem;

            /*! Constructor
             */
            NC2010CanonicalSystem() {};

            /*! Destructor
             */
            virtual ~NC2010CanonicalSystem() {};

            /*! Assignment operator
             */
            NC2010CanonicalSystem& operator=(const NC2010CanonicalSystem& nc2010cs);

            /*! Initializes the canonical system from state and parameters
             * @param parameters
             * @param state
             * @return True on success, otherwise False
             */
            bool initialize(const NC2010CSParamPtr parameters, const NC2010CSStatePtr state);

            /*!
             * @param parameters
             * @param state
             * @return True on success, otherwise False
             */
            bool get(NC2010CSParamConstPtr& parameters, NC2010CSStateConstPtr& state) const;

            /*! Reset the canonical system
             */
            void reset();

            /*! Integrate the canonical system
             */
            bool integrate(const Time& dmp_time);

            /*!
             * @return
             */
            double getTime() const;

            /*!
             * @param num_time_steps
             * @param rollout
             * @return True on success, otherwise False
             */
            bool getRollout(const int num_time_steps, const double cutoff, Eigen::VectorXd& rollout) const;

            /*! Returns the version string
             * @return version string
             */
            std::string getVersionString() const
            {
                return "NC2010";
            }

        private:
            /*!
             */
            NC2010CSParamPtr parameters_;

            /*!
             */
            NC2010CSStatePtr state_;
    };

    /*! Abbreviation for convinience
     */
    typedef NC2010CanonicalSystem NC2010CS;
    typedef boost::shared_ptr<NC2010CS> NC2010CSPtr;
    typedef boost::shared_ptr<NC2010CS const> NC2010CSConstPtr;
}
#endif /* _NC2010_CANONICAL_SYSTEM_H */
