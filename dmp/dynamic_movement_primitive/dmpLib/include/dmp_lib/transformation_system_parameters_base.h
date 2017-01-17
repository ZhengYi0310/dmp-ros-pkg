/*************************************************************************
	> File Name: transformation_system_parameters_base.h
	> Author: 
	> Mail: 
	> Created Time: Wed 26 Oct 2016 04:18:50 PM PDT
 ************************************************************************/

#ifndef _TRANSFORMATION_SYSTEM_PARAMETERS_BASE_H
#define _TRANSFORMATION_SYSTEM_PARAMETERS_BASE_H

// system includes 
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <lwr_lib/lwr.h>

// local includes 
#include "dmp_lib/status.h"

namespace dmp_lib
{

    /*! This class contains all variables that need to be saved to file in order to store a dmp to file
     */
    class TransformationSystemParametersBase : public Status
    {

        /*! Allow the TransformationSystem class to access private member variables directly
         */
        friend class TransformationSystemBase;
        friend class DynamicMovementPrimitiveBase;

        public:

            /*! Constructor
             */
            TransformationSystemParametersBase() : initial_start_(0.0), initial_goal_(0.0), name_("unnamed") {};

            /*! Destructor
             */
            virtual ~TransformationSystemParametersBase() {};

            /*!
             * @param params
             * @return True if equal, otherwise False
             */
            bool operator==(const TransformationSystemParametersBase &params_base) const
            {
                return ((isInitialized() && params_base.isInitialized())
                                         && (*lwr_model_ == *(params_base.lwr_model_))
                                         && (fabs(initial_start_ - params_base.initial_start_) < EQUALITY_PRECISSION)
                                         && (fabs(initial_goal_ - params_base.initial_goal_) < EQUALITY_PRECISSION)
                                         && (name_.compare(params_base.name_) == 0) );
            }

            bool operator!=(const TransformationSystemParametersBase &params_base) const
            {
                return !(*this == params_base);
            }

            /*! Assignment operator
             */
            TransformationSystemParametersBase& operator=(const TransformationSystemParametersBase& parameters);

            /*!
             * @param lwr_model
             * @param name
             * @param initial_start
             * @param initial_goal
             * @return True if success, otherwise False
             */
            bool initialize(const lwr_lib::LWRPtr lwr_model,
                            const std::string& name,
                            const double initial_start = 0,
                            const double initial_goal = 0);

            /*!
             * @param lwr_model
             * @param name
             * @param initial_start
             * @param initial_goal
             * @return True if success, otherwise False
             */
            bool get(lwr_lib::LWRConstPtr& lwr_model, std::string& name, double& initial_start, double& initial_goal) const;

            /*!
             * @return
             */
            double getInitialStart() const;

            /*!
             * @param initial_start
             */
            void setInitialStart(const double initial_start);

            /*!
             * @return
             */
            double getInitialGoal() const;

            /*!
             * @param initial_goal
             */
            void setInitialGoal(const double initial_goal);

            /*!
             * @return
             */
            std::string getName() const;

            /*!
             * @param name
             */
            void setName(const std::string& name);

            /*!
             * @return
             */
            const lwr_lib::LWRPtr getLWRModel() const
            {
                return lwr_model_;
            }

        protected:

            static const double EQUALITY_PRECISSION = 1e-6;

            /*!
             */
            lwr_lib::LWRPtr lwr_model_;

            /*!
             */
            double initial_start_;
            double initial_goal_;

            /*!
             */
            std::string name_;

        private:
    };

    /*! Abbreviation for convinience
     */
    typedef TransformationSystemParametersBase TSParamBase;
    typedef boost::shared_ptr<TSParamBase> TSParamBasePtr;
    typedef boost::shared_ptr<TSParamBase const> TSParamBaseConstPtr;
}
#endif /* _TRANSFORMATION_SYSTEM_PARAMETERS_BASE_H */
