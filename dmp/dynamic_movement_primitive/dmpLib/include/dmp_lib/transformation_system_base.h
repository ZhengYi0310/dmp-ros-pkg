/*************************************************************************
	> File Name: transformation_system_base.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 27 Oct 2016 10:52:50 AM PDT
 ************************************************************************/

#ifndef _TRANSFORMATION_SYSTEM_BASE_H
#define _TRANSFORMATION_SYSTEM_BASE_H

// system includes 
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

// local includes 
#include "dmp_lib/transformation_system_parameters_base.h"
#include "dmp_lib/transformation_system_state_base.h"
#include "dmp_lib/canonical_system_state_base.h"
#include "dmp_lib/canonical_system_parameters_base.h"
#include "dmp_lib/time.h"
#include "dmp_lib/state.h"
#include "dmp_lib/status.h"

namespace dmp_lib
{

    /*!
     */
    class TransformationSystemBase : public Status 
    {
        /*! Allow DynamicMovementPrimitive class to access private member variables directly
         */
        friend class DynamicMovementPrimitiveBase;
        
        public:
                
            /*! Constructor
             */
            TransformationSystemBase() : integration_method_(NORMAL) {};

            /*! Destructor
             */
            virtual ~TransformationSystemBase() {};

            /*! 
             * @param transformation_system 
             * @return true if equal, otherwise false 
             */
            bool operator==(const TransformationSystemBase& transformation_system_base) const 
            {
                if ((isInitialized() && transformation_system_base.isInitialized())
                   && (integration_method_ == transformation_system_base.integration_method_)
                   && (parameters_.size() == transformation_system_base.parameters_.size())
                   && (states_.size() == transformation_system_base.states_.size()))
                {
                    for (unsigned int i = 0; i < parameters_.size(); ++i)
                    {
                        if (*(parameters_[i]) != *(transformation_system_base.parameters_[i]))
                        {
                            return false;
                        }
                    }

                    for (unsigned int i = 0; i < states_.size(); ++i)
                    {
                        if (*(states_[i]) != *(transformation_system_base.states_[i]))
                        {
                            return false;
                        }
                    }
                    
                    return true;
                }

                return false;
            }

            bool operator!=(const TransformationSystemBase& transformation_system_base) const 
            {
                return !(*this == transformation_system_base);
            }

            /*!
             */
            enum IntegrationMethod
            {
                NORMAL, //!< NORMAL
                QUATERNION //!< QUATERNION
            };

            /*! Initializes the transformation system. The parameters have to be initialized.
             * @param parameters 
             * @param states 
             * @return true if success, otherwise false 
             */
            bool initialize(const std::vector<TSParamBasePtr> parameters,
                            const std::vector<TSStateBasePtr> states,
                            const IntegrationMethod integration_method);

            /*!
             * @param parameters 
             * @param states 
             * @return true if sucess, otherwise false 
             */
            bool get(std::vector<TSParamBaseConstPtr>& parameters,
                     std::vector<TSStateBaseConstPtr>& states,
                     IntegrationMethod& integration_method) const;

            /*! Reset the transfomration system 
             */
            virtual void reset() = 0;

            /*! This function takes the target states, the state of the canonical system, and the duration tau 
             * and fits the transfomration system
             *
             * @param target_states 
             * @param canonical_system_state 
             * @return true if success, otherwise false 
             */
            virtual bool integrateAndFit(const std::vector<State>& target_states,
                                         const CSStateBasePtr canonical_system_state,
                                         const Time& dmp_time) = 0;

            /*!
             * @param canonical_system_state
             * @param dmp_time
             * @param feedback
             * @param num_iterations
             * @return False if the lwr model could not come up with a prediction for various reasons, otherwise True.
             * REAL-TIME REQUIREMENTS
             */
            virtual bool integrate(const CSStateBasePtr canonical_system_state,
                                   const Time& dmp_time,
                                   const Eigen::VectorXd& feedback,
                                   const int num_iterations = 1) = 0;

            /*!
             * @param index 
             * @param current_state 
             * @return true if success, otherwise false 
             * REAL-TIME REQUIREMENTS
             */
            bool setCurrentState(const int index, const State& current_state);

            /*!
             * @param current_states
             * @return true if success, otherwise false 
             * REAL-TIME REQUIREMENTS
             */
            bool setCurrentStates(const std::vector<State>& current_states);

            /*!
             * @param index 
             * @param state 
             * @return true if success, otherwise false 
             * REAL-TIME REQUIREMENTS
             */
            bool getCurrentState(const int index, State& state) const;

            /*!
             * @param states 
             */
            void getCurrentStates(std::vector<State>& states) const;

            /*!
             * @param index 
             * @param start 
             * @param initial 
             * @return true if success, otherwise false 
             */
            bool setStart(const int index, const double start, bool initial = false);

            /*!
             * @param start
             * @param initial 
             * @return true if success, otherwise false 
             */
            bool setStart(const std::vector<double> start, bool initial = false);

            /*!
             * @param index
             * @param start 
             * @param initial 
             * @return true if success, otherwise false 
             */
            bool getStart(const int index, double& start, bool initial = false) const;

            /*!
             * @param start 
             * @param initial 
             */
            void getStart(std::vector<double>& start, bool initial = false) const;

            /*!
             * @param index
             * @param goal 
             * @param Initial 
             * @return true if success, otherwise false 
             */
            bool setGoal(const int index, const double goal, bool initial = false);

            /*!
             * @param goal 
             * @param initial 
             * @return true if success, otherwise false 
             */
            bool setGoal(const std::vector<double> goal, bool intial = false);

            /*!
             * @param index
             * @param goal
             * @param initial
             * @return True if success, otherwise False
             */
            bool getGoal(const int index, double& goal, bool initial = false) const;

            /*!
             * @param goal
             * @param initial
             * @return True if success, otherwise False
             */
            void getGoal(std::vector<double>& goal, bool initial = false) const;

            /*!
             * @param index
             * @param initial_start
             * @return True if success, otherwise False
             */
            bool setInitialStart(const int index, const double initial_start);

            /*!
             * @param initial_start
             * @return True if success, otherwise False
             */
            bool setInitialStart(const std::vector<double> initial_start);

            /*!
             * @param index
             * @param initial_start
             * @return True if success, otherwise False
             */
            bool getInitialStart(const int index, double& initial_start) const;

            /*!
             * @param initial_start
             */
            void getInitialStart(std::vector<double>& initial_start) const;

            /*!
             * @param index
             * @param initial_goal
             * @return True if success, otherwise False
             */
            bool setInitialGoal(const int index, const double initial_goal);

            /*!
             * @param initial_goal
             * @return True if success, otherwise False
             */
            bool setInitialGoal(const std::vector<double> initial_goal);

            /*!
             * @param index
             * @param initial_goal
             * @return True if success, otherwise False
             */
            bool getInitialGoal(const int index, double& initial_goal) const;

            /*!
             * @param initial_goal
             */
            void getInitialGoal(std::vector<double>& initial_goal) const;

            /*!
             * @return
             */
            int getNumDimensions() const;

            /*!
             * @param index
             * @param name
             * @return
             */
            bool getName(const int index, std::string& name) const;

            /*!
             * @param index
             * @return The name of the controlled variable, if index is invalid then "null" is returned
             * REAL-TIME REQUIREMENTS
             */
            const std::string& getName(const int index) const;

            /*!
             * @param names
             * @return
             */
            void getNames(std::vector<std::string>& names) const;

            /*!
             * @return
             */
            std::vector<TSParamBasePtr> getParameters() const;

            /*!
             * @return
             */
            std::vector<TSStateBasePtr> getStates() const;

            /*!
             * @return
             * REAL-TIME REQUIREMENTS
             */
            TSParamBasePtr getParameters(const int index) const;

            /*!
             * @return
             * REAL-TIME REQUIREMENTS
             */
            TSStateBasePtr getStates(const int index) const;

            /*! Sets the integration method. Default is NORMAL. Set to QUATERNION if transformation system encodes a quaternion.
             * @param integration_method
             * @return True on success, False otherwise
             */
            bool setIntegrationMethod(IntegrationMethod integration_method);

            /*! Gets the integration method used by the transformation system
             * @return
             */
            const IntegrationMethod& getIntegrationMethod() const
            {
                return integration_method_;
            }

        protected:
            
            /*!
             */
            std::vector<TSParamBasePtr> parameters_;

            /*!
             */
            std::vector<TSStateBasePtr> states_;

            /*!
             */
            IntegrationMethod integration_method_;
    };

    /*! Abbreviation for convinience
     */
    typedef boost::shared_ptr<TransformationSystemBase> TSBasePtr;
    typedef boost::shared_ptr<TransformationSystemBase const> TSBaseConstPtr;

    // Inline definitions follow
    // REAL-TIME REQUIREMENT
    // TODO: Think about this again... number of dimension changes if transformation system encodes a quaternion, does it ?
    inline int TransformationSystemBase::getNumDimensions() const
    {
        assert(initialized_);
        assert(parameters_.size() == states_.size());
        return static_cast<int>(parameters_.size());
    }

    // REAL-TIME REQUIREMENT
    inline std::vector<TSParamBasePtr> TransformationSystemBase::getParameters() const
    {
        assert(initialized_);
        return parameters_;
    }

    // REAL-TIME REQUIREMENT
    inline std::vector<TSStateBasePtr> TransformationSystemBase::getStates() const
    {
        assert(initialized_);
        return states_;
    }

    // REAL-TIME REQUIREMENT
    inline TSParamBasePtr TransformationSystemBase::getParameters(const int index) const
    {
        assert(index >= 0);
        assert(index < getNumDimensions());
        return parameters_[index];
    }

    // REAL-TIME REQUIREMENT
    inline TSStateBasePtr TransformationSystemBase::getStates(const int index) const
    {
        assert(index >= 0);
        assert(index < getNumDimensions());
        return states_[index];
    }


}
#endif /* _TRANSFORMATION_SYSTEM_BASE_H */
