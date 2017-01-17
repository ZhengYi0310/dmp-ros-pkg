/*************************************************************************
	> File Name: transformation_system_state_base.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 26 Oct 2016 02:21:44 PM PDT
 ************************************************************************/

#ifndef _TRANSFOMRATION_SYSTEM_STATE_BASE_H
#define _TRANSFOMRATION_SYSTEM_STATE_BASE_H

// system includes
#include <vector>
#include <boost/shared_ptr.hpp>

// local includes 
#include "dmp_lib/state.h"

namespace dmp_lib
{
    
    /*!
     */
    class TransformationSystemStateBase
    {
        /*! Allow the TranformationSystemBase and DynamicMovementPrimitiveBase class to access private member variables directly
         */
        friend class TransformationSystemBase;
        friend class DynamicMovementPrimitiveBase;

        public:

            /*! Constructor
             */
            TransformationSystemStateBase() : start_(0), goal_(0), f_(0), ft_(0) {};

            /*! Destructor
             */
            virtual ~TransformationSystemStateBase() {};
            
            /*!
             * @param params 
             * @return true if equal, otherwise false
             */
            bool operator==(const TransformationSystemStateBase& state_base) const
            {
                return ( (internal_ == state_base.internal_)
                        && (target_ == state_base.target_)
                        && (current_ == state_base.current_)
                        && (fabs(start_ - state_base.start_) < EQUALITY_PRECISSION)
                        && (fabs(goal_ - state_base.goal_) < EQUALITY_PRECISSION)
                        && (fabs(f_ - state_base.f_) < EQUALITY_PRECISSION)
                        && (fabs(ft_ - state_base.ft_) < EQUALITY_PRECISSION)
                        && (function_input_ == state_base.function_input_)
                        && (function_target_ == state_base.function_target_) );
            }
            bool operator!=(const TransformationSystemStateBase& state_base) const
            {
                return !(*this == state_base);
            }

            /*!
             * @param internal 
             * @param target 
             * @param current 
             * @param start 
             * @param goal 
             * @param f 
             * @param ft 
             * @param function_input
             * @param function_target 
             * @return 
             */
            bool set(const State& internal,
                     const State& target,
                     const State& current,
                     const double start,
                     const double goal,
                     const double f,
                     const double ft/*,
                     const std::vector<double>& function_input,
                     const std::vector<double>& function_target*/);

            /*!
             * @param internal
             * @param target
             * @param current
             * @param start
             * @param goal
             * @param f
             * @param ft
             * @param function_input
             * @param function_target
             * @return
             */
            bool get(State& internal,
                     State& target,
                     State& current,
                     double& start,
                     double& goal,
                     double& f,
                     double& ft/*,
                     std::vector<double>& function_input,
                     std::vector<double>& function_target*/) const;
             
            /*!
             */
            void reset();

            /*!
             * @param internal_state
             */
            void setInternalState(const State& internal_state);

            /*!
             * @return
             */
            State getInternalState() const;

            /*!
             * @return
             */
            double getInternalStateX() const;

            /*!
             * @return
             */
            double getInternalStateXd() const;
            double getInternalStateXdd() const;

            /*!
             * @param target_state
             */
            void setTargetState(const State& target_state);

            /*!
             * @return
             */
            State getTargetState() const;

            /*!
             * @return
             */
            double getTargetStateX() const;

            /*!
             * @return
             */
            double getTargetStateXd() const;
            double getTargetStateXdd() const;

            /*!
             * @param current_state
             */
            void setCurrentState(const State& current_state);

            /*!
             * @return
             */
            State getCurrentState() const;

            /*!
             * @return
             */
            double getCurrentStateX() const;

            /*!
             * @return
             */
            double getCurrentStateXd() const;
            double getCurrentStateXdd() const;

            /*!
             */
            void setStart(const double start);

            /*!
             */
            double getStart() const;

            /*!
             */
            void setGoal(const double goal);

            /*!
             */
            double getGoal() const;

            /*!
             * @param ft
             */
            void setFT(const double ft);

            /*!
             * @return
             */
            double getFT() const;

            /*!
             * @param f
             */
            void setF(const double f);

            /*!
             * @return
             */
            double getF() const;

        protected:
            static const double EQUALITY_PRECISSION = 1e-6;

            /*!
             */
            State internal_;

            /*!
             */
            State target_;

            /*!
             */
            State current_;

            /*!
             */
            double start_;
            double goal_;

            /*!
             */
            double f_;
            double ft_;

            /*!
             */
            std::vector<double> function_input_;
            std::vector<double> function_target_;

        private:
    };

    /*! Abbreviation for convinience
     */
    typedef TransformationSystemStateBase TSStateBase;
    typedef boost::shared_ptr<TSStateBase> TSStateBasePtr;
    typedef boost::shared_ptr<TSStateBase const> TSStateBaseConstPtr;

    // inline functions
    inline void TransformationSystemStateBase::setInternalState(const State& internal_state)
    {
        internal_ = internal_state;
    }
    inline State TransformationSystemStateBase::getInternalState() const
    {
        return internal_;
    }
    inline double TransformationSystemStateBase::getInternalStateX() const
    {
        return internal_.getX();
    }
    inline double TransformationSystemStateBase::getInternalStateXd() const
    {
        return internal_.getXd();
    }
    inline double TransformationSystemStateBase::getInternalStateXdd() const
    {
        return internal_.getXdd();
    }
    inline void TransformationSystemStateBase::setTargetState(const State& target_state)
    {
        target_ = target_state;
    }
    inline State TransformationSystemStateBase::getTargetState() const
    {
        return target_;
    }
    inline double TransformationSystemStateBase::getTargetStateX() const
    {
        return target_.getX();
    }
    inline double TransformationSystemStateBase::getTargetStateXd() const
    {
        return target_.getXd();
    }
    inline double TransformationSystemStateBase::getTargetStateXdd() const
    {
        return target_.getXdd();
    }
    inline void TransformationSystemStateBase::setCurrentState(const State& current_state)
    {
        current_ = current_state;
    }
    inline State TransformationSystemStateBase::getCurrentState() const
    {
        return current_;
    }
    inline double TransformationSystemStateBase::getCurrentStateX() const
    {
        return current_.getX();
    }
    inline double TransformationSystemStateBase::getCurrentStateXd() const
    {
        return current_.getXd();
    }
    inline double TransformationSystemStateBase::getCurrentStateXdd() const
    {
        return current_.getXdd();
    }
    inline void TransformationSystemStateBase::setStart(const double start)
    {
        start_ = start;
    }
    inline double TransformationSystemStateBase::getStart() const
    {
        return start_;
    }
    inline void TransformationSystemStateBase::setGoal(const double goal)
    {
        goal_ = goal;
    }
    inline double TransformationSystemStateBase::getGoal() const
    {
        return goal_;
    }
    inline void TransformationSystemStateBase::setFT(const double ft)
    {
        ft_ = ft;
    }
    inline double TransformationSystemStateBase::getFT() const
    {
        return ft_;
    }
    inline void TransformationSystemStateBase::setF(const double f)
    {
        f_ = f;
    }
    inline double TransformationSystemStateBase::getF() const
    {
        return f_;
    }
}
#endif /* _TRANSFOMRATION_SYSTEM_STATE_BASE_H */
