/*************************************************************************
	> File Name: canonical_system_state_base.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 18 Oct 2016 03:00:38 PM PDT
 ************************************************************************/

#ifndef _CANONICAL_SYSTEM_STATE_BASE_H
#define _CANONICAL_SYSTEM_STATE_BASE_H

// system inlcudes
#include <boost/shared_ptr.hpp>
#include <math.h>

// local includes
#include "dmp_lib/state.h"
namespace dmp_lib
{
    class CanonicalSystemStateBase
    {
        /*! Allow the CanonicalSystemBase class and TransformationSystemBase class to access private member variables directly
         */
        friend class CanonicalSystemBase;
        // friend class TransformationSystemBase;

        public:
            
            /*! Constructor
             */
            CanonicalSystemStateBase() : time_(0), progress_time_(0) {};

            /*! Destructor
             */
            virtual ~CanonicalSystemStateBase() {};

            /*! Only check whether the state is the same
             * @param state 
             * @return True if equal, otherwise False 
             */
            bool operator==(const CanonicalSystemStateBase &state_base) const 
            {
                return ((state_ == state_base.state_) && (fabs(time_ - state_base.time_) < EQUALITY_PRECISSION));
            }

            bool operator!=(const CanonicalSystemStateBase &state_base) const 
            {
                return !(*this == state_base);
            }

            /*!
             * @param other_state_base
             * @return
             */
            bool isCompatible(const CanonicalSystemStateBase& other_state_base) const;

            /*!
             * @param state 
             * @param time 
             */
            void set(const State& state, const double time);

            /*!
             * @param state 
             * @param time 
             */
            void get(State& state, double& time) const; 

            /*!
             * @return 
             */
            double getStateX() const;

            /*!
             * @return 
             */
            double getStateXd() const;

             /*!
              * @return
              */
            double getCanX() const;

            /*!
             * @return
             */
            void setCanX(const double x);

            /*!
             * @return
             */
            void setState(const State& state);

            /*!
             * @param x
             */
            void setStateX(const double x);

            /*!
             * @param xd
             */
            void setStateXd(const double xd);

            /*!
             * @return
             */
            double getTime() const;

            /*!
             * @param time 
             */
            void setTime(const double time);

            /*!
             * @param dt
             */
            void addTime(const double dt);

            /*!
             * @return
             */
            double getProgressTime() const;

            /*!
             * @return
             */
            void setProgressTime(const double time);

            /*!
             * @param dt
             */
            void addProgressTime(const double dt);

        protected:
            static const double EQUALITY_PRECISSION = 1e-6;

            /*!
             */
            State state_;
            double time_;
            double progress_time_;
    };

    /*! Abbreviation for convinience
     */
    typedef CanonicalSystemStateBase CSStateBase;
    typedef boost::shared_ptr<CSStateBase> CSStateBasePtr;
    typedef boost::shared_ptr<CSStateBase const> CSStateBaseConstPtr;

    inline void CanonicalSystemStateBase::set(const State& state, const double time)
    {
        state_ = state;
        time_ = time;
    }

    inline void CanonicalSystemStateBase::get(State& state, double& time) const
    {
        state = state_;
        time = time_;
    }


    inline bool CanonicalSystemStateBase::isCompatible(const CanonicalSystemStateBase& /*other_state*/) const
    {
        // for now canonical system states are always compatible.
        return true;
    }

    inline double CanonicalSystemStateBase::getStateX() const
    {
        return state_.getX();
    }

    inline double CanonicalSystemStateBase::getCanX() const
    {
        // small hacks here and there keep things funny (TODO: change this)
        return state_.getXdd();
    }
    
    inline double CanonicalSystemStateBase::getStateXd() const
    {
        return state_.getXd();
    }
    
    inline void CanonicalSystemStateBase::setState(const State& state)
    {
        state_ = state;
    }
    
    inline void CanonicalSystemStateBase::setStateX(const double x)
    {
        state_.setX(x);
    }

    inline void CanonicalSystemStateBase::setStateXd(const double xd)
    {
        state_.setX(xd);
    }

    inline void CanonicalSystemStateBase::setCanX(const double x)
    {
        // small hacks here and there keep things funny (TODO: change this)
        state_.setXdd(x);
    }

    inline double CanonicalSystemStateBase::getTime() const
    {
        return time_;
    }

    inline void CanonicalSystemStateBase::setTime(const double time)
    {
        time_ = time;
    }

    inline void CanonicalSystemStateBase::addTime(const double dt)
    {
        time_ += dt;
    }

    inline double CanonicalSystemStateBase::getProgressTime() const
    {
        return progress_time_;
    }

    inline void CanonicalSystemStateBase::setProgressTime(const double time)
    {
        progress_time_ = time;
    }

    inline void CanonicalSystemStateBase::addProgressTime(const double dt)
    {
        progress_time_ += dt;
    }
}
#endif
