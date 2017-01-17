/*************************************************************************
	> File Name: time.h
	> Author: Yi Zheng
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 17 Oct 2016 10:09:12 PM PDT
 ************************************************************************/

#ifndef _TIME_BASE_H
#define _TIME_BASE_H

// system includes
#include <math.h>

// local includes
#include "dmp_lib/logger.h"

namespace dmp_lib
{
    class Time 
    {
        public:
            /*! Constructor
             */
            Time() : delta_t_(0), tau_(0) {};
            Time(const double delta_t, const double tau) : delta_t_(delta_t), tau_(tau) {};

            /*! Destructor
             */
            virtual ~Time() {};
            
            /*！
             * @param time
             * @return True if the Time is equal, otherwise False
             */
            inline bool operator==(const Time &time) const 
            {
                return (fabs(delta_t_ - time.delta_t_) < EQUALITY_PRECISSION && fabs(tau_ - time.tau_) < EQUALITY_PRECISSION);
            }

            inline bool operator!=(const Time &time) const
            {
                return !(*this == time);
            }

            /*!
             * @param tau
             * REAL-TIME REQUIREMENTS
             */
            bool setTau(const double tau);

            /*!
             * @return 
             * REAL-TIME REQUIREMENTS
             */
            double getTau() const;

            /*!
             * @param delta_t
             * REAL-TIME REQUIREMENTS
             */
            bool setDeltaT(const double delta_t);
            
            /*!
             * @return
             * @REAL-TIME REQUIREMENTS
             */
            double getDeltaT() const;
            
            /*!
             * @return 
             * REAL-TIME REQUIREMENTS
             */
            bool getNumberOfIntervalSteps(int& num_interval_steps) const;

            /*!
             * @return
             * REAL-TIME REQUIREMENTS
             */
            bool getSamplingFrequency(double& sampling_frenquency) const;

        private:
            static const double EQUALITY_PRECISSION = 1e-6;

            /*!
             */
            double delta_t_;

            /*!
             */
            double tau_;
    };

    // Inline functions follow
    inline bool Time::setTau(const double tau)
    {
        if (tau <= 0)
        {
            Logger::logPrintf("Tau >%f< is invalid (Real-time violation).", Logger::ERROR, tau);
            return false;
        }
        tau_ = tau;
        return true;
    }

    // REAL-TIME REQUIREMENTS
    inline double Time::getTau() const 
    {
        return tau_;
    }

    inline bool Time::setDeltaT(const double delta_t)
    {
        if (delta_t <= 0)
        {
            Logger::logPrintf("Delta >%f< is invalid (Real-time violation).", Logger::ERROR, delta_t);
            return false;
        }
        delta_t_ = delta_t;
        return true;
    }

    // REAL-TIME REQUIREMENTS
    inline double Time::getDeltaT() const 
    {
        return delta_t_;
    }

    // REAL-TIME REQUIREMENTS
    inline bool Time::getNumberOfIntervalSteps(int& num_interval_steps) const 
    {
        if (delta_t_ <= 0)
        {
            Logger::logPrintf("DeltaT >%f< is not set or is invalid, cannot compute number of interval steps (Real-time violation).", Logger::ERROR, delta_t_);
            return false;
        }

        num_interval_steps = static_cast<int> (floor(tau_ / delta_t_));
        return true;
    }

    inline bool Time::getSamplingFrequency(double& sampling_frenquency) const 
    {
        if (delta_t_ <= 0)
        {
            Logger::logPrintf("DeltaT >%f< is not set or is invalid, cannot compute sampling frenquency (Real-time violation).", Logger::ERROR, delta_t_);
            return false;
        }
        sampling_frenquency = static_cast<double> (1.0) / delta_t_;
        return true;
    }
}
#endif
