/*************************************************************************
	> File Name: dynamic_movement_primitive_parameters_base.h
	> Author: 
	> Mail: 
	> Created Time: Tue 18 Oct 2016 04:40:22 PM PDT
 ************************************************************************/

#ifndef _DYNAMIC_MOVEMENT_PRIMITIVE_PARAMETERS_BASE_H
#define _DYNAMIC_MOVEMENT_PRIMITIVE_PARAMETERS_BASE_H

// system includes
#include <string>
#include <boost/shared_ptr.hpp>

// local includes
#include "dmp_lib/time.h"

namespace dmp_lib
{
    /*! Parameters of a DMP to be saved to file 
     */
    class DynamicMovementPrimitiveParametersBase
    {
        /*! Allow the DynamicMovementPrimitiveBase class to access private member variables directly
         */
        friend class DynamicMovementPrimitiveBase;

        public:
            /*! Constructor 
             */ 
            DynamicMovementPrimitiveParametersBase() : teaching_duration_(0), execution_duration_(0), cutoff_(0), type_(-1), id_(0) {};

            /*! Destructor 
             */
            virtual ~DynamicMovementPrimitiveParametersBase() {};

            /*!
             * @param params 
             * @return True if equal, otherwise False 
             */
            bool operator==(const DynamicMovementPrimitiveParametersBase& params_base) const 
            {
                return ((initial_time_ == params_base.initial_time_)
                        && (fabs(teaching_duration_ - params_base.teaching_duration_ < EQUALITY_PRECISSION))
                        && (fabs(execution_duration_ - params_base.execution_duration_ < EQUALITY_PRECISSION))
                        && (fabs(cutoff_ - params_base.cutoff_) < EQUALITY_PRECISSION)
                        && (type_ == params_base.type_)
                        && (id_ == params_base.id_));
            }
            bool operator!=(const DynamicMovementPrimitiveParametersBase &params_base) const
            {
                return !(*this == params_base);
            }

            /*!
             * @param initial_time 
             * @param teaching_duration 
             * @param execution_duration
             * @param cutoff
             * @param type 
             * @param id 
             * @return True on success, otherwise False 
             */
            bool initialize(const Time& initial_time, const double teaching_duration, const double execution_duration, const double cutoff, const int type, const int id = 0);

            /*!
             * @param cutoff
             */
            bool setCutoff(const double cutoff);

            /*!
             * @param other_parameters_base 
             * @return
             */
            bool isCompatible(const DynamicMovementPrimitiveParametersBase& other_parameters_base) const;

            /*！
             * @param other_parameters_base
             * @return 
             */
            bool changeType(const DynamicMovementPrimitiveParametersBase& other_parameters_base);

            /*!
             * @param initial_time
             * @param teaching_duration 
             * @param execution_duration
             * @param cutoff
             * @param type
             * @param id
             * @return True on success, otherwise False 
             */
            bool get(Time& initial_time, double& teaching_duration, double&execution_duration, double& cutoff, int& type, int& id) const; 

        private:
            static const double EQUALITY_PRECISSION = 1e-6;

            /*! Time parameters which have been used during learning 
             */
            Time initial_time_;

            /*! Default durations are important to find values for alpha_z to 
             * obtain desired behavior
             */
            double teaching_duration_;
            double execution_duration_;

            /*! Defines when a particular movement has finished
             */
            double cutoff_;

            /*! TYpe of the DMP
             */
            int type_;

            /*! ID of the DMP 
             */ 
            int id_; 
    };

    /*! Abbreviation for convinience 
     */
    typedef DynamicMovementPrimitiveParametersBase DMPParamBase;
    typedef boost::shared_ptr<DMPParamBase> DMPParamBasePtr;
    typedef boost::shared_ptr<DMPParamBase const> DMPParamBaseConstPtr;
}
#endif
