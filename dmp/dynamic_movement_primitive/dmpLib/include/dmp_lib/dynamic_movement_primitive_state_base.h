/*************************************************************************
	> File Name: dynamic_movement_primitive_state_base.h
	> Author: Yi Zheng
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 17 Oct 2016 11:53:18 PM PDT
 ************************************************************************/

#ifndef _DYNAMIC_MOVEMENT_PRIMITIVE_STATE_BASE_H
#define _DYNAMIC_MOVEMENT_PRIMITIVE_STATE_BASE_H

// system includes
#include <boost/shared_ptr.hpp>

// local includes
#include "dmp_lib/time.h"

namespace dmp_lib
{
/*!
*/
class DynamicMovementPrimitiveStateBase
{
    /*! Allow the DynamicMovementPrimitiveBase class to access private member variables directly
    * */
    friend class DynamicMovementPrimitiveBase;
public:
    /*! Constructor
     */
    DynamicMovementPrimitiveStateBase() : is_learned_(false),is_setup_(false),is_start_set_(false),num_training_samples_(0),num_generated_samples_(0),seq_(0) {};

    /*! Destructor
     */
    virtual ~DynamicMovementPrimitiveStateBase() {};

    /*!
     * @param params
     * @return True if equal, otherwise False
     */
    bool operator==(const DynamicMovementPrimitiveStateBase &state_base) const
    {
        return ((is_learned_ == state_base.is_learned_) 
                && (is_setup_ == state_base.is_setup_) 
                && (is_start_set_ == state_base.is_start_set_) 
                && (current_time_ == state_base.current_time_) 
                && (num_training_samples_ == state_base.num_training_samples_) 
                && (num_generated_samples_ == state_base.num_generated_samples_) 
                && (seq_ == state_base.seq_));

    }

    bool operator!=(const DynamicMovementPrimitiveStateBase &state_base) const
    {
        return !(*this == state_base);
    }

    /*!
     * @param is_learned 
     * @param is_setup 
     * @param is_start_set 
     * @param current_time 
     * @param current_time
     * @param num_training_samples 
     * @param num_generated_samples 
     * @param seq 
     * @return True on success, otherwise False 
             */
    bool initialize(bool is_learned = false, bool is_setup = false, bool is_start_set = false, const Time& current_time = Time(), const int num_training_samples = 0, const int num_generated_samples = 0, const int seq = 0);

    /*!
     * @param other_state 
     * @return 
     */
    bool isCompatible(const DynamicMovementPrimitiveStateBase& other_state_base) const;

    /*!
     * @param is_learned 
     * @param is_setup 
     * @param is_start_set 
     * @param current_time 
     * @param num_training_samples 
     * @param num_generated_samples
     * @param seq
     * @return True on success, otherwise False 
    */
    bool get(bool& is_learned, bool& is_setup, bool& is_start_set, Time& current_time, int& num_training_samples, int& num_generated_samples, int& seq) const;

private:
    /*! Indicates whether the DMP has been learned, i.e. the parameters of the DMP has been computed/set
     */
    bool is_learned_;

    /*! Indicates whether the DMP is setup, i.e. the start, goal and duration has been set
     *
     */
    bool is_setup_;

    /*! Indicates whether the start of the DMP has been set. This flag is used to 
     * update the start position of successive DMPs, when the start position cannot 
     * be determined beforehand.
    */
    bool is_start_set_;

    /*! Time of the dmp used while integrating the system 
     */
    Time current_time_;

    /*! Number of training samples used during learning a DMP 
     */
    int num_training_samples_;

    /*! Number of generated samples used during propagating a DMP 
     */
    int num_generated_samples_;

    /*! sequence number used for communication between dmp controller and dmp client 
     */
    int seq_;
};

/*! Abbreviation for convinience 
*/
typedef DynamicMovementPrimitiveStateBase DMPStateBase;
typedef boost::shared_ptr<DMPStateBase> DMPStateBasePtr;
typedef boost::shared_ptr<DMPStateBase const> DMPStateBaseConstPtr;
}
#endif
