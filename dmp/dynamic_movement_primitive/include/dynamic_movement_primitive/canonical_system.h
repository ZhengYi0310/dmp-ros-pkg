/*************************************************************************
	> File Name: canonical_system.h
	> Author: 
	> Mail: 
	> Created Time: Mon 14 Nov 2016 05:21:18 PM PST
 ************************************************************************/

#ifndef _CANONICAL_SYSTEM_H
#define _CANONICAL_SYSTEM_H

// system includes
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <dmp_lib/canonical_system_base.h>

// local includes
#include "dynamic_movement_primitive/CanonicalSystemMsg.h"

namespace dmp
{
    /*! Abbreviation for convinience
     */
    typedef dynamic_movement_primitive::CanonicalSystemMsg CSMsg;

    class CanonicalSystem 
    {
        /*! Allow the DynamicMovementPrimitive class to access private member variables directly 
         */
        friend class DynamicMovementPrimitive;

        public:

            /*!
             * @param canonical_system
             * @param node_handle
             * @return True if success, otherwise False
             */
            static bool initFromNodeHandle(dmp_lib::CSBasePtr canonical_system, ros::NodeHandle& node_handle);

            /*!
             * @param canonical_system
             * @param cs_msg
             * @return True if success, otherwise False
             */
            static bool initFromMessage(dmp_lib::CSBasePtr canonical_system, const CSMsg& cs_msg);


            /*!
             *
             * @param canonical_system
             * @param cs_msg
             * @return True if success, otherwise False
             */
            static bool writeToMessage(const dmp_lib::CSBaseConstPtr canonical_system, CSMsg& cs_msg);

        private:
            
            /*! Constructor
             */
            CanonicalSystem() {};

            /*! Destructor 
             */
            virtual ~CanonicalSystem() {};
    };

    /*! Abbreviation for convenience
     */
    typedef CanonicalSystem CS;
    typedef boost::shared_ptr<CS> CSPtr;
    typedef boost::shared_ptr<CS const> CSConstPtr;
}
#endif /* _CANONICAL_SYSTEM_H */
