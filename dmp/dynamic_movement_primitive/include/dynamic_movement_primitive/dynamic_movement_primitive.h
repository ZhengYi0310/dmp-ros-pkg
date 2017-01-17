/*************************************************************************
	> File Name: dynamic_movement_primitive.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 14 Nov 2016 08:36:14 PM PST
 ************************************************************************/

#ifndef _DYNAMIC_MOVEMENT_PRIMITIVE_H
#define _DYNAMIC_MOVEMENT_PRIMITIVE_H

// system includes 
#include <ros/ros.h>
#include <vector>
#include <string>
#include <dmp_lib/dynamic_movement_primitive_base.h>

// local includes 
#include <dynamic_movement_primitive/DynamicMovementPrimitiveMsg.h>

namespace dmp 
{
    /*! Abbreviation for convinience
     */
    typedef dynamic_movement_primitive::DynamicMovementPrimitiveMsg DMPMsg;

    class DynamicMovementPrimitive
    {
        public:
            /*! Initializes a DMP from parameters on the param server in the namespace of the node handle
             * @param dmp (input)
             * @param node_handle (input)
             * @return True if initialization is successful, otherwise False
             */
            static bool initFromNodeHandle(dmp_lib::DMPBasePtr dmp,
                                           ros::NodeHandle& node_handle);

            /*! Initializes a DMP from a message
             * @param dmp (input)
             * @param dmp_msg (input)
             * @return True if initialization is successful, otherwise False
             */
            static bool initFromMessage(dmp_lib::DMPBasePtr dmp,
                                        const DMPMsg& dmp_msg);

            /*!
             * @param dmp
             * @param dmp_msg
             * @return True if successful, otherwise False
             */
            static bool writeToMessage(dmp_lib::DMPBaseConstPtr dmp,
                                       DMPMsg& dmp_msg);

        private:

            /*! Constructor
             */
            DynamicMovementPrimitive() {};

            /*! Destructor
             */
            virtual ~DynamicMovementPrimitive() {};            
    };

    /*! Abbreviation for convinience
     */
    typedef DynamicMovementPrimitive DMP;
    typedef boost::shared_ptr<DMP> DMPPtr;
    typedef boost::shared_ptr<DMP const> DMPConstPtr;
}
#endif /* _DYNAMIC_MOVEMENT_PRIMITIVE_H */
