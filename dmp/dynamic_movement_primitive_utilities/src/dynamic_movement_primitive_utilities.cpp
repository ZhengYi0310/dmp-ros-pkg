/*************************************************************************
	> File Name: dynamic_movement_primitive_utilities.cpp
	> Author: Yi Zheng
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 07 Dec 2016 05:57:23 PM PST
 ************************************************************************/

#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <usc_utilities/assert.h>
#include <usc_utilities/file_io.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive_io.h>
#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/TypeMsg.h>

// local includes 
#include "dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h"

using namespace std;
using namespace dmp;

namespace dmp_utilities
{
    bool DynamicMovementPrimitiveUtilities::getControllerName(const dmp_lib::DMPBasePtr dmp,
                                                              std::string& controller_name)
    {
        if (dmp->hasType(dynamic_movement_primitive::TypeMsg::DISCRETE_JOINT_SPACE))
        {
            controller_name.assign("/SL/" + dmp->getVersionString() + "DMPJointTrajectoryController");
        }

        else if ((dmp->hasType(dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_SPACE)) || (dmp->hasType(dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_AND_JOINT_SPACE)))
        {
            controller_name.assign("/SL/" + dmp->getVersionString() + "DMPCartesianTrajectoryController");
        }

        else
        {
            ROS_ERROR("DMP is of invalid type >%i<.", dmp->getType());
            return false;
        }

        return true;
    }

    bool DynamicMovementPrimitiveUtilities::getDMP(const dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg, dmp_lib::DMPBasePtr& dmp)
    {
        //if (dmp_utilities_msg.dmp_version == dynamic_movement_primitive::DMPUtilitiesMsg::ICRA2009)
        //{
        //    return getDMP(dmp_utilities_msg.icra2009_dmp, dmp);
        //}

        if (dmp_utilities_msg.dmp_version == dynamic_movement_primitive::DMPUtilitiesMsg::NC2010)
        {
            return getDMP(dmp_utilities_msg.nc2010_dmp, dmp);
        }

        else
        {
            ROS_ERROR("DMP version >%s< is invalid, needed to get DMP.", dmp_utilities_msg.dmp_version.c_str());
            return false;
        }

        return true;
    }

    //bool DynamicMovementPrimitiveUtilities::getDMP(const dmp::ICRA2009DMPMsg& msg,
    //                                               dmp_lib::DMPBasePtr& dmp)
    //{
    //    ICRA2009DynamicMovementPrimitive::DMPPtr icra2009_dmp;
    //    if(!ICRA2009DynamicMovementPrimitive::createFromMessage(icra2009_dmp, msg))
    //    {
    //        return false;
    //    }
    //    dmp = icra2009_dmp;
    //    return true;
    //}

    bool DynamicMovementPrimitiveUtilities::getDMP(const dmp::NC2010DMPMsg& msg,
                                                   dmp_lib::DMPBasePtr& dmp)
    {
        NC2010DynamicMovementPrimitive::DMPPtr nc2010_dmp;
        if (!NC2010DynamicMovementPrimitive::createFromMessage(nc2010_dmp, msg))
        {
            return false;
        }
        dmp = nc2010_dmp;
        return true;
    }

    bool DynamicMovementPrimitiveUtilities::writeToFile(const std::string& abs_bag_file_name, const dmp_lib::DMPBasePtr& dmp)
    {
        if (dmp->getVersionString() == dynamic_movement_primitive::DMPUtilitiesMsg::NC2010)
        {
            if (!NC2010DynamicMovementPrimitive::writeToDisc(boost::dynamic_pointer_cast<dmp_lib::NC2010DMP>(dmp), abs_bag_file_name))
            {
                return false;
            }
        }

        else 
        {
            ROS_ERROR("DMP version >%s< is invalid, needed to get DMP.", dmp->getVersionString().c_str());
            return false;
        }

        return true;
    }

    bool DynamicMovementPrimitiveUtilities::readFromFile(const std::string& abs_bag_file_name, dmp_lib::DMPBasePtr& dmp)
    {
        dmp_lib::NC2010DMPPtr nc2010_dmp;
        if (!DynamicMovementPrimitiveIO<NC2010DMP, NC2010DMPMsg>::readFromDisc(nc2010_dmp, abs_bag_file_name, false))
        {
            return false;
        }

        if (nc2010_dmp.get())
        {
            dmp = nc2010_dmp;
        }
        else
        {
            ROS_ERROR("Bag file >%s< does not contain a DMP.", abs_bag_file_name.c_str());
            return false;
        }

        return true;
    }

    bool DynamicMovementPrimitiveUtilities::setMsg(const dmp_lib::DMPBasePtr& dmp,
                                                   dmp::NC2010DMPMsg& msg)
    {
        if (dmp->getVersionString() == dynamic_movement_primitive::DMPUtilitiesMsg::NC2010)
        {
            return NC2010DynamicMovementPrimitive::writeToMessage(boost::dynamic_pointer_cast<dmp_lib::NC2010DMP>(dmp), msg);
        }

        return false;
    }

    bool DynamicMovementPrimitiveUtilities::getGoal(const dmp::NC2010DMPMsg& msg,
                                                    std::vector<double> goal)
    {
        dmp_lib::DMPBasePtr dmp;
        if (!getDMP(msg, dmp))
        {
            return false;
        }
        return dmp->getGoal(goal, false);
    }

    bool DynamicMovementPrimitiveUtilities::getGoal(const dmp::NC2010DMPMsg& msg,
                                                    const std::vector<std::string>& variable_names,
                                                    std::vector<double> goal)
    {
        dmp_lib::DMPBasePtr dmp;
        if (!getDMP(msg, dmp))
        {
            return false;
        }
        return dmp->getGoal(variable_names, goal);
    }


    
}

