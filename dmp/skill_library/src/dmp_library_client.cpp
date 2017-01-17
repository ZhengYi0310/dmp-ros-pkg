/*************************************************************************
	> File Name: dmp_library_client.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 23 Dec 2016 08:53:08 PM PST
 ************************************************************************/

// system includes
#include <dynamic_movement_primitive/DMPUtilitiesMsg.h>

// local includes 
#include "skill_library/dmp_library_client.h"

using namespace dmp;
using namespace std;

namespace skill_library
{
    bool DMPLibraryClient::initialize(const string& library_root_directory)
    {
        ROS_VERIFY(nc2010_dmp_library_.initialize(library_root_directory));
        return true;
    }

    bool DMPLibraryClient::addDMP(const dmp_lib::DMPBasePtr& dmp,
                                  const string& name)
    {
        if (dmp->getVersionString() == dynamic_movement_primitive::DMPUtilitiesMsg::NC2010)
        {
            NC2010DMP::DMPMsg msg;
            NC2010DynamicMovementPrimitive::writeToMessage(boost::dynamic_pointer_cast<dmp_lib::NC2010DMP>(dmp), msg);
            ROS_VERIFY(addDMP(msg, name));
        }

        else
        {
            ROS_ERROR("Could not send DMP with version >%s< to the controller.", dmp->getVersionString().c_str());
            return false;
        }
        return true;
    }

    // NC2010
    bool DMPLibraryClient::addDMP(const NC2010DMP::DMPMsg& msg,
                                  const string& name)
    {
        return nc2010_dmp_library_.addDMP(msg, name);
    }

    bool DMPLibraryClient::getDMP(const string& name,
                                  NC2010DMP::DMPMsg& msg)
    {
        return nc2010_dmp_library_.getDMP(name, msg);
    }
}

