/*************************************************************************
	> File Name: dmp_library_client.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 23 Dec 2016 08:13:26 PM PST
 ************************************************************************/

#ifndef _DMP_LIBRARY_CLIENT_H
#define _DMP_LIBRARY_CLIENT_H

// system includes
#include <string>

#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>

// local includes 
#include "skill_library/dmp_library.h"

namespace skill_library
{
    class DMPLibraryClient
    {
        public:

            DMPLibraryClient() {};
            virtual ~DMPLibraryClient() {};

            /*!
             * @param data_directory_name
             * @return 
             */
            bool initialize(const std::string& library_root_directory);

            /*!
             * @param dmp 
             * @param name 
             * @return 
             */
            bool addDMP(const dmp_lib::DMPBasePtr& dmp,
                        const std::string& name);

            /*! NC2010 functions 
             */
            bool addDMP(const dmp::NC2010DMP::DMPMsg& msg,
                        const std::string& name);

            bool getDMP(const std::string& name,
                        dmp::NC2010DMP::DMPMsg& dmp_message);

        private:
            
            /*!
             */
            DMPLibrary<dmp::NC2010DMP, dmp::NC2010DMPMsg> nc2010_dmp_library_;
    };
}
#endif
