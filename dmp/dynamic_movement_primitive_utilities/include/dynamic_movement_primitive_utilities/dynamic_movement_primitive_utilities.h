/*************************************************************************
	> File Name: dynamic_movement_primitive_utilities.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 05 Dec 2016 12:00:42 PM PST
 ************************************************************************/

#ifndef _DYNAMIC_MOVEMENT_PRIMITIVE_UTILITIES_H
#define _DYNAMIC_MOVEMENT_PRIMITIVE_UTILITIES_H

// system includes 
#include <vector>

#include <ros/ros.h>

#include <inverse_kinematics/inverse_kinematics_with_nullspace_optimization.h>

#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/DMPUtilitiesMsg.h>
#include <dynamic_movement_primitive/TypeMsg.h>

// local includes 
namespace dmp_utilities 
{
    class DynamicMovementPrimitiveUtilities
    {
        public:

            /*!
             * @param dmp 
             * @param controller_name 
             * @return True on success, otherwise False 
             */
            static bool getControllerName(const dmp_lib::DMPBasePtr dmp,
                                          std::string& controller_name);

            /*!
             * @param dmp 
             * @param cost 
             * @return True on success, otherwise False 
             */
            static bool computeCost(const dmp_lib::DMPBasePtr dmp,
                                    double& cost);

            /*!
             * @param dmp_utilities_msg 
             * @param dmp 
             * @return True on success, otherwise False 
             */
            static bool getDMP(const dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg,
                          dmp_lib::DMPBasePtr& dmp);

            /*!
             * @param msg 
             * @param dmp 
             * @return True on success, otherwise False
             */
            static bool getDMP(const dmp::NC2010DMPMsg& msg,
                               dmp_lib::DMPBasePtr& dmp);

            /*! Write the dmp to file 
             * @param abs_bag_file_name 
             * @param dmp 
             * @return True on success, otherwise False
             */
            static bool writeToFile(const std::string& abs_bag_file_name, const dmp_lib::DMPBasePtr& dmp);

            /*! Read the dmp from file 
             * @oaram abs_bag_file_name
             * @param dmp
             * @return True on success, otherwise False 
             */
            static bool readFromFile(const std::string& abs_bag_file_name, dmp_lib::DMPBasePtr& dmp);

            /*!
             * @param dmp 
             * @param msg 
             * @return True on success, otherwise False
             */
            static bool setMsg(const dmp_lib::DMPBasePtr& dmp,
                               dmp::NC2010DMPMsg& msg);

            /*!
             * @param msg 
             * @param goal 
             * @return True on success, otherwise False 
             */
            static bool getGoal(const dmp::NC2010DMPMsg& msg,
                                std::vector<double> goal);

            /*!
             * @param msg 
             * @param variable_names 
             * @param goal 
             * @return True on success, otherwise False 
             */
            static bool getGoal(const dmp::NC2010DMPMsg& msg,
                                const std::vector<std::string>& variable_names,
                                std::vector<double> goal);

        private:

            /*!
             */
            DynamicMovementPrimitiveUtilities() {};

            /*!
             */
            virtual ~DynamicMovementPrimitiveUtilities() {};
    };
}
#endif
