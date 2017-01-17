/*************************************************************************
	> File Name: nc2010_dynamic_movement_primitive.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 01 Nov 2016 11:06:19 AM PDT
 ************************************************************************/

#ifndef _NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_H
#define _NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_H

// system includes
#include <vector>
#include <string>

// local includes 
#include "dmp_lib/dynamic_movement_primitive_base.h"
#include "dmp_lib/nc2010_dynamic_movement_primitive_state.h"
#include "dmp_lib/nc2010_dynamic_movement_primitive_parameters.h"
#include "dmp_lib/nc2010_transformation_system.h"
#include "dmp_lib/nc2010_canonical_system.h"

namespace dmp_lib
{
    class NC2010DynamicMovementPrimitive : public DynamicMovementPrimitiveBase
    {

        public:
            
            /*! Constructor 
             */
            NC2010DynamicMovementPrimitive() {};

            /*! Destructor 
             */
            virtual ~NC2010DynamicMovementPrimitive() {};

            /*! Assignment operator 
             * @param nc2010dmp 
             * @return 
             */
            NC2010DynamicMovementPrimitive& operator=(const NC2010DynamicMovementPrimitive& nc2010dmp);

            /*!
             * @param parameters 
             * @param state 
             * @param transformation_systems 
             * @param canonical_system 
             * @return true on success, otherwise false 
             */
            bool initialize(NC2010DMPParamPtr& parameters, NC2010DMPStatePtr& state, std::vector<NC2010TSPtr>& transformation_systems, NC2010CSPtr& canonical_system);

            /*!
             * @param nc2010_dmp
             * @return 
             */
            bool add(const NC2010DynamicMovementPrimitive& nc2010_dmp, bool check_for_compatibility = true);

            /*!
             * @param parameters 
             * @param state 
             * @return true on success, otherwise false 
             */
            bool get(NC2010DMPParamConstPtr& parameters, NC2010DMPStateConstPtr& state, std::vector<NC2010TSConstPtr>& transformation_systems, NC2010CSConstPtr& canonical_system) const;

            /*!
             * @return true if initialization is successful, otherwise false 
             */
            bool initialize(const std::vector<std::string>& variable_names, lwr_lib::LWRParamPtr lwr_parameters, const double k_gain, const double d_gain);

            /*! Returns the version string 
             * @return version string 
             */
            std::string getVersionString() const 
            {
                return "NC2010";
            }

        private:

            /*!
             */
            NC2010DMPParamPtr parameters_;

            /*!
             */
            NC2010DMPStatePtr state_;

            /*!
             */
            std::vector<NC2010TSPtr> transformation_systems_;

            /*!
             */
            NC2010CSPtr canonical_system_;
    };

    /*! Abbreviation for convinience
     */
    typedef NC2010DynamicMovementPrimitive NC2010DMP;
    typedef boost::shared_ptr<NC2010DMP> NC2010DMPPtr;
    typedef boost::shared_ptr<NC2010DMP const> NC2010DMPConstPtr;
}
#endif /* _NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_H */
