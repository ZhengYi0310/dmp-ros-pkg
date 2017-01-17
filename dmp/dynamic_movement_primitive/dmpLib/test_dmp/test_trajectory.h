/*************************************************************************
	> File Name: test_trajectory.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 02 Nov 2016 01:31:29 PM PDT
 ************************************************************************/

#ifndef _TEST_TRAJECTORY_H
#define _TEST_TRAJECTORY_H

// system includes

// local includes 
#include <dmp_lib/trajectory.h>
#include "test_data.h"

namespace test_dmp
{

    /*!
     */
    class TestTrajectory
    {
        
        public:
            
            /*!
             */
            static bool test(const std::string& filename, const TestData& testdata, const std::string = "");

        private:
            
            /*!
             */
            TestTrajectory() {};

            /*!
             */
            virtual ~TestTrajectory() {};
    };
}
#endif /* _TEST_TRAJECTORY_H */
