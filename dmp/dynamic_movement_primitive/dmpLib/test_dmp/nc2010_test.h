/*************************************************************************
	> File Name: nc2010_test.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 03 Nov 2016 11:46:34 AM PDT
 ************************************************************************/

#ifndef _NC2010_TEST_H
#define _NC2010_TEST_H

// system includes
// local includes
#include <dmp_lib/nc2010_dynamic_movement_primitive.h>
#include "test_data.h"

namespace test_dmp
{

    class NC2010Test
    {

        public:

        static bool initialize(dmp_lib::NC2010DMP& dmp, const TestData& testdata);

        private:

            NC2010Test() {};
            virtual ~NC2010Test() {};

    };

}

#endif
