/*************************************************************************
	> File Name: policy.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 17 Jan 2017 04:57:17 PM PST
 ************************************************************************/
// system includes 

// local includes 
#include "policy.h"
using namespace Eigen;

namespace policy_library
{
    bool Policy::computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices,
                                     const std::vector<std::vector<Eigen::VectorXd> >& parameters,
                                     const double weight,
                                     std::vector<Eigen::VectorXd>& control_costs)
    {
        int num_dimensions = control_cost_matrices.size();
        int num_time_steps = parameters[0].size();

        std::vector<int> num_parameters(num_dimensions);
        this->getNumParameters(num_parameters);

        // initialize ouput vector if necessary
        if (int(control_costs.size()) != num_dimensions)
        {
            control_costs.clear();
            for (int d = 0; d < num_dimensions; d++)
            {
                control_costs.push_back(VectorXd::Zero(num_time_steps));
            }
        }

        // compute the costs 
        for (int d = 0; d < num_dimensions; d++)
        {
            for (int t = 0; t < num_time_steps; t++)
            {
                control_costs[d](t) = weight * parameters[d][t].dot(control_cost_matrices[d] * parameters[d][t]);
            }
        }

        return true;
    }
}

