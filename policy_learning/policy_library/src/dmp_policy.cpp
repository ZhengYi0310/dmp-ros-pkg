/*************************************************************************
	> File Name: dmp_policy.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 18 Jan 2017 10:10:59 AM PST
 ************************************************************************/
// system includes 
#include <assert.h>
#include <Eigen/Core>
#include <ros/console.h>
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h>

// local includes 
#include "dmp_policy.h"

using namespace Eigen;

namespace policy_library
{
    bool DMPPolicy::initialize(dmp_lib::DMPBasePtr dmp)
    {
        if (!dmp->isInitialized())
        {
            ROS_ERROR("DMP is not initialized.");
            return false;
        }
        dmp_ =dmp;
        return (initialized_ = true);
    }
    
    bool DMPPolicy::setNumTimeSteps(const int num_time_steps)
    {
        if (!initialized_)
        {
            return false;
        }
        num_time_steps_ = num_time_steps;
        return true;
    }

    bool DMPPolicy::getNumTimeSteps(int& num_time_steps)
    {
        if (num_time_steps_ < 0)
        {
            ROS_ERROR("Number of time steps >%i< has not bees set yet.", num_time_steps_);
            return false;
        }
        num_time_steps = num_time_steps_;
        return true;
    }

    bool DMPPolicy::getNumDimensions(int& num_dimensions)
    {
        if (!initialized_)
        {
            return false;
        }
        num_dimensions = dmp_->getNumDimensions();
        return true;
    }

    bool DMPPolicy::getBasisFunctions(std::vector<MatrixXd>& basis_functions)
    {
        if (initialized_)
        {
            return false;
        }
        return dmp_->generateBasisFunctionMatrix(num_time_steps_, basis_functions);
    }

    bool DMPPolicy::computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices,
                                        const std::vector<std::vector<Eigen::VectorXd> >& parameters,
                                        const double weight,
                                        std::vector<Eigen::VectorXd>& control_costs)
    {
        int num_dimensions = control_cost_matrices.size();
        int num_time_steps = parameters[0].size();

        std::vector<int> num_parameters(num_dimensions);
        getNumParameters(num_parameters);

        // initialize output vector if necessary:
        if (int(control_costs.size()) != num_dimensions)
        {
            control_costs.clear();
            for (int d = 0; d < num_dimensions; ++d)
            {
                control_costs.push_back(VectorXd::Zero(num_time_steps));
            }
        }

        // compute the costs
        for (int d = 0; d < num_dimensions; ++d)
        {
            for (int t = 0; t < num_time_steps; ++t)
            {
                control_costs[d](t) = weight * parameters[d][t].dot(control_cost_matrices[d] * parameters[d][t]);
            }
        }

        return true;
    }

    bool DMPPolicy::getControlCosts(std::vector<MatrixXd>& control_costs)
    {
        if (!initialized_)
        {
            return false;
        }

        control_costs.clear();
        std::vector<int> num_thetas;
        if (!getNumParameters(num_thetas))
        {
            ROS_ERROR("Could not get number of parameters.");
            return false;
        }

        std::vector<VectorXd> basis_functions_centers;
        if (!dmp_->getBasisFunctionCenters(basis_functions_centers))
        {
            ROS_ERROR("Could not get basis function centers, cannot compute control cost.");
            return false;
        }

        if (num_thetas.size() != basis_functions_centers.size())
        {
            ROS_ERROR("Number of dimensions >%i< does not correspond to number of basis function vectors >%i<.", (int)num_thetas.size(), (int)basis_functions_centers.size());
            return false;
        }

        for (int i = 0; i < (int)num_thetas.size(); i++)
        {
            if (num_thetas[i] != basis_functions_centers[i].size())
            {
                ROS_ERROR("Number of theatas >%i< does not match numebr of basis function centers >%i<.", num_thetas[i], (int)basis_functions_centers[i].size());
                return false;
            }

            MatrixXd control_cost_matrix = MatrixXd::Identity(num_thetas[i], num_thetas[i]);
            for (int j = 0; j < num_thetas[i]; ++j)
            {
                control_cost_matrix(j, j) = basis_functions_centers[i](j) * basis_functions_centers[i](j);
            }
            control_costs.push_back(control_cost_matrix);
        }
        return true;
    }

    bool DMPPolicy::updateParameters(const std::vector<MatrixXd>& updates, const std::vector<Eigen::VectorXd>& time_step_weights)
    {
        if (!initialized_)
        {
            return false;
        }

        std::vector<VectorXd> theta_vectors;
        if (!dmp_->getThetas(theta_vectors))
        {
            ROS_ERROR("Could not get parameter vector. Cannot update parameters of the DMP policy.");
            return false;
        }

        assert(theta_vectors.size() == updates.size());

        std::vector<MatrixXd> basis_functions;
        if (!getBasisFunctions(basis_functions))
        {
            ROS_ERROR("Could not get parameter vector. Cannot update parameters of the DMP policy.");
            return false;
        }

        assert(basis_functions.size() == updates.size());

        for (int d = 0; d < static_cast<int>(updates.size()); d++)
        {
            int num_rfs = basis_functions[d].cols();

            assert(updates[d].rows() == static_cast<int>(num_time_steps_));
            assert(updates[d].cols() == static_cast<int>(theta_vectors[d].size()));
            assert(updates[d].rows() == basis_functions[d].rows());
            assert(updates[d].cols() == basis_functions[d].cols());

            for (int j = 0; j < num_rfs; j++)
            {
                double sum_time_weight_times_basis_funciton_weight = 0;
                double sum_time_weight_times_basis_funciton_weight_times_update = 0;
                for (int i = 0; i < num_time_steps_; i++)
                {
                    double time_weight = num_time_steps_ - i;
                    sum_time_weight_times_basis_funciton_weight += (time_weight * basis_functions[d](i, j));
                    sum_time_weight_times_basis_funciton_weight_times_update += ((time_weight * basis_functions[d](i, j)) * updates[d](i, j));
                }

                // update the theta vector 
                theta_vectors[d](j) += sum_time_weight_times_basis_funciton_weight_times_update / sum_time_weight_times_basis_funciton_weight;
            }
        }
        if (!dmp_->setThetas(theta_vectors))
        {
            ROS_ERROR("Could not set parameter vector. Cannot update parameters of the DMP policy.");
            return false;
        }

        return true;
    }

    bool DMPPolicy::getParameters(std::vector<VectorXd>& parameters)
    {
        if (!initialized_)
        {
            return false;
        }
        parameters.clear();
        return dmp_->getThetas(parameters);
    }

    bool DMPPolicy::setParameters(const std::vector<VectorXd>& parameters)
    {
        if (!initialized_)
        {
            return false;
        }
        return dmp_->setThetas(parameters);
    }

    bool DMPPolicy::getDMP(dmp_lib::DMPBasePtr& dmp)
    {
        if (!initialized_)
        {
            ROS_ERROR("DMPPolicy is not initialized.");
            return false;
        }
        dmp = dmp_;
        return true;
    }

    bool DMPPolicy::writeToFile(const std::string& abs_file_name)
    {
        return dmp_utilities::DynamicMovementPrimitiveUtilities::writeToFile(abs_file_name, dmp_);
    }

    bool DMPPolicy::readFromFile(const std::string& abs_file_name)
    {
        return dmp_utilities::DynamicMovementPrimitiveUtilities::readFromFile(abs_file_name, dmp_);
    }

    std::string DMPPolicy::getClassName()
    {
        return "DMPPolicy";
    }
}

