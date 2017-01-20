/*************************************************************************
	> File Name: policy.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 17 Jan 2017 03:32:46 PM PST
 ************************************************************************/

#ifndef _POLICY_H
#define _POLICY_H

#include <vector>
#include <Eigen/Core>

namespace policy_library
{
    class Policy 
    {
        public:
            
            Policy() {};
            virtual ~Policy() {};
            
            /**
             * Sets the number of time steps used in reinforcement learning
             * @param num_time_steps
             * @return true on success, false on failure
             */
            virtual bool setNumTimeSteps(const int num_time_steps) = 0;

            /**
             * Gets the number of time steps used in reinforcement learning
             * @param num_time_steps
             * @return true on success, fase on failure
             */
            virtual bool getNumTimeSteps(int& num_time_steps) = 0;
            
            /**
             * Gets the number of dimensions
             * @param num_dimensions (output) number of dimensions
             * @return true on success, false on failure
             */
            virtual bool getNumDimensions(int& num_dimensions) = 0;

            /**
             * Gets the number of policy parameters per dimension
             *
             * @param num_params (output) vector of number of parameters per dimension
             * @return true on success, false on failure
             */
            virtual bool getNumParameters(std::vector<int>& num_params) = 0;

            /**
             * Gets the basis functions that multiply the policy parameters in the dynamical system
             * @param basis_function_matrix_array (output) Array of "num_time_steps x num_parameters" matrices, per dimension
             * @return true on success, false on failure
             */
            virtual bool getBasisFunctions(std::vector<Eigen::MatrixXd>& basis_functions) = 0;

            /**
             * Gets the basis functions that multiply the policy parameters in the dynamical system
             * @param basis_function_matrix_array (output) Array of "num_time_steps x num_parameters" matrices, per dimension
             * @return true on success, false on failure
             */
            virtual bool getControlCosts(std::vector<Eigen::MatrixXd>& basis_functions) = 0;

            /**
             * Update the policy parameters based on the updates per timestep
             * @param updates (input) parameter updates per time-step, num_time_steps x num_parameters
             * @return true on success, false on failure
             */
            virtual bool updateParameters(const std::vector<Eigen::MatrixXd>& updates, const std::vector<Eigen::VectorXd>& time_step_weights) = 0;

            /**
             * Get the policy parameters per dimension
             * @param parameters (output) array of parameter vectors
             * @return true on success, false on failure
             */
            virtual bool getParameters(std::vector<Eigen::VectorXd>& parameters) = 0;

            /**
             * Set the policy parameters per dimension
             * @param parameters (input) array of parameter vectors
             * @return true on success, false on failure
             */
            virtual bool setParameters(const std::vector<Eigen::MatrixXd>& parameters) = 0;

            /**
             * Compute the control costs over time, given the control cost matrix per dimension and parameters over time
             * @param control_cost_matrices (input) [num_dimensions] num_parameters x num_parameters: Quadratic control cost matrix (R)
             * @param parameters (input) [num_dimensions][num_time_steps] num_parameters: Parameters over time (can also be theta + projected noise)
             * @param weight (input) constant multiplier for the control costs
             * @param control_costs (output) [num_dimensions] num_time_steps: Control costs over time
             * @return
             */
            virtual bool computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices, const std::vector<std::vector<Eigen::VectorXd> >& parameters, const double weight, std::vector<Eigen::VectorXd>& control_costs);

            /**
             * Read the policy from file
             * @param abs_file_name
             * @return
             */
            virtual bool readFromFile(const std::string& abs_file_name) = 0;

            /**
             * Write the policy to file
             * @param abs_file_name
             * @return
             */
            virtual bool writeToFile(const std::string& abs_file_name) = 0;
    };
}
#endif
