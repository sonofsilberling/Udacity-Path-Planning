#pragma once

#ifndef DRIVER
#define DRIVER

#include "trajectory.h"
#include "road.h"
#include "utils.h"
#include "vehicle.h"
#include <vector>
#include <cmath>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include <assert.h>
#include <functional>

using namespace Eigen;


typedef std::function<bool(std::vector<double>&, const std::vector<double>&, const std::vector<double>&, const double&)> jmt_function;

class Driver {
public:

	// Attributes
	// Vehicle for the driver = ego car
	EgoVehicle vehicle;

	// Road the driver is on (if any)
	Road road;

	/**
	 * @brief Constructor
	 */
	Driver(EgoVehicle vehicle_in, Road road_in);
	Driver();

	~Driver() {}


	std::ostream& operator<<(std::ostream& os);

	/**
	 * @brief Observe the environment by assigning the cars on the road
	 *        into grid
	 */
	bool observe(std::vector<std::vector<double>>& observations, bool refresh);

	/**
	 * @brief Decide and set strategy for vehicle
	 */
	bool decide();

	/**
	 * @brief Time points in seconds
	 *        Most likely times are between 3s and 6s, hence smaller steps in that range
	 *
	 */	
	std::vector<double> target_times = {3.0, 3.5, 3.7, 4, 4.2, 4.5, 5, 5.5, 6.0, 7.0};	

protected:
	/**
	 * @brief Generate multiple trajectories
	 */	
	bool generate_trajectories();

	/**
	 * @brief Trajectory candidates 
	 */
	std::map<Lane, std::vector<Trajectory>> trajectory_candidates_d;
	std::map<Lane, std::vector<Trajectory>> trajectory_candidates_s;		
	std::vector<CombinedTrajectory> trajectory_candidates;	


	/**
	 * @brief Determine coefficients for jerk minimizing trajectory JMT when following a vehicle, i.e. when aiming for a specific position
	 */
	bool JMT_position(std::vector<double>& coefficients, const std::vector<double>& start, const std::vector<double>& end, const double& T);

	/**
	 * @brief Determine coefficients for jerk minimizing trajectory JMT when maintaining a velocity, i.e. when aiming for a specific velocity
	 */
	bool JMT_velocity(std::vector<double>& coefficients, const std::vector<double>& start, const std::vector<double>& end, const double& T);

	/**
	 * @brief Generate a single trajectory
     *
	 * @parameters
	 * JMT: function to generate coefficients
	 * cost_f: function to get cost for final stage
	 * start: position, velocity, acceleration at beginning
	 * end: position, velocity, aceeleration at end
	 * T: time to achieve all this
	 */
	bool generate_trajectory(Trajectory& trajectory, std::function<bool(std::vector<double>&, const std::vector<double>&, const std::vector<double>&, const double&)>& JMT, const std::vector<double>& start, const std::vector<double>& end, const double& T);

	/**
	 * @brief Combine s and d trajectory candidates into combined trajectory candidates
	 */
	bool combine_trajectories();

	/**
	 * @brief Check collisions on trajectory candidates
	 */
	bool check_collisions();

	double get_cost_position(std::vector<double>& coefficients, double& T, double& target_position);
	double get_cost_velocity(std::vector<double>& coefficients, double& T, double& target_velocity);

private:
	/**
	 * @brief Internal (pre-calculated) values for JMT
	 */		
	std::map<double, MatrixXd> JMT_Matrix_following;
	std::map<double, MatrixXd> JMT_Matrix_velocity;

	/**
	 * @brief Initialize data structures
	 */	
	void init();

	/**
	 * @brief Cost Functions
	 */		
	double get_cost_jerk(const std::vector<double>& coefficients, double t0, double t1);

	/**
	 * @brief Find maximum jerk on time interval t0 to t1
	 */	
	double get_max_jerk(const std::vector<double>& coefficients, double t0, double t1);	

	/**
	 * @brief Find maximum acceleration on time interval t0 to t1
	 */	
	double get_max_a(const std::vector<double>& coefficients, double t0, double t1);
	/**
	 * @brief Find maximum velocity on time interval t0 to t1
	 */	
	double get_max_v(const std::vector<double>& coefficients, double t0, double t1);
};

#endif 