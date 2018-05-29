#pragma once

#ifndef TRAJECTORY
#define TRAJECTORY

#include <vector>
#include "constants.h"
#include "utils.h"
#include <cmath>
#include <iostream>

struct Trajectory {


	Trajectory() {
		coefficients.reserve(6);
		cost = REALLY_BIG_NUMBER;
		max_jerk = REALLY_BIG_NUMBER;
		max_a = REALLY_BIG_NUMBER;
	}

	Trajectory(std::vector<double>& coefficients_in, double& max_jerk_in, double& max_a_in, double& cost_in) {
		coefficients = coefficients_in;
		cost = cost_in;
		max_jerk = max_jerk_in;
		max_a = max_a_in;
	}	

	/*
	 * @brief Coefficients that describe trajectory
	 *
	 * Quintic if following another vehicle
	 * Quartic if aim is to keep velocity, i.e. quintic with last coefficient c5 = 0
	 */	
	std::vector<double> coefficients;

	/*
	 * @brief Cost of given trajectory
	 */	
	double cost;

	/*
	 * @brief Maximum jerk and acceleration when setting up the trajectory
	 */		
	double max_jerk;
	double max_a;
	double max_v;

	/*
	 * @brief Target time T
	 */	
	double T;
	std::vector<double> end;
	std::vector<double> start;

	/*
	 * @brief Total jerk and acceleration when combined with other trajectory
	 *
	 * Technically this should take the square root of the value
	 * yet is specifically does NOT do so:
	 * absolute result is irrelevant, just need to compare this result to MAX_JERK / MAX_A squared
	 */		
	double get_combined_jerk_squared(Trajectory other) { return (max_jerk*max_jerk + other.max_jerk*other.max_jerk); }
	double get_combined_a_squared(Trajectory other) { return (max_a*max_a + other.max_a*other.max_a); }

	/*
	 * @brief Type: velocity keeping (0) or position (1)
	 */
	int type;



};


struct CombinedTrajectory {

	CombinedTrajectory() {
		feasible = true;
	}

	CombinedTrajectory(std::vector<double>& coefficients_d_in, 
		               std::vector<double>& coefficients_s_in, 
		               double& max_jerk_in, 
		               double& max_a_in, 
		               double& cost_in, 
		               double& T_in, 
		               std::vector<double>& start_s_in, 
		               std::vector<double>& end_s_in, 
		               std::vector<double>& start_d_in, 
		               std::vector<double>& end_d_in, 
		               int& type_in) {

		#ifdef TRACE 
		std::cout << "__CombinedTrajectory::CombinedTrajectory" << std::endl;
		#endif		
		coefficients_d = coefficients_d_in;
		coefficients_s = coefficients_s_in;
		cost           = cost_in;
		max_jerk       = max_jerk_in;
		max_a          = max_a_in;
		T              = T_in;
		start_s        = start_s_in;
		end_s          = end_s_in;
		start_d        = start_d_in;
		end_d          = end_d_in;		
		feasible       = true;
		type           = type_in;
		lane           = get_lane(end_d_in[0]);
	}		

	/*
	 * @brief Coefficients that describe trajectory
	 *
	 * Quintic if following another vehicle
	 * Quartic if aim is to keep velocity, i.e. quintic with last coefficient c5 = 0
	 */	
	std::vector<double> coefficients_s;	
	std::vector<double> coefficients_d;	

	/*
	 * @brief Is trajectory feasible?
	 *
	 * A trajectory is not feasible if
	 * - there is a projected collision
	 * - it exceeds maximum jerk or acceleration
	 */
	bool feasible;

	/*
	 * @brief Total Cost
	 */	
	double cost;

	/*
	 * @brief Target time T
	 */	
	double T;	
	std::vector<double> end_s;
	std::vector<double> start_s;
	std::vector<double> end_d;
	std::vector<double> start_d;	

	/*
	 * @brief Combined Maximum jerk and acceleration
	 */		
	double max_jerk;
	double max_a;

	int type;
	Lane lane;

};



#endif 