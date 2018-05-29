#pragma once

#ifndef VEHICLE
#define VEHICLE

#include "trajectory.h"
#include "navigator.h"
#include <math.h>
#include "constants.h"
#include "utils.h"
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"

/**
 * @brief Current State of a vehicle
 */
struct State
{
	// Cartesian coordinates
	double x;
	double y;

	double yaw; // in rad
	double v; // in meters / second
	double vx;// in meters / second
	double vy;// in meters / second

	// Frenet coordinates
	double s; // longitudinal
	double d; // lateral

};

class Vehicle {
public:

	Vehicle();

	virtual ~Vehicle()=0;

	/**
	 * @brief ID from simulator
	 */	
	int id;	

	Lane lane;
	State state;

	std::ostream& operator<<(std::ostream& os);

	/**
	/**
	 * @brief Check for collision with trajectory from time t0 to time t1
	 */	
	bool is_collision(double t0, double t1, std::vector<double>& coefficients_s, std::vector<double>& coefficients_d);

	/**
	 * @brief Get coefficients for current trajectory
	 */
	virtual std::vector<double> get_coefficients_s();
	virtual std::vector<double> get_coefficients_d();

	
};


class EgoVehicle : public Vehicle {
public:

	EgoVehicle();
	~EgoVehicle();

	double end_path_s;
	double end_path_d;

	// Start time from simulator based on previous path
	double t_current;

	std::vector<double> previous_path_x;
	std::vector<double> previous_path_y;

	std::vector<double> next_x_vals;
	std::vector<double> next_y_vals;
	std::vector<double> next_s_vals;
	std::vector<double> next_d_vals;	

	// State lateral / longitudinal in Frenet frame
	double s; // position
	double s_dot; // velocity
	double s_ddot; // acceleration
	double d;
	double d_dot;
	double d_ddot;

	/**
	 * @brief Execute strategy and set next x/y coordinates
	 */
	bool drive();

	/**
	 * @brief Get coefficients for current trajectory
	 */
	CombinedTrajectory trajectory;
	std::vector<double> get_coefficients_s();
	std::vector<double> get_coefficients_d();	

	void set_trajectory(CombinedTrajectory& trajectory);

	/**
	 * @brief Post-process telemetry data
	 */
	void process_telemetry();

	/**
	 * @brief Map and coordinates
	 */
	Navigator navigator;

	/**
	 * @brief Initialized flag
	 */
	bool started;	


protected:
	std::vector<double> m_coeffs_s;
	std::vector<double> m_coeffs_d;

};

class ObservedVehicle : public Vehicle {
public:

	ObservedVehicle();
	ObservedVehicle(Vehicle& ego_vehicle, std::vector<double>& observation);
	~ObservedVehicle();



	/**
	 * @brief Gap (distance) to ego vehicle: positive if in front
	 */	
	double s_gap;
	/**
	 * @brief Velocity dependent reference distance to ego vehicle
	 */	
	double s_ref;

	/**
	 * @brief Get coefficients for current trajectory
	 */
	std::vector<double> get_coefficients_s(); 
	std::vector<double> get_coefficients_d(); 

	/**
	 * @brief Get predicted position at time t
	 *
	 * Simple model with constant velocity and no lane change
	 */	
	double get_position_s(double& t) { return (state.s + t * state.v); }
	double get_position_d(double& t) { return (state.d); }

};


#endif  