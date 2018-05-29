#include "vehicle.h"
#ifdef LOG
#include <glog/stl_logging.h>
#endif

std::ostream& Vehicle::operator<<(std::ostream& os) {
	os << "Vehicle";
	return os;
}

Vehicle::Vehicle() {

}
Vehicle::~Vehicle() {

}
std::vector<double> Vehicle::get_coefficients_s() {
	assert(false);
}
std::vector<double> Vehicle::get_coefficients_d() {
	assert(false);
}
/**
 * @brief Check for collision with other vehicle from time t0 to time t1
 *        given a set of coefficients in frenet frame
 *
 * The model represents a vehicle with a rectangle that lengthens over time to simulate a safety distance
 */	
bool Vehicle::is_collision(double t0, double t1, std::vector<double>& coefficients_s, std::vector<double>& coefficients_d) {
	#ifdef TRACE 
	std::cout << "__Vehicle::check_collision" << std::endl;
	#endif 

	std::vector<double> my_coefficients_s = get_coefficients_s();
	std::vector<double> my_coefficients_d = get_coefficients_d();

	// If there is no time gap, there can't be a collision
	if (t1 <= t0)
		return false;

	for (double t = t0; t<t1; t+=(CYCLE_INCREMENT * 2 ) ) {

		// Get center points of rectangle
		// Longitudinal position
		double s       = eval_polynomial(t, my_coefficients_s);
		double s_dot   = eval_polynomial_dot(t, my_coefficients_s);

		double s_other = eval_polynomial(t, coefficients_s);
		// Lateral position
		double d       = eval_polynomial(t, my_coefficients_d);
		double d_other = eval_polynomial(t, coefficients_d);


		// Expand vehicle along s axis
		// Distance is time and velocity dependent:
		// - the longer we look ahead the "longer" the vehicle gets
		// - the faster the vehicles moves, the "longer" is gets
		// Velocity is first derivative of coefficients
		double half_length = (t - t0) * VEHICLE_EXPANSION * s_dot + VEHICLE_LENGTH;

		// Keep other vehicle at same length: it's position is assumed to be more accurate
		// double half_length_other = (t - t0) * VEHICLE_EXPANSION * eval_polynomial_dot(t, coefficients_d) + 2;
		double half_length_other = VEHICLE_LENGTH;

		if ( 
				(d-VEHICLE_WITDH) < (d_other+VEHICLE_WITDH) &
				(d+VEHICLE_WITDH) > (d_other-VEHICLE_WITDH) &
				(s+half_length) > (s_other-half_length_other) &
				(s-half_length) < (s_other+half_length_other)
			)
			// Collision detected
			return true;
		
	}

	// No collision detected
	return false;
}


std::vector<double> EgoVehicle::get_coefficients_s() {
	return m_coeffs_s;
}
std::vector<double> EgoVehicle::get_coefficients_d() {
	return m_coeffs_d;
}

/**
 * @brief Coefficients setters
 */
void EgoVehicle::set_trajectory(CombinedTrajectory& trajectory_in) {
	#ifdef TRACE 
	std::cout << "__EgoVehicle::set_trajectory" << std::endl;
	#endif 	

	m_coeffs_s = trajectory_in.coefficients_s;
	m_coeffs_d = trajectory_in.coefficients_d;
	trajectory = trajectory_in;	

}

/**
 * @brief Post-process telemetry data
 */
void EgoVehicle::process_telemetry() {
	#ifdef TRACE 
	std::cout << "__EgoVehicle::process_telemetry" << std::endl;
	#endif 	

	if (!started) {

		s      = state.s;
		s_dot  = state.v;
		s_ddot = 0.0;
		d      = state.d;
		d_dot  = 0.0;
		d_ddot = 0.0;

		t_current = 0.0;

		m_coeffs_s = {state.s,0,0,0,0,0};
		m_coeffs_d = {state.d,0,0,0,0,0};

		started = true;
	}
	else {

		t_current = (CYCLE_STEPS - previous_path_x.size() ) * CYCLE_INCREMENT;

		// Determine current position, velocity, acceleration based on current coefficient,
		std::vector<double> coefficients_d = get_coefficients_d();
		std::vector<double> coefficients_s = get_coefficients_s();
		s      = eval_polynomial(t_current, coefficients_s);
		s_dot  = eval_polynomial_dot(t_current, coefficients_s);
		s_ddot = eval_polynomial_dot_dot(t_current, coefficients_s);
		d      = eval_polynomial(t_current, coefficients_d);
		lane   = get_lane(d);
		d_dot  = eval_polynomial_dot(t_current, coefficients_d);
		d_ddot = eval_polynomial_dot_dot(t_current, coefficients_d);		
	}

	if (s > MAP_MAX_S) {s = s - MAP_MAX_S;}
			// Determine lane from d value
	lane = get_lane(d);

	#ifdef LOG
	LOG(INFO)  << "Time t_current: " << t_current << std::endl;
	LOG(INFO)  << "Starting Position s: " << state.s << ", " << s << ", " << s_dot << ", " << s_ddot << std::endl;
	LOG(INFO)  << "Starting Position d: " << state.d << ", " << d << ", " << d_dot << ", " << d_ddot << std::endl;
	#endif

	// assert(fabs(d-state.d) < 3.0);
	// assert(fabs(s-state.s) < 3.0);

}

/**
 * @brief Execute Driving Strategy
 */
bool EgoVehicle::drive() {
	#ifdef TRACE 
	std::cout << "__EgoVehicle::drive" << std::endl;
	#endif	

	std::vector<double> coefficients_d = get_coefficients_d();
	std::vector<double> coefficients_s = get_coefficients_s();

	next_x_vals.clear();
	next_x_vals.reserve(CYCLE_STEPS);
	next_y_vals.clear();
	next_y_vals.reserve(CYCLE_STEPS);
	// next_s_vals.clear();
	// next_s_vals.reserve(CYCLE_STEPS);
	// next_d_vals.clear();
	// next_d_vals.reserve(CYCLE_STEPS);

	// Set next coordinates
	const int previous_path_size = static_cast<int>(previous_path_x.size());
	int previous_predictive_path_size = std::min(10, previous_path_size);
	for (int i = 0; i < previous_predictive_path_size; ++i)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	State current_state = state;

	for (double t = (previous_predictive_path_size ) * CYCLE_INCREMENT; t < CYCLE_TIME; t += CYCLE_INCREMENT)
	{

		current_state.s      = eval_polynomial(t, coefficients_s);
		current_state.d      = eval_polynomial(t, coefficients_d);
		// current_state.v      = eval_polynomial_dot(t, coefficients_s);

		std::vector<double> coordinates = navigator.to_cartesian(current_state.s, current_state.d);
		
		next_x_vals.push_back(coordinates[0]);
		next_y_vals.push_back(coordinates[1]);
		// next_s_vals.push_back(current_state.s);
		// next_d_vals.push_back(current_state.d);		

	}		

	return true;
}

EgoVehicle::~EgoVehicle() {
}


/**
 * @brief Constructor
 */
EgoVehicle::EgoVehicle() {

	#ifdef TRACE 
	std::cout << "__EgoVehicle::EgoVehicle" << std::endl;
	#endif		

	// Make sure current coefficients are cleared
	m_coeffs_s.clear();
	m_coeffs_d.clear();

	// Set all coefficients to 0 = vehicle is stopped and not moving
	m_coeffs_s = {0,0,0,0,0,0};
	m_coeffs_d = {0,0,0,0,0,0};

	id = -1;

	// Vehicle has not yet started its journey
	started = false;
}

ObservedVehicle::~ObservedVehicle() {
}

ObservedVehicle::ObservedVehicle() {
}

ObservedVehicle::ObservedVehicle(Vehicle& ego_vehicle, std::vector<double>& observation) {
	#ifdef TRACE 
	std::cout << "__ObservedVehicle::ObservedVehicle" << std::endl;
	#endif
	id        = observation[0];
	state.x   = observation[1];
	state.y   = observation[2];
	state.vx  = observation[3] * MPH_TO_MS;
	state.vy  = observation[4] * MPH_TO_MS;
	state.s   = observation[5];
	state.d   = observation[6];
	state.v   = sqrt(state.vx * state.vx + state.vy * state.vy);

	state.yaw = atan2(state.vy, state.vx); // atan???
	lane      = get_lane(state.d);

	// Safety distance spacing policy at maximum acceleration of vehicle behind
	// compared to maximum deceleration of vehicle ahead
	// Naively assumes a common maximum acceleration across all vehicles
	s_gap = state.s - ego_vehicle.state.s;
	// if (s_gap > 0)
	// 	s_ref = MIN_GAP + MIN_TIME_GAP * ego_vehicle.state.v + MAX_A * (ego_vehicle.state.v * ego_vehicle.state.v - state.v * state.v ) - s_gap;
	// else
	// 	s_ref = MIN_GAP + MIN_TIME_GAP * state.v + MAX_A * (- ego_vehicle.state.v * ego_vehicle.state.v + state.v * state.v ) + s_gap;

}

/**
 * @brief For an observed vehicle it is assumed that it travels along the lane without acceleration
 */
std::vector<double> ObservedVehicle::get_coefficients_s() { 
	#ifdef TRACE 
	std::cout << "__ObservedVehicle::get_coefficients_s" << std::endl;
	#endif	
	std::vector<double> s(2);
	s = {state.s, state.v};
	return s; 
}

/**
 * @brief For an observed vehicle it is assumed that it doesn't change lanes
 */
std::vector<double> ObservedVehicle::get_coefficients_d() {
	#ifdef TRACE 
	std::cout << "__ObservedVehicle::get_coefficients_d" << std::endl;
	#endif		
	std::vector<double> d(2);
	d = {state.d, 0.0};	
	return d; 
};	
