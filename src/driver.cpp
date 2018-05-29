#include "driver.h"

#ifdef LOG
#include <glog/stl_logging.h>
#endif

using namespace std::placeholders;
/**
 * @brief Constructor
 */
Driver::Driver(EgoVehicle vehicle_in, Road road_in) {
	vehicle = vehicle_in;
	road = road_in;
	init();
};
Driver::Driver() {
	init();
};


std::ostream& Driver::operator<<(std::ostream& os) {
	// os << "Driver in " << vehicle << " on " << road;
	return os;
}

/**
 * @brief Initialize internal data structures
 *
 * Pre-calculate time-dependent matrices for jerk minimizing trajectory JMT
 */
void Driver::init() {

	#ifdef TRACE 
	std::cout << "__Driver::init__" << std::endl;
	#endif
	MatrixXd A(3,3);
	MatrixXd B(2,2);
	
	JMT_Matrix_following.clear();
	JMT_Matrix_velocity.clear();

 	// pre-calculate transformation matrices for JMT
 	// see Werling M et al "Optimal trajectories for time-critical street scenarios using discretized terminal manifolds"
	for (double& T: target_times) {

		double T2 = T  * T;
		double T3 = T2 * T;
		double T4 = T3 * T;
		double T5 = T4 * T;

		A <<   T3,    T4,    T5,
		     3*T2,  4*T3,  5*T4,
		      6*T, 12*T2, 20*T3;

		B << 3*T2,  4*T3,
		      6*T, 12*T2;		      

		JMT_Matrix_following[T] = A.inverse();
		JMT_Matrix_velocity[T]  = B.inverse();
	}
}

/**
 * @brief Observe the environment by assigning the cars on the road
 *        into grid
 */
bool Driver::observe(std::vector<std::vector<double>>& observations, bool refresh) {
	#ifdef TRACE
		std::cout << "__driver::observe__" << std::endl;
	#endif

	// Set up road
	road.observe(vehicle, observations, refresh);

	return true;
}


/**
 * @brief Decide and set strategy for vehicle
 */
bool Driver::decide() {
	#ifdef TRACE
	std::cout << "__driver::decide__" << std::endl;
	#endif

	Lane current_lane;


	// Generate Trajectory candidates and cost those
	// Check for vehicles ahead in each lane: following vs cruising

	/**
	 * Requirement for all trajectories:
	 * - starting position is given
	 * - starting velocity and acceleration are given by current coefficients
	 */	
	if (!generate_trajectories())
		return false;

    // Combine trajectories and assign total cost
	if (!combine_trajectories())
		return false;

	// Collision check: find lowest cost trajectory with no collision
	if (!check_collisions(-100)) {
		// If no collision free trajectory can be found
		// Check only cars ahead. It might be that a car just came up from behind
		if (!check_collisions(-1)) {
			// There is no collision free trajectory
			// In this case take lowest cost trajectory that stays in the lane
			// Better than giving up is to try and move forward a little bit and hope that next cycle
			// the situation will have improved
			// This happens if a vehicle pulls in right in front of ego vehicle
			for (CombinedTrajectory& trajectory : trajectory_candidates) {
				if (trajectory.lane == vehicle.lane) {
					trajectory.feasible = true;
					break;
				}
			}
			
		}
	}

	#ifdef LOG
	int n_counter = 0;
	for (CombinedTrajectory& trajectory : trajectory_candidates) {
		if (trajectory.feasible) {
			LOG(INFO) << "type: " << trajectory.type << " - cost " << trajectory.cost << " - target lane " << trajectory.lane << std::endl;
			n_counter++;
			if (n_counter > 10)
				break;
		}
	}	
	#endif

	// Assign best feasible trajectory to vehicle
	for (CombinedTrajectory& trajectory : trajectory_candidates) {
		if (trajectory.feasible) {
			#ifdef LOG
			if (trajectory.lane == vehicle.lane)
				LOG(INFO) << "Decision: stay in lane " << vehicle.lane;
			else
				LOG(INFO) << "Decision: swith from lane " << vehicle.lane << " to " << trajectory.lane;
			#endif
			vehicle.set_trajectory(trajectory);
			return true;
		}
	}

	return false;
}

/**
 * @brief Check collision for each trajectory candidate
 *
 * NOTE: trajectory candidates assumed to be sorted by cost ascending, i.e. checks lowest cost trajectory first
 *       trajectories all assumed to be set to feasible = true
 */
bool Driver::check_collisions(int s_gap_min) {
	#ifdef TRACE 
		std::cout << "__Driver::check_collisions__" << std::endl;
	#endif
	std::map<Lane, std::vector<ObservedVehicle>>::iterator it_lane;
	std::vector<ObservedVehicle>::iterator it_vehicle;

	for (CombinedTrajectory& trajectory : trajectory_candidates) {
	// iterate through all observed vehicles on road
		trajectory.feasible = true;
		it_lane = road.vehicles_ahead.begin();
		while (trajectory.feasible & it_lane != road.vehicles_ahead.end()) {
			it_vehicle = it_lane->second.begin();

			while (trajectory.feasible & it_vehicle != it_lane->second.end()) {
				// For for a maximum of 1 seconds into the future
				if (it_vehicle->s_gap >= s_gap_min)
					trajectory.feasible = trajectory.feasible && !(it_vehicle->is_collision(0, std::min(1.0, trajectory.T), trajectory.coefficients_s, trajectory.coefficients_d));
				++it_vehicle;
			}
			++it_lane;
		}
	// Feasible trajectory found? If so return, no need to check for further collisions
		if (trajectory.feasible)
			return true;
	}

// No feasible trajectory found
	return false;
}

/**
 * @brief Combine all found trajectories
 *
 * Check for maximum jerk and acceleration
 * Set cost
 */
bool Driver::combine_trajectories() {

	#ifdef TRACE 
	std::cout << "__Driver::combine_trajectories__" << std::endl;
	#endif

	trajectory_candidates.clear();
	trajectory_candidates.reserve(30);
	for (Lane lane : {left, center, right}) {
		for (Trajectory& trajectory_d : trajectory_candidates_d[lane]) {
			for (Trajectory& trajectory_s : trajectory_candidates_s[lane]) {
				// Get combined jerk and acceleration
				double max_jerk = trajectory_s.get_combined_jerk_squared(trajectory_d);
				double max_a    = fabs(trajectory_s.max_a) + fabs(trajectory_d.max_a);
				if (max_jerk <= MAX_JERK_SQUARED & max_a <= MAX_A) {

					double cost = WEIGHT_COST_D * trajectory_d.cost + WEIGHT_COST_S * trajectory_s.cost;
					double T = std::max(trajectory_s.T, trajectory_d.T);
					
					trajectory_candidates.emplace_back(trajectory_d.coefficients, 
						                               trajectory_s.coefficients, 
						                               max_jerk, 
						                               max_a, 
						                               cost, 
						                               T, 
						                               trajectory_s.start, 
						                               trajectory_s.end, 
						                               trajectory_d.start, 
						                               trajectory_d.end, 
						                               trajectory_s.type);
				}
			}
		}
	}

	// Sort trajectories by cost
    std::sort(trajectory_candidates.begin(), trajectory_candidates.end(), [](CombinedTrajectory& t1, CombinedTrajectory& t2) {
        return (t1.cost < t2.cost);
    });	
   

	return true;
}

/**
 * @brief Generate tranjectories for ego vehicle
 */
bool Driver::generate_trajectories() {
	#ifdef TRACE 
	std::cout << "__Driver::generate_trajectories__" << std::endl;
	#endif

	std::vector<Lane> target_lanes;

	// final parameters: position, velocity, acceleration
	double sf, sf_dot, sf_ddot;
	double df, df_dot, df_ddot;	

	double cost_d;
	double cost_s;

	double jerk_max;
	double a_max;

	std::vector<double> coefficients;
	Trajectory trajectory_candidate;

		// Clear all previous trajectories
	for (Lane lane : {left, center, right}) {
		trajectory_candidates_d[lane].clear();
		trajectory_candidates_s[lane].clear();
		// Reserve memory for candidates
		trajectory_candidates_d[lane].reserve(30);
		trajectory_candidates_s[lane].reserve(30);
	}

	/**
	 * Determine target lanes
	 *
	 * Check for minimum speed:
	 * - if below a minimum speed, gain speed within current lane
	 * - if above a minimum speed, include other lane for options
	 * - only if vehicle is actually on the road, otherwise ignore minimum speed
	 */
	if (vehicle.lane==unknown) {
		target_lanes = {left, center, right};
	}
	else {
		if (vehicle.state.v < MIN_VELOCITY_LANE ) {
			// Target lane
			target_lanes = {vehicle.lane};
		}
		// If we are fast enough or not on the road, test for all lanes
		else {
			switch (vehicle.lane) {
				case left:
			target_lanes = {left, center};
			break;
				case center:
			target_lanes = {left, center, right};
			break;
				case right:
			target_lanes = {center, right};
			break;
			}
			// target_lanes = {left, czenter, right};
		}
	}

	#ifdef DEBUG
	std::cout << "Target Lanes: ";
	for (Lane& d: target_lanes) {
		std::cout << d << ", " ;
	}
	std::cout << "\b\b " << std::endl;
	#endif


	// aim for target lateral velocity and acceleration to be 0
	df_dot  = 0;
	df_ddot = 0;
	// aim for target longitudinal acceleration to be 0	
	sf_ddot = 0;

	for (Lane& lane : target_lanes) {

		// Generate lateral positions
		// Target lateral position is lane center
		df = road.lane_centers[lane];
		jmt_function JMT = std::bind(&Driver::JMT_position, this, _1, _2, _3, _4);
		for (double& T : target_times) {
			if (generate_trajectory(trajectory_candidate, JMT, {vehicle.d, vehicle.d_dot, vehicle.d_ddot}, {df, df_dot, df_ddot}, T)) {
				trajectory_candidate.cost = road.lane_weights[lane] *get_cost_position(trajectory_candidate.coefficients, T, df);	
				trajectory_candidate.T = T;				
				trajectory_candidate.type = 1;
				trajectory_candidates_d[lane].emplace_back(trajectory_candidate);
			}				
		}		

		bool velocity_keeping = false;
		if (road.vehicles_ahead[lane].size() > 0)
			velocity_keeping = (road.vehicles_ahead[lane][0].s_gap > 60);
		else
			velocity_keeping = (road.vehicles_ahead[lane].size() == 0 );

		if (velocity_keeping) {
			#ifdef LOG
			LOG(INFO) << "Trajectory for velocity keeping in lane " << lane << std::endl;
			#endif

			// Target longitudinal velocity
			sf_dot = SPEED_LIMIT;
			// Longitudinal
			jmt_function JMT = std::bind(&Driver::JMT_velocity, this, _1, _2, _3, _4);
			for (double& T : target_times) {
				// Target position arbitrary - here 0
				if (generate_trajectory(trajectory_candidate, JMT, {vehicle.s, vehicle.s_dot, vehicle.s_ddot}, {0,  sf_dot, sf_ddot}, T)) {
					trajectory_candidate.cost = get_cost_velocity(trajectory_candidate.coefficients, T, sf_dot);
					trajectory_candidate.type = 0;
					trajectory_candidates_s[lane].emplace_back(trajectory_candidate);
				}
			}

		}
		else {
			// Vehicle ahead - aim for following at safe velocity dependent distance
			// Future enhancement: generated trajectory for all vehicles ahead
			// just in case the ego vehicle could 'squeeze' in  successfully
			ObservedVehicle& observed_vehicle = road.vehicles_ahead[lane][0];
			#ifdef LOG
			LOG(INFO) << "Trajectory for following vehicle " << observed_vehicle.id << " in lane " << lane << " with distance " << observed_vehicle.s_gap << std::endl;
			#endif

			// Desired velocity is leading vehicle's velocity
			sf_dot = observed_vehicle.state.v;
			jmt_function JMT = std::bind(&Driver::JMT_position, this, _1, _2, _3, _4);

			for (double& T : target_times) {
				// Desired position is leading vehicle's position at time T - (safety margin + 2 seconds)
				sf = observed_vehicle.get_position_s(T) - (MIN_GAP + MIN_TIME_GAP * sf_dot);
				if (generate_trajectory(trajectory_candidate, JMT, {vehicle.s, vehicle.s_dot, vehicle.s_ddot}, {sf, sf_dot, sf_ddot}, T)) {
					trajectory_candidate.cost = get_cost_position(trajectory_candidate.coefficients, T, sf);	
					trajectory_candidate.type = 1;
					trajectory_candidates_s[lane].emplace_back(trajectory_candidate);
				}

			}

		}

			// Sort trajectory candidates according to their cost
	    std::sort(trajectory_candidates_s[lane].begin(), trajectory_candidates_s[lane].end(), [](Trajectory t1, Trajectory t2) {
	        return (t1.cost < t2.cost);
	    });
	    std::sort(trajectory_candidates_d[lane].begin(), trajectory_candidates_d[lane].end(), [](Trajectory t1, Trajectory t2) {
	        return (t1.cost < t2.cost);
	    });
	}

	return true;

}


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
bool Driver::generate_trajectory(Trajectory& trajectory, std::function<bool(std::vector<double>&, const std::vector<double>&, const std::vector<double>&, const double&)>& JMT, const std::vector<double>& start, const std::vector<double>& end, const double& T) {
	#ifdef TRACE 
	std::cout << "__Driver::generate_trajectory__" << std::endl;
	#endif	
	// Mainly for lateral: if we start and end in the same position, nothing to do
	if (start == end)
		return true;

	if (JMT(trajectory.coefficients, start, end, T)) {

		trajectory.max_jerk = get_max_jerk(trajectory.coefficients, 0, T);
		trajectory.max_a    = get_max_a(trajectory.coefficients, 0, T);
		trajectory.max_v    = get_max_v(trajectory.coefficients, 0, T);

		// Save trajectory parameters 
		trajectory.T = T;
		trajectory.start = start;
		trajectory.end = end;

		if ((trajectory.max_jerk <= MAX_JERK) & (trajectory.max_a <= MAX_A) & (trajectory.max_v <= SPEED_LIMIT))
			return true;
	}

	return false;
}



double Driver::get_cost_position(std::vector<double>& coefficients, double& T, double& target_position) { 
	// Total Jerk + Time + Position difference
	return WEIGHT_J * get_cost_jerk(coefficients, 0, T) + WEIGHT_T * T +  WEIGHT_P * pow((eval_polynomial(T, coefficients) - target_position),2);
}
double Driver::get_cost_velocity(std::vector<double>& coefficients, double& T, double& target_velocity) {
	// Total Jerk + Time + Velocity difference
	// Always prefer velocity keeping by adjusting down
	return WEIGHT_VELOCITY * (WEIGHT_J * get_cost_jerk(coefficients, 0, T) + WEIGHT_T * T + WEIGHT_V * pow((eval_polynomial_dot(T, coefficients) - target_velocity),2));

}

/**
* @brief Calculate the Jerk Minimizing Trajectory that connects the initial state
*        to the final state in time T
*/
bool Driver::JMT_position(std::vector<double>& coefficients, const std::vector<double>& start, const std::vector<double>& end, const double& T) {
    /*
    see Werling M et al "Optimal trajectories for time-critical street scenarios using discretized terminal manifolds"

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    */

	#ifdef TRACE 
	std::cout << "__Driver::JMT_position" << std::endl;
	#endif    

    assert(start.size()==3);
    assert(end.size()==3);

    VectorXd b(3);
    VectorXd x(3);

    b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T*T), 
         end[1] - (start[1] + start[2]*T), 
         end[2] - start[2];

    x = JMT_Matrix_following[T] * b;

    coefficients = {start[0], start[1], start[2]/2, x[0], x[1], x[2]};

    return true;
}


/**
* @brief Calculate the Jerk Minimizing Trajectory that connects the initial state
*        to the final state in time T when attempting to keep a velocity
*/
bool Driver::JMT_velocity(std::vector<double>& coefficients, const std::vector<double>& start, const std::vector<double>& end, const double& T) {
    /*
    see Werling M et al "Optimal trajectories for time-critical street scenarios using discretized terminal manifolds"

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    */
	#ifdef TRACE 
	std::cout << "__Driver::JMT_velocity" << std::endl;
	#endif  

    assert(start.size()==3);
    assert(end.size()==3);

    VectorXd b(2);
    VectorXd x(2);

    b << end[1] - (start[1] + start[2]*T), 
         end[2] - (start[2]);


    x = JMT_Matrix_velocity[T] * b;

    coefficients = {start[0], start[1], start[2]/2, x[0], x[1], 0};

    return true;
}


/**
 * @brief Cost Functions: total jerk cost
 */	
double Driver::get_cost_jerk(const std::vector<double>& coefficients, double t0, double t1) {
	#ifdef TRACE 
	std::cout << "__Driver::get_cost_jerk" << std::endl;
	#endif   
	// Defined as integral over jerk function squared from t0 to t1
	// jerk function is fourth derivative of coefficients
	std::function<double(double)> jerk_integral = [&coefficients](double T) {  
	                return (36 * coefficients[3] * coefficients[3] * T \
						+  144 * coefficients[3] * coefficients[4] * T * T \
						+ (192 * coefficients[4] * coefficients[4] + 240 * coefficients[3] * coefficients[5]) * T * T * T \
						+  720 * coefficients[4] * coefficients[5] * T*T*T*T \
						+  720 * coefficients[5] * coefficients[5] * T*T*T*T*T);  
	        } ;

	if (t0 == 0) {
		return (jerk_integral(t1));
	}
	return (jerk_integral(t1) - jerk_integral(t0));

}


/**
 * @brief Find maximum jerk on time interval t0 to t1
 */	
double Driver::get_max_jerk(const std::vector<double>& coefficients, double t0, double t1) {
	#ifdef TRACE 
	std::cout << "__Driver::get_max_jerk" << std::endl;
	#endif   

	// Assert a Quintic polynomial
	assert(coefficients.size()==6);

	// Maximum at interval edges
	double max_jerk = std::max(fabs(6*coefficients[3]+24*coefficients[4]*t0),fabs(6*coefficients[3]+24*coefficients[4]*t1));

	if (coefficients[5]!=0) {
		// fifth derivative set to 0
		double t = - coefficients[4] / (coefficients[5] * 5);
		if (t0 <= t & t<=t1) {
			max_jerk = std::max(max_jerk, fabs(6*coefficients[3]+24*coefficients[4]*t+60*coefficients[5]*t*t));
		}
	}

	return max_jerk;
}

/**
 * @brief Find maximum acceleration on time interval t0 to t1
 */	
double Driver::get_max_a(const std::vector<double>& coefficients, double t0, double t1) {
	#ifdef TRACE 
	std::cout << "__Driver::get_max_a" << std::endl;
	#endif 

	// Assert a Quintic polynomial
	assert(coefficients.size()==6);

	// Maximum at interval edges
	double max_a = std::max(fabs(eval_polynomial_dot_dot(t0, coefficients)),fabs(eval_polynomial_dot_dot(t1, coefficients)));

	if (coefficients[5]==0 & coefficients[4]!=0) {
			double t_critical = - coefficients[3] / (4 * coefficients[4]);
			if (t0 <= t_critical & t_critical <= t1) {
				max_a = std::max(max_a, fabs(eval_polynomial_dot_dot(t_critical, coefficients)));
			}		
	}
	else {
		// Fourth derivative set to 0
		double a = sqrt(576*coefficients[4]*coefficients[4]-1400*coefficients[3]*coefficients[5]);
		double t_critial_1 = - (24 * coefficients[4] + a) / (120*coefficients[5]);
		double t_critial_2 = - (24 * coefficients[4] - a) / (120*coefficients[5]);

		// Maximum at critical points
		if (t0 <= t_critial_1 & t_critial_1 <= t1) {
			max_a = std::max(max_a, fabs(eval_polynomial_dot_dot(t_critial_1, coefficients)));
		}
		if (t0 <= t_critial_2 & t_critial_2 <= t1) {
			max_a = std::max(max_a, fabs(eval_polynomial_dot_dot(t_critial_2, coefficients)));
		}	
	}

	return max_a;			

}

/**
 * @brief Find maximum velocity on time interval t0 to t1
 */	
double Driver::get_max_v(const std::vector<double>& coefficients, double t0, double t1) {
	#ifdef TRACE 
	std::cout << "__Driver::get_max_v" << std::endl;
	#endif 

	// Assert a Quintic polynomial
	assert(coefficients.size()==6);

	// Maximum at interval edges
	double max_v = 0;

	for (double t = t0; t < t1+1; t+=CYCLE_INCREMENT )
		double max_v = std::max(max_v,fabs(eval_polynomial_dot(t, coefficients)));

	return max_v;			

}