#include "navigator.h"


/**
 * @brief Read map data from file into memory
 */
Navigator::Navigator() {
	#ifdef TRACE 
	std::cout << "__Navigator::Navigator" << std::endl;
	#endif		

	std::string line;
	std::ifstream in_map_(MAP_FILE.c_str(), std::ifstream::in);

	while (getline(in_map_, line)) {
		std::istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	// Cater for gap at last waypoint: append first way point to end of map
	map_waypoints_x.push_back(map_waypoints_x.front());
	map_waypoints_y.push_back(map_waypoints_y.front());
	map_waypoints_s.push_back(map_waypoints_s.front() + MAP_MAX_S);	
	map_waypoints_dx.push_back(map_waypoints_dx.front());
	map_waypoints_dy.push_back(map_waypoints_dy.front());

	// Smooth waypoints
	process_map();
}

std::vector<double>  Navigator::to_frenet(double x, double y, double s_init) {

		// Gradient descent is used to find the point (s,d) on the spline, which is closest to point ptXY.
	const double EPS = 1.0e-6;
	const double GAMMA = 0.001;
	const double PRECISION = 1e-12;

	double s = s_init;
	double prev_step_size = s;

	while (prev_step_size > PRECISION)
	{
		double prev_s = s;
		s -= GAMMA * (-2. * (x - spline_x(prev_s)) * spline_dx(prev_s)
		             - 2. * (y - spline_y(prev_s)) * spline_dy(prev_s) );
		prev_step_size = fabs(s - prev_s);
	}

	double d1 = (x - spline_x(s) / spline_dy(s));
	double d2 = (y - spline_y(s) / spline_dx(s));
	double d = 0.5 * (d1 + d2);

	return {s,d};

}

/**
 * @brief Process map coordinates
 *
 * Smooth into evenly spaced steps
 */
void Navigator::process_map() {
	#ifdef TRACE 
	std::cout << "__Navigator::process_map" << std::endl;
	#endif	

	// Space s waypoints
	// make x/y coordinates dependent
	spline_x.set_points(map_waypoints_s, map_waypoints_x);
	spline_y.set_points(map_waypoints_s, map_waypoints_y);	
	spline_dx.set_points(map_waypoints_s, map_waypoints_dx);	
	spline_dy.set_points(map_waypoints_s, map_waypoints_dy);	

}
/**
 * @brief Transform from Frenet s,d coordinates to Cartesian x,y
 */
std::vector<double> Navigator::to_cartesian(double s, double d) {
	// #ifdef TRACE 
	// std::cout << "__Navigator::to_cartesian" << std::endl;
	// #endif	

	s = fmod(s, MAP_MAX_S); 
	double x = spline_x(s) + d * spline_dx(s);
	double y = spline_y(s) + d * spline_dy(s);

	return {x,y};	
}