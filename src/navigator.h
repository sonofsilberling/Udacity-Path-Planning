#pragma once

#ifndef NAVIGATOR
#define NAVIGATOR

#include "spline.h"
#include "constants.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <sstream>
#include <array>
#include <fstream>

/**
 * @brief Navigator class to process waypoints and trajetories
 */
class Navigator {
  public:

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	std::vector<double> map_waypoints_x;
	std::vector<double> map_waypoints_y;
	std::vector<double> map_waypoints_s;
	std::vector<double> map_waypoints_dx;
	std::vector<double> map_waypoints_dy;

	Navigator();

	/**
	 * @brief Reads map data from file
	 */
	void read_map();

	/**
	 * @brief Pre-processes map
	 */
	void process_map();	

	/**
	 * @brief Transform from Frenet s,d coordinates to Cartesian x,y
	 */
	std::vector<double> to_cartesian(double s, double d);

	/**
	 * @brief Transform from Frenet s,d coordinates to Cartesian x,y
	 */
	std::vector<double> to_frenet(double x, double y, double yaw);


  protected:

	  tk::spline spline_x;
	  tk::spline spline_y;
	  tk::spline spline_dx;
	  tk::spline spline_dy;
	  
};

#endif 