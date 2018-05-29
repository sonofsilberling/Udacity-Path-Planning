#pragma once

#ifndef ROAD
#define ROAD


#include "vehicle.h"
#include <map>
#include <vector>
#include <iostream>

/**
 * @brief Road represents the road based on sensor fusion data
 */
class Road {
public:

	Road();

	// Keep a map of vehicles for each lane
	// key to the map is the distance d along the frenet frame
	std::map<Lane, std::vector<ObservedVehicle>> vehicles;
	std::map<Lane, std::vector<ObservedVehicle>> vehicles_ahead;

	std::ostream& operator<<(std::ostream& os);

    /*
	 * observe environment
	 * Add vehicles from sensor fusion data and process their states
	 *
	 *  @refresh: clear all previous observations (or not)

	  * @return: all successful (true) or not (false)
	 */
	bool observe(Vehicle& ego_vehicle, std::vector<std::vector<double>>& observations, bool refresh);



	void reset();

	/**
	 * @brief Center for each lane
	 */
	std::map<int, double> lane_centers =  {
		{0, 2.2},
		{1, 6.0},
		{2, 9.8},
	};
	/**
	 * @brief weights for each lane: favour center lane by a small margin
	 *        Affects cost calculation
	 */
	std::map<int, double> lane_weights =  {
		{0, 1.0},
		{1, 0.95}, // Lower weight to center lane
		{2, 1.0},
	};


};

#endif 
