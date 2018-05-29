#include "road.h"

#ifdef LOG
#include <glog/stl_logging.h>
#endif

Road::Road() {

}

std::ostream& Road::operator<<(std::ostream& os) {
// os << "Road with " << vehicles.size() << " cars";
	return os;
}

/**
* @brief Reset previous observations (if any)
*/
void Road::reset() {
	vehicles.clear();
}

/**
* @brief Process any observations from sensor fustion data
*/
bool Road::observe(Vehicle& ego_vehicle, std::vector<std::vector<double>>& observations, bool refresh) {
	double s_ref;
	double s_gap;
	#ifdef DEBUG
	int vehicle_count = 0;
	#endif

	if (refresh)
		reset();

	std::vector<ObservedVehicle> vehicles_left;
	std::vector<ObservedVehicle> vehicles_center;
	std::vector<ObservedVehicle> vehicles_right;
	std::vector<ObservedVehicle> vehicles_left_ahead;
	std::vector<ObservedVehicle> vehicles_center_ahead;
	std::vector<ObservedVehicle> vehicles_right_ahead;

	for (std::vector<double>& observation : observations) {

		ObservedVehicle vehicle = ObservedVehicle(ego_vehicle, observation);

		// Only update if
		// - the vehicle is close enough ahead or behind to be relevant 
		// - the vehicle is actually on the road and in a lane
		if (vehicle.s_gap < 99 & vehicle.s_gap > -60) {
		// if (vehicle.s_ref > 0) {
				#ifdef DEBUG
				vehicle_count++;
				#endif
				// Store observed vehicles according to gap to ego vehicle
				// There should never be two vehicles in the same spot
				switch(vehicle.lane) {
					case unknown:
					#ifdef DEBUG
						std::cout << "Observed vehicle is not on the road: d = " << vehicle.state.d << std::endl;
					#endif					
					break;
					case left :
						vehicles_left.push_back(vehicle);
						if (vehicle.s_gap > 0.)
							vehicles_left_ahead.push_back(vehicle);
					break;
					case center :
					vehicles_center.push_back(vehicle);
						if (vehicle.s_gap > 0.)
							vehicles_center_ahead.push_back(vehicle);						
					break;
					case right :
					vehicles_right.push_back(vehicle);
						if (vehicle.s_gap > 0.)
							vehicles_right_ahead.push_back(vehicle);
					break;
					default :

					break;

				}
			}
		}

		// Sort vehicles by their gap to ego vehicle
	    std::sort(vehicles_left_ahead.begin(), vehicles_left_ahead.end(), [](ObservedVehicle& t1, ObservedVehicle& t2) {
	        return (t1.s_gap < t2.s_gap);
	    });
	    std::sort(vehicles_center_ahead.begin(), vehicles_center_ahead.end(), [](ObservedVehicle& t1, ObservedVehicle& t2) {
	        return (t1.s_gap < t2.s_gap);
	    });
	    std::sort(vehicles_right_ahead.begin(), vehicles_right_ahead.end(), [](ObservedVehicle& t1, ObservedVehicle& t2) {
	        return (t1.s_gap < t2.s_gap);
	    });

		vehicles[left] = vehicles_left;
		vehicles[center] = vehicles_center;
		vehicles[right] = vehicles_right;
		vehicles_ahead[left] = vehicles_left_ahead;
		vehicles_ahead[center] = vehicles_center_ahead;
		vehicles_ahead[right] = vehicles_right_ahead;

		return true;
	}