#pragma once

#ifndef CONSTANTS
#define CONSTANTS

#include <string>
#include <cmath>

// Available constants = hyperparameters for the project

// Print debug messages and / or write to log
// #define DEBUG
// #define TRACE
// #define LOG

// Speed limit in meters / second
const double SPEED_LIMIT = 20.5 ; //22.262 is 49.8 mph
// Conversion factor MPH to MS
const double MPH_TO_MS = 0.44704;
// Conversion factor MS to MPH
const double MS_TO_MPH = 2.23693629;


// Number of Lanes
const int LANE_NUM = 3;
// Width of lane in meters
const int LANE_WIDTH = 4;

// Half of vehicle width - wider than normal (2m)
const double VEHICLE_WITDH = LANE_WIDTH / 2 - 0.4;
// Half of vehicle length
const double VEHICLE_LENGTH = 5.0 ;
// Expansion factor: the longer we look into the future, the longer a vehicle is assumed to cater for uncertainty
const double VEHICLE_EXPANSION = 0.02;

// Kinematics
// Maximum acceleration / deceleration
const double MAX_A = 9.0; // in m/s^2
const double MAX_A_SQUARED = MAX_A * MAX_A; // in m/s^2
// Maximum jerk
const double MAX_JERK = 50; // in m/s^3
const double MAX_JERK_SQUARED = MAX_JERK * MAX_JERK; // in m/s^3

// Minimum velocity to attempt a lane change
const double MIN_VELOCITY_LANE = 6.7; // 15 mph

// Distance to leading vehicle
const double MIN_GAP = 1.0; // in meters
const double MIN_TIME_GAP = 2.0; // in seconds

// Cycle Time = Plannning Horizon
// Cycle time in seconds
const double CYCLE_TIME = 2.5;
// Time increment for trajectory
const double CYCLE_INCREMENT = 0.02;
const int CYCLE_STEPS = CYCLE_TIME / CYCLE_INCREMENT;
// Maximum number of steps from previous cycle
const int CYCLE_STEPS_PREV = 0.5 / CYCLE_INCREMENT;

// Cost weights
const double WEIGHT_T = 50; // Time Cost
const double WEIGHT_J = 1; // Jerk Cost
const double WEIGHT_V = 10; // Final State Cost: velocity
const double WEIGHT_P = 10; // Final State Cost: position

// Lateral vs Longitudinal weight 
const double WEIGHT_COST_S = 1;
const double WEIGHT_COST_D = 2;

// Positional vs Velocity Keeping: Values < 1 means to prefer velocity keeping
const double WEIGHT_VELOCITY = 0.1;

// Map constants
// The max s value before wrapping around the track back to 0
const double MAP_MAX_S = 6945.554;

// Define a big arbitrary number
const double REALLY_BIG_NUMBER = 999999;


// Waypoint map to read from
const  std::string MAP_FILE = "../data/highway_map.csv";

#endif 