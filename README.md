# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


### Implementation
The project implements the goals based on the work of Werling et al. and plans completely in a Frenet space by generating trajectories for velocity keeping (no vehicle ahead) or position (vehicle ahead), and selecting the best via a cost function.

![Simulator Screenshot](https://github.com/sonofsilberling/Udacity-Path-Planning/blob/master/screenshot.jpg)

The ego vehicle has a preference for driving in the center lane by weighting costs for each lane, and it has a preference to follow a velocity keeping trajectory, i.e. driving in a lane without a vehicle ahead.


### Structure

The structure of the project can be broken into the following parts:
- the driver (the behavior module) observes the road by identifying vehicles
- the driver generates possible trajectories, dismisses "bad" ones and selects the best of the remaining ones
- the driver asks the ego vehicle to execute (drive) the selected trajectory
- the ego vehicle executes the trajectory (drives) by calculating waypoints ahead and asks the navigator to translate it into cartesian coordinates for the simulator

### Observation
Most effort in implementing the project was not with the approach on handling trajectories or calibrating the cost function, which still has scope to improve.
Most effort was in dealing with the simulator and its waypoints. In the end, the project uses the waypoints to generate a single spline along the path, and the only two values coming back from the simulator for the ego vehicle are the initial s and d coordinates, which quickly get smoothed into the generated spline - hence the initial "flicker" of the ego vehicle.
