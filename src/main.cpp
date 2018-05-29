#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <ratio>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"

#include "driver.h"
#include "constants.h"
#include "utils.h"

#ifdef LOG
#include <glog/stl_logging.h>
#endif

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main(int argc, char* argv[]) {
  uWS::Hub h;

  #ifdef LOG
  FLAGS_log_dir = "./log";
  google::InitGoogleLogging(argv[0]);  
  #endif

  // Initialize Ego Car, Road, and Driver
  Driver driver = Driver();

  h.onMessage([&driver](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
    uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Measure start time
          auto time_start = chrono::steady_clock::now();

          // Main car's localization Data
          driver.vehicle.state.x = j[1]["x"];
          driver.vehicle.state.y = j[1]["y"];
          driver.vehicle.state.s = j[1]["s"];
          driver.vehicle.state.d = j[1]["d"];
          driver.vehicle.state.yaw = j[1]["yaw"];
          driver.vehicle.state.v = j[1]["speed"].get<double>() * MPH_TO_MS;

          // Previous path data given to the Planner
          driver.vehicle.previous_path_x = j[1]["previous_path_x"].get<vector<double>>();
          driver.vehicle.previous_path_y = j[1]["previous_path_y"].get<vector<double>>();

          // Previous path's end s and d end values
          driver.vehicle.end_path_s = j[1]["end_path_s"].get<double>();
          driver.vehicle.end_path_d = j[1]["end_path_d"].get<double>();
          
          // Post process telemetry data
          driver.vehicle.process_telemetry();

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"].get<vector<vector<double>>>();

          // Use previously calculated steps a minimum set of steps left
          if (driver.vehicle.previous_path_x.size()  > 150 ) {
            driver.vehicle.next_x_vals = driver.vehicle.previous_path_x;
            driver.vehicle.next_y_vals = driver.vehicle.previous_path_y;
          }
          else {
            if (driver.observe(sensor_fusion, true)) {
              // Make a decision
              if (driver.decide()) {
                #ifdef DEBUG
                std::cout << "Found a good trajectory!!!!" << std::endl;
                #endif
                // Execute decision and output result
                driver.vehicle.drive();
              }

              #ifdef LOG
              else {
                LOG(INFO) << "NO(!) good trajectory!!!!" << std::endl;
              }
              #endif
            }
          }

          // Measure end time and time to get here
          // This determines which next x / y points we hand back to the simulator
          // assuming that the simulator kept going at 0.02s interval
          auto time_end = chrono::steady_clock::now();
          int time_units_elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start).count() / (1000*CYCLE_INCREMENT);

          // assert(time_units_elapsed < driver.vehicle.next_x_vals.size());

          #ifdef DEBUG
          std::cout << "Time units elapsed: " << time_units_elapsed << std::endl;
          #endif

          json msgJson;

          msgJson["next_x"] = driver.vehicle.next_x_vals;
          msgJson["next_y"] = driver.vehicle.next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
          // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse * res, uWS::HttpRequest req, char *data,
    size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
        // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
    char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
