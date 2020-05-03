#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::cout;
using std::endl;
using std::string;
using std::vector;

int main(int argc, char **argv) {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
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

  const double MAX_SPEED_LIMIT = (argc > 1) ? atof(argv[1]) : 49.5;   // Speed limit
  const double MAX_ACC_LIMIT   = (argc > 2) ? atof(argv[2]) : 0.224;  // Acceleration limit
  const int carGap             = (argc > 3) ? atof(argv[3]) : 30;     // Gap from the front car
  double carVel                = 0;                                   // Control variable: Speed of the car
  int carLane                  = 1;                                   // Control variable: lane number for the car

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,
               &carLane, &carVel, &carGap, &MAX_SPEED_LIMIT, &MAX_ACC_LIMIT](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                                                             size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Car's localization Data
          double carX          = j[1]["x"];
          double carY          = j[1]["y"];
          double carS          = j[1]["s"];
          double carYaw        = j[1]["yaw"];
          double carSpeed      = j[1]["speed"];
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          double end_path_s    = j[1]["end_path_s"];
          double end_path_d    = j[1]["end_path_d"];

          double carD = j[1]["d"];
          if (carD <= 4.0)
            carLane = 0;  // Left lane
          else if (carD <= 8.0)
            carLane = 1;  // Center lane
          else if (carD <= 12.0)
            carLane = 2;  // Right lane

          // Car start from the last point, if the previous path size is greater than 0.
          int prevPathSize = previous_path_x.size();
          if (prevPathSize > 0) carS = end_path_s;

          // Obstacle detection
          // The data format for each obstacle: [ id, x, y, vx, vy, s, d]
          auto sensor_fusion = j[1]["sensor_fusion"];  // List of all obstcles (same side of the road).
          bool obstacleFront = false;                  // Flag for obstacle at front
          bool obstacleLeft  = false;                  // Flag for obstacle at left side
          bool obstacleRight = false;                  // Flag for obstacle at right side
          for (int i = 0; i < sensor_fusion.size(); i++) {
            double obstVX = sensor_fusion[i][3];                      // longitudinal speed of the obstacle
            double obstVY = sensor_fusion[i][4];                      // lateral speed of the obstacle
            double obstS  = sensor_fusion[i][5];                      // S of the obstacle
            float obstD   = sensor_fusion[i][6];                      // lane position of the other obstacles
            double obstV  = sqrt(obstVX * obstVX + obstVY * obstVY);  // speed of the obstacle
            int obstLane;                                             // Obstacle lane number
            obstS += (double)(prevPathSize * 0.02 * obstV);           // Prediction

            // Determine obstacle lane number
            if (obstD > 0 && obstD < 4)
              obstLane = 0;  // Left lane
            else if (obstD > 4 && obstD < 8)
              obstLane = 1;  // Center lane
            else if (obstD > 8 && obstD < 12)
              obstLane = 2;  // Right lane
            else
              continue;

            // Determine obstacle flag
            if (obstLane == carLane)  // There's a car ahead
              obstacleFront |= (obstS > carS) && ((obstS - carS) < carGap);
            else if (obstLane - carLane == 1)  // Another car is to the right
              obstacleRight |= ((carS - carGap) < obstS) && ((carS + carGap) > obstS);
            else if (obstLane - carLane == -1)  // Another car is to the left
              obstacleLeft |= ((carS - carGap) < obstS) && ((carS + carGap) > obstS);

          }  // Obstacle detection

          // Determine speed to avoid collisions based upon obstacle information
          if (obstacleFront) {  // An obstacle is at front, decide to shift lanes or slow down
            if (!obstacleRight && carLane < 2)
              carLane++;  // No obstacle at the right AND there is a right lane
            else if (!obstacleLeft && carLane > 0)
              carLane--;  // No obstacle at the left AND there is a left lane
            else
              carVel -= MAX_ACC_LIMIT;  // Not possible to change lane, therefore, slow down
          } else {
            if (carLane != 1) {  // Not in the center lane. Check if it is safe to move back
              if ((carLane == 2 && !obstacleLeft) || (carLane == 0 && !obstacleRight))
                carLane = 1;  // Move back to the center lane
            }
            if (carVel < MAX_SPEED_LIMIT)  // No obstacle at front AND we are below the speed limit
              carVel += MAX_ACC_LIMIT;
          }

          // Trajectory Planning: create a list of evenly spaced, 30m, waypoints
          vector<double> ptsx, ptsy;
          double refX   = carX;             // car's current x-coordinate
          double refY   = carY;             // car's current y-coordinate
          double refYaw = deg2rad(carYaw);  // car's current  orientation. simulator data for yaw is in degrees

          if (prevPathSize < 2) {
            // if previous path is almost empty, use the car current location as starting reference
            // generate two points that make the path tangent to the car
            double prev_carx = carX - cos(carYaw);
            double prev_cary = carY - sin(carYaw);
            ptsx.push_back(prev_carx);
            ptsx.push_back(carX);
            ptsy.push_back(prev_cary);
            ptsy.push_back(carY);
          } else {
            // else use the previous path's end point as starting reference
            refX            = previous_path_x[prevPathSize - 1];
            refY            = previous_path_y[prevPathSize - 1];
            double refXPrev = previous_path_x[prevPathSize - 2];
            double refYPrev = previous_path_y[prevPathSize - 2];
            refYaw          = atan2(refY - refYPrev, refX - refXPrev);
            ptsx.push_back(refXPrev);
            ptsx.push_back(refX);
            ptsy.push_back(refYPrev);
            ptsy.push_back(refY);
          }

          //Adding Frenet points ahead of the car as starting reference
          vector<double> nextWP0 = getXY(carS + 30, (2 + 4 * carLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextWP1 = getXY(carS + 50, (2 + 4 * carLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextWP2 = getXY(carS + 70, (2 + 4 * carLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextWP3 = getXY(carS + 90, (2 + 4 * carLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(nextWP0[0]);
          ptsx.push_back(nextWP1[0]);
          ptsx.push_back(nextWP2[0]);
          ptsx.push_back(nextWP3[0]);
          ptsy.push_back(nextWP0[1]);
          ptsy.push_back(nextWP1[1]);
          ptsy.push_back(nextWP2[1]);
          ptsy.push_back(nextWP3[1]);

          // Change car reference angle to 0 degrees in the car's local coordinate
          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - refX;
            double shift_y = ptsy[i] - refY;
            ptsx[i]        = (shift_x * cos(0 - refYaw) - shift_y * sin(0 - refYaw));
            ptsy[i]        = (shift_x * sin(0 - refYaw) + shift_y * cos(0 - refYaw));
          }

          tk::spline sp;                        // Create the spline.
          sp.set_points(ptsx, ptsy);            //set (x,y) points to the spline
          vector<double> nextXVals, nextYVals;  // define the actual (x,y) points we will use for the planner

          // Generate next path plann points, start with previous path points
          for (int i = 0; i < previous_path_x.size(); i++) {
            nextXVals.push_back(previous_path_x[i]);
            nextYVals.push_back(previous_path_y[i]);
          }

          // Populate spline points in order to travel at desired velocity
          double targetX    = carGap;
          double targetY    = sp(targetX);  // Spline interpolation
          double targetDist = sqrt(targetX * targetX + targetY * targetY);
          double dX         = 0;  //since we are starting at the origin
          for (int i = 1; i < 50 - previous_path_x.size(); i++) {
            double N      = targetDist / (0.02 * carVel / 2.24);  // 2.24: conversion to m/s
            double pointX = dX + targetX / N;
            double pointY = sp(pointX);  // Spline interpolation
            dX            = pointX;
            double tempX  = pointX;                                     // temporary variable
            double tempY  = pointY;                                     // temporary variable
            pointX        = tempX * cos(refYaw) - tempY * sin(refYaw);  // Transform to global coordinate
            pointY        = tempX * sin(refYaw) + tempY * cos(refYaw);
            pointX += refX;
            pointY += refY;
            nextXVals.push_back(pointX);
            nextYVals.push_back(pointY);
          }

          json msgJson;
          msgJson["next_x"] = nextXVals;
          msgJson["next_y"] = nextYVals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

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