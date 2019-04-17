#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Function to ensure that a variable does not go over (under) a determined upper (lower) limit.
double clip_signal(double variable, double min_limit, double max_limit){ return fmax(fmin(variable, max_limit), min_limit);}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  /**
   * Initialize the pid variable.
   */
  double kp, ki, kd;
  // PIDs parameters
  kp = 0.15;
  ki = 0.002;
  kd = 3.1;
  pid.Init(kp, ki, kd);

  // Set initial throttle and the vehicle's target velocity
  double throttle = 0;
  // Set the vehicle's target margin of velocity (between target_vel - target_vel_margin and target_vel + target_vel_margin)
  double target_vel = 35; // mph
  double target_vel_margin = 3; // mph

  h.onMessage([&pid, &throttle, &target_vel, &target_vel_margin](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                             size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * Calculate steering value here, remember the steering value is
           * [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          // Update the controller's errors using the new cte
          pid.UpdateError(cte);
          // Use the controller to calculate the next steering angle
          steer_value = pid.NextAction();
          steer_value = clip_signal(steer_value,-1,1);

          // Change in the car's trhottle to follow the target speed
          if (speed > target_vel + target_vel_margin){
            throttle -= 0.1;
          } else if(speed < target_vel + target_vel_margin) {
            throttle += 0.05;
          } else {
            // Don't accelerate if the car is within the target speed tolerance zone
            throttle = 0.0;
          }

          // Clip throttle between 0 and 0.5
          throttle = clip_signal(throttle,0,0.5);
          
          // Reduce the car's speed if it is making a steep turn
          if (fabs(angle) > 8.0){
            throttle -= 0.2;
            // Allow the car to break a little
            throttle = clip_signal(throttle,-0.08,0.5);
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}