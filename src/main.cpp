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

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

//int main(int argc, char *argv[]) {
int main() {
  uWS::Hub h;

  // Initialize the pid variable
  PID pid;

  // PID parameters
//  double init_Kp = 0.2;
//  double init_Ki = 0.0001;
//  double init_Kd = 3.0;

  double init_Kp = 0.1; // 0.1
  double init_Ki = 0.0001;
  double init_Kd = 3; //5

  pid.Init(init_Kp, init_Ki, init_Kd);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());

          //
          pid.Twiddle(cte);

          //
          //pid.TwiddleError(cte);

          // Calculate PID error
          pid.UpdateError(cte);

          // Calculate steering which is the control input from PID error. The steering value is in [-1, 1]
          double steer_value = pid.TotalError();

          // DEBUG
          std::cout << "--------------" << std::endl;
          std::cout << "Counter: " << pid.counter << " | error_accumulation_started: " << pid.error_accumulation_started << " | parameter_tuning_index: " << pid.parameter_tuning_index << std::endl;
          std::cout << "Kp: " << pid.Kp << " | Ki: " << pid.Ki << " | Kd: " << pid.Kd << std::endl;
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << std::endl;
          std::cout << "dp[0]: " << pid.dp[0] << " | dp[1]: " << pid.dp[1] << " | dp[2]: " << pid.dp[2] << std::endl;
          std::cout << "initial_best_error_recorded: " << pid.initial_best_error_recorded << " | best_error: " << pid.best_error<< " | error: " << pid.error << std::endl;
          std::cout << "--------------" << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
