#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid_steer;
  PID pid_throttle;

  // Twiddle-algorithm:
  // [max_speed is mostly reached during 2nd lap, right before the first curve]
  // 1.) Twiddle to find optimal coefficient for steering only to finalize laps very stable at maxcte=2.5, set throttle to a constant value (e.g. 0.3); choose the following in the code:
  //     - pid_steer.Init(0.0, 0.0, 0.0, true, "pid_steer");
  //     - pid_throttle.Init(0.0, 0.0, 0.0, false, "pid_throttle");
  //     - throttle = 0.3;
  // 2.) Keep steering constant with idenfified, optimal coefficients and twiddle to find optimal throttle coefficients at maxcte=2.5; choose the following in the code:
  //     - Constant optimal coefficients for steering, here: pid_steer.Init(0.153144, 0.0, 2.9, false, "pid_steer");
  //     - pid_throttle.Init(0.0, 0.0, 0.0, true, "pid_throttle");
  //     - pid_throttle.UpdateError(fabs(steer_value));
  //     - throttle = 0.7 + 0.3*tanh(pid_throttle.TotalError());
  // 3.) Twiddle to find optimal coefficient for steering to hit maximum speed based upon the first optimal set of coefficients which enabled a very safe finalization of labs at maxcte=4.0, keep the optimal throttle coefficients; choose the following in the code:
  //     - Optimal coefficients for steering, here: pid_steer.Init(0.153144, 0.0, 2.9, true, "pid_steer");
  //     - Constant optimal throttle coefficients, here: pid_throttle.Init(1.0, 0.0, 0.0, false, "pid_throttle");
  //     - pid_throttle.UpdateError(fabs(steer_value));
  //     - throttle = 0.7 + 0.3*tanh(pid_throttle.TotalError());
  //pid_steer.Init(0.0, 0.0, 0.0, true, "pid_steer"); // Setup for twiddle, 1st round
  //pid_steer.Init(0.153144, 0.0, 2.9, false, "pid_steer"); // Setup for twiddle, 2nd round, and result Master-twiddle, 1st round,  maxcte=2.5, throttle=3.0 => max_speed ~30MPH (err=0.441286)
  //pid_steer.Init(0.153144, 0.0, 2.9, true, "pid_steer"); // Setup for twiddle, 3rd round
  pid_steer.Init(0.153144, 0.0, 6.95, false, "pid_steer"); // Result: Derived Master-twiddle, 3rd round, maxcte=4.0 => max_speed ~51MPH (TotErrP1), max_speed ~70MPH (TotErrP2), max_speed ~80MPH (TotErrP3)
 
  //pid_throttle.Init(0.0, 0.0, 0.0, false, "pid_throttle"); // Setup for twiddle, 1st round
  //pid_throttle.Init(0.0, 0.0, 0.0, true, "pid_throttle"); // Setup for twiddle, 2nd round and 3rd round
  // *** Results ***
  pid_throttle.Init(1.0, 0.0, 0.0, false, "pid_throttle"); // Result: Master-twiddle, maxcte=2.5

  h.onMessage([&pid_steer, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          //double speed = std::stod(j[1]["speed"].get<std::string>());
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
	  double throttle;

	  // *** Steering ***
	  pid_steer.UpdateError(cte);
	  steer_value = tanh(pid_steer.TotalError());

	  // *** Throttle ***
	  // * Twiddling
	  pid_throttle.UpdateError(fabs(steer_value)); 
	  //throttle = 0.5 + 0.5*tanh(pid_throttle.TotalError()); // TotErrP1 [For "Mom Mia", fast (~51MPH) but very safe]
	  //throttle = 0.65 + 0.35*tanh(pid_throttle.TotalError()); // For Videos
	  throttle = 0.7 + 0.3*tanh(pid_throttle.TotalError()); // TotErrP2 [For "Dad Brian" (RiP), faster (~70MPH), safe]
	  //throttle = 0.8 + 0.2*tanh(pid_throttle.TotalError()); // TotErrP3 [For "Don", very fast (~80MPH) but only safe for a few laps with differing (non-reproducable) number of laps finalized, something for a real stuntman, a rocket's drive, seems to depend to uWebSocket communication]
	  // * Constant values
	  //throttle = 0.3; // Choose for twiddle, 1st round
	  
	  
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          if (pid_steer.reset_run || pid_throttle.reset_run) {
	    pid_steer.reset_run = false;
	    pid_throttle.reset_run = false;
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
	  } else {
	    json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
	    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
