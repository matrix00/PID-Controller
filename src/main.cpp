#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

PID pidSteering;
PID pidThrottle;
double maxSpeed=0.5;

double max(double a, double b){return a>b?a:b;}

double min(double a, double  b){return a<b?a:b;}

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

void Twiddle(PID& pid)
{

	double p[] = {pid.Kp, pid.Kd, pid.Ki};
	double dp[] = {1, 1, 1};
  double best_err = pid.p_error;
	int it=0;

	while ((dp[0]+dp[1]+dp[2]) > 0.0001)
	{
		for (int i=0; i < 3; i++)
		{
			p[i] += dp[i];
			// need to do something x_trajectory, y_trajectory, err = run(robot, p)

			double err = 1;// need to find out
			if (err < best_err)
     	{
				best_err = err;
				dp[i] *= 1.1;
			}
			else
			{
				p[i] -= 2*dp[i];
				//x_trajectory, y_trajectory, err = run(robot, p)
        if (err < best_err)
				{
					best_err = err;
					dp[i] *= 1.1;
				}
				else
				{
					p[i] += dp[i];
					dp[i] *= 0.9;
				}
			}
			it +=1;
		}
		pid.Kp = p[0];
		pid.Kd = p[1];
		pid.Ki = p[2]; 
	}
}

int main()
{
  uWS::Hub h;

  // TODO: Initialize the pid variable.

	pidSteering.Init(0.2, 0.004, 3.0);
	//pidSteering.Init(0.1, 0.001, 2.8);
	pidThrottle.Init(0.4, 0.0001, 0.6);
	//pid.Init(2.9, 0.49, 10.3);


  h.onMessage([&pidSteering](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;


          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
					pidSteering.UpdateError(cte);
					steer_value = -pidSteering.Kp * cte - pidSteering.Kd * pidSteering.d_error - pidSteering.Ki * pidSteering.i_error;

					//if (steer_value > 1 || steer_value < -1) //invalid values
					//	steer_value = angle; //keep previous

					
					pidThrottle.UpdateError(cte);
					double throttle= min(maxSpeed -pidThrottle.Kp * cte - pidThrottle.Kd * pidThrottle.d_error - pidThrottle.Ki * pidThrottle.i_error,maxSpeed);

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " d error " << pidSteering.d_error << " i error " << pidSteering.i_error << " Throttle " << throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;

          //msgJson["throttle"] = 0.3;
          msgJson["throttle"] = throttle;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
