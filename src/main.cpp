#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#define TWIDDLE 0

// for convenience
using json = nlohmann::json;
using namespace std;

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

int main()
{
  uWS::Hub h;

  // TODO: Initialize the pid variable.
  PID steerpid;
  steerpid.Init(0.075 , 0.0004, 2, deg2rad(2));//(0.075, 0.004, 2)
  PID speedpid;
  speedpid.Init(.5 , .0005 , 0.0, 0);
  double counts = 0;

  h.onMessage([&steerpid, &speedpid, &counts](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = stod(j[1]["cte"].get<string>());
          double speed = stod(j[1]["speed"].get<string>());
          double angle = stod(j[1]["steering_angle"].get<string>());
          double throttle = stod(j[1]["throttle"].get<string>());
          if (0) {
            cout << "[" << counts << "] ";
            cout << " cte: " << cte;
            cout << " speed: " << speed;
            cout << " angle: " << angle;
            cout << " throttle: " << throttle;
            cout << endl;
          }
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          steerpid.UpdateError(cte);
          double steer_value = steerpid.GetSteerValue();
          double turn = steerpid.GetCTERate();
          double tspeed = 40. * (1 - abs(steer_value)) + 20.;
          speedpid.UpdateError(speed - tspeed);
          double speed_value = speedpid.GetSpeedValue();
          
          // DEBUG
          if (1) {
            cout << "[" << counts << "] ";
            cout << " cte: " << cte;
            cout << " tse: " << speed -  tspeed;
            cout << " speed_value: " << speed_value;
            cout << " steer_value: " << steer_value;
            cout << endl;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = speed_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //cout << msg << endl;
          if ((steerpid.numSteps > 2*N) && TWIDDLE) {
            steerpid.twiddle();
            steerpid.numSteps = 0;
          }
          if ((speedpid.numSteps > 2*N) && TWIDDLE) {
            speedpid.twiddle();
            speedpid.numSteps = 0;
          }
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        if ((steerpid.numSteps > 2*N) && TWIDDLE) {
            steerpid.twiddle();
            steerpid.numSteps = 0;
        }
        if ((speedpid.numSteps > 2*N) && TWIDDLE) {
            speedpid.twiddle();
            speedpid.numSteps = 0;
        }
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
    counts++;
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const string s = "<h1>Hello world!</h1>";
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
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    cout << "Listening to port " << port << endl;
  }
  else
  {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}
