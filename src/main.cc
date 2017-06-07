//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   26.05.2017
//------------------------------------------------------------------------------

#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <stdexcept>
#include <sstream>
#include <thread>
#include <json.hpp>

#include "utils.hh"
#include "poly.hh"
#include "mpc.hh"

using json = nlohmann::json;

//------------------------------------------------------------------------------
// Message processor
//------------------------------------------------------------------------------
class MessageHandler {
  public:
    //--------------------------------------------------------------------------
    //! Constructor
    //--------------------------------------------------------------------------
    MessageHandler(): mpc_(100, 10, 0.05, lf_) {}

   //---------------------------------------------------------------------------
   //! Message processor
   //---------------------------------------------------------------------------
   void operator() (uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
                    uWS::OpCode opCode) {

    //--------------------------------------------------------------------------
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message, the 2 signifies a websocket event.
    //--------------------------------------------------------------------------
    if(length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = HasData(std::string(data).substr(0, length));

      //------------------------------------------------------------------------
      // Autonomous driving
      //------------------------------------------------------------------------
      if(s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          //--------------------------------------------------------------------
          // Convert the waypoints to the car coordinate system
          //--------------------------------------------------------------------
          std::vector<double> waypoints_x_g = j[1]["ptsx"];
          std::vector<double> waypoints_y_g = j[1]["ptsy"];

          double x_g   = j[1]["x"];
          double y_g   = j[1]["y"];
          double psi_g = j[1]["psi"];
          double x_l   = 0;
          double y_l   = 0;
          double psi_l = 0;
          double v     = j[1]["speed"];

          //v *= 0.44704; // convert to meters per second

          std::vector<double> waypoints_x_l;
          std::vector<double> waypoints_y_l;

          ConvertWaypoints(waypoints_x_l, waypoints_y_l,
                           waypoints_x_g, waypoints_y_g,
                           x_g, y_g, psi_g);

          //--------------------------------------------------------------------
          // Fit a polynomial
          //--------------------------------------------------------------------
          Eigen::Map<Eigen::VectorXd> wx(waypoints_x_l.data(), waypoints_x_l.size());
          Eigen::Map<Eigen::VectorXd> wy(waypoints_y_l.data(), waypoints_y_l.size());
          auto trajectory = PolyFit(wx, wy, 3);
          double cte  = PolyEval(trajectory, x_l);
          double epsi = -atan(trajectory[1]);

          //--------------------------------------------------------------------
          // Take the delay into account. Since the simulator only very loosely
          // adheres to the laws of physics and the units are messed up anyways,
          // we can assume that 0.065 is a good approximation of 100ms. It seems
          // to wrok well enough anyways.
          //--------------------------------------------------------------------
          double delay = 0.065;

          x_l   += v * cos(psi_l) * delay;
          y_l   += v * sin(psi_l) * delay;
          psi_l += v * delta_prev_ / lf_ * delay;
          cte   += v * sin(epsi) * delay;
          epsi  += v * delta_prev_ / lf_ * delay;
          v     += a_prev_ * delay;

          //--------------------------------------------------------------------
          // Compute the steering angle and the throttle
          //--------------------------------------------------------------------
          Eigen::VectorXd state(6);
          state << x_l, y_l, psi_l, v, cte, epsi;

          auto result = mpc_.Solve(state, trajectory);
          double steer_value    = -result.delta / Deg2Rad(25);
          double throttle_value = result.a;
          std::cout << "throttle: " << throttle_value << ", ";
          std::cout << "steering angle: " << steer_value << std::endl;
          a_prev_ = result.a;
          delta_prev_ = result.delta;

          //--------------------------------------------------------------------
          // Send the message
          //--------------------------------------------------------------------
          json msgJson;

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"]       = throttle_value;
          msgJson["mpc_x"]          = std::vector<double>{};//result.xs;
          msgJson["mpc_y"]          = std::vector<double>{};//result.ys;
          msgJson["next_x"]         = std::vector<double>{};//waypoints_x_l;
          msgJson["next_y"]         = std::vector<double>{};//Waypoints_y_l;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      //------------------------------------------------------------------------
      // Manual driving
      //------------------------------------------------------------------------
      else {
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  }

  private:
    //--------------------------------------------------------------------------
    // Checks if the websock event has JSON data. If there is data the JSON
    // object in string format will be returned, else the empty string "" will
    // be returned.
    //--------------------------------------------------------------------------
    std::string HasData(const std::string &s) {
      auto found_null = s.find("null");
      auto b1         = s.find_first_of("[");
      auto b2         = s.find_last_of("]");
      if (found_null != std::string::npos)
        return "";
      else if (b1 != std::string::npos && b2 != std::string::npos)
        return s.substr(b1, b2 - b1 + 1);
      return "";
    }

    double         lf_         = 2.6;
    MPC            mpc_;
    double         a_prev_     = 0;
    double         delta_prev_ = 0;
};

//------------------------------------------------------------------------------
// Start the show
//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  uWS::Hub h;

  //----------------------------------------------------------------------------
  // Install callbacks
  //----------------------------------------------------------------------------
  MessageHandler msg_handler;
  h.onMessage(
    [&msg_handler]
    (uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
      uWS::OpCode opCode) {
      msg_handler(ws, data, length, opCode);
    });
  h.onConnection(
    [&msg_handler]
    (uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
      std::cout << "Connected" << std::endl;
    });

  h.onDisconnection(
    [&msg_handler]
    (uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
      std::cout << "Disconnected" << std::endl;
    });


  //----------------------------------------------------------------------------
  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  //----------------------------------------------------------------------------
  h.onHttpRequest(
    [](uWS::HttpResponse *res, uWS::HttpRequest req, char *, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
      res->end(s.data(), s.length());
    else
      res->end(nullptr, 0);
    });

  //----------------------------------------------------------------------------
  // Run the server
  //----------------------------------------------------------------------------
  int port = 4567;
  if(h.listen(port))
    std::cout << "Listening to port " << port << std::endl;
  else {
    std::cerr << "Failed to listen to port: " << port << std::endl;
    return -1;
  }
  h.run();
}
