#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <algorithm>
#include <thread>
#include <fstream>

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}
double deg2rad(double x) {
  return x * pi() / 180;
}
double rad2deg(double x) {
  return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

/***
 * help function to parse command line option
 */
string getCmdOption(char** begin, char** end, const string &option) {
  char ** itr = std::find(begin, end, option);
  if (itr != end && ++itr != end)
    return string(*itr);
  return string();
}
/***
 * help function to parse command line option
 */
bool cmdOptionExists(char** begin, char** end, const string &optoin) {
  return std::find(begin, end, optoin) != end;
}

/***
 * thread function for manual tune pid value,
 * param: pid_y the instance of pid for y position
 * param: pid_s the isntance of pid for speed
 * param: h the socket to send reset command to simulator
 */
void tunePIDInteractiveFunc(PID& pid_y, PID& pid_s,
                            uWS::WebSocket<uWS::SERVER> &ws);

//target speed, sharing between PID and tuning thread
//TODO: add synchronization mechanism for multi-thread access.
static double target_speed = 30;

int main(int argc, char * argv[]) {

  /***
   * Here for initialization
   */

  //pid parameters for position y
  double Kp_y, Ki_y, Kd_y;
  //values from manual tune
  Kp_y = 0.16;
  Ki_y = 0.005;
  Kd_y = 4;
  //pid parameters for speed
  double Kp_s, Ki_s, Kd_s;
  Kp_s = 0.9;
  Ki_s = 0.0002;
  Kd_s = 1.1;

  //tune method
  enum Tune {
    NONE,
    AUTO,
    MANUAL
  } tune_method;

  tune_method = MANUAL;
  //output file name for errors
  string output_file = "";
  /**
   * CommandLine parsing
   */
  if (cmdOptionExists(argv, argv + argc, "-h")) {
    cout << "usage " << endl;
    cout
        << "  Option: --init \"Kp_position_y Ki_position_y Kd_position_y Kp_speed Ki_speed Kd_speed\""
        << endl;
    cout << "  Option: --target_speed [speed]  default:100" << endl;
    cout << "  Option: -o err_output_file_name default \"error.txt\"" << endl;
    cout
        << "  Option: --tune  [none|auto|manual tune PID parameters, default none"
        << endl;
    cout << "  Option: -h this message" << endl;
    return 0;
  }
  string init_parameters = getCmdOption(argv, argv + argc, "--init");
  if (!init_parameters.empty()) {
    stringstream ss(init_parameters);
    ss >> Kp_y >> Ki_y >> Kd_y >> Kp_s >> Ki_s >> Kd_s;
  }
  if (!getCmdOption(argv, argv + argc, "-o").empty()) {
    output_file = getCmdOption(argv, argv + argc, "-o");
  }

  string speed = getCmdOption(argv, argv + argc, "--target_speed");
  if (!speed.empty()) {
    stringstream ss(speed);
    ss >> target_speed;
  }

  string tune = getCmdOption(argv, argv + argc, "--tune");
  if (tune == "auto") {
    tune_method = AUTO;
  } else if (tune == "none") {
    tune_method = NONE;
  } else {
    tune_method = MANUAL;
  }

  cout << "Intial PID for Y Position:" << Kp_y << " " << Ki_y << " " << Kd_y
       << endl;
  cout << "Intial PID for      Speed:" << Kp_s << " " << Ki_s << " " << Kd_s
       << endl;
  cout << "Target Speed:" << target_speed << endl;
  cout << "Tune method:" << (tune_method == MANUAL ? "MANUAL" : tune) << endl;

  uWS::Hub h;
  uWS::WebSocket<uWS::SERVER> connected_ws;
  thread tune_t;

  // PID for y position, cte to lane center
  PID pid_y;
  // PID for speed control
  PID pid_s;
  // Initialize the pid variable.
  pid_y.Init(Kp_y, Ki_y, Kd_y);

  pid_s.Init(Kp_s, Ki_s, Kd_s);

  //open a file to write cte error, for external tool to analysis, e.g. GNUPLOT
  ofstream log;
  if (!output_file.empty()) {
    log.open(output_file);
  }

  h.onMessage(
      [&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
              double throttle_value;
              /*
               * TODO: Calcuate steering value here, remember the steering value is
               * [-1, 1].
               * NOTE: Feel free to play around with the throttle and speed. Maybe use
               * another PID controller to control the speed!
               */

              //PID is working at different mode, with/with_out tuning
              switch(tune_method) {
                case NONE: {
                  //without tuning, the PID is not adjusted during runtime.
                  //It will fail when Motion Model/environment changes
                  double sample_time = speed/20;
                  steer_value = pid_y.getControlValue(cte * speed/20, -1, 1,sample_time);
                  //slow down the speed for big error, e.g. at curve
                  double mse = pid_y.TotalError();
                  target_speed = 10 + 80 * 0.4/(mse +0.4);
                  throttle_value = pid_s.getControlValue(speed - target_speed, -1, 1);
                  // DEBUG
                  cout << "CTE: " << cte << " Steering Value: " << steer_value << " target Speed:" << target_speed<< std::endl;
                  break;
                }

                case MANUAL: {
                  //with manual tuning by observing the error in an output file
                  //adjusting the PID through console
                  steer_value = pid_y.getControlValue(cte , -1, 1);
                  throttle_value = pid_s.getControlValue(speed - target_speed, -1, 1);
                  if(log.is_open()) {
                    log << cte << " "<< speed - target_speed<<" "<<steer_value <<" "<<throttle_value<<endl;
                    log.flush();
                  }
                  break;
                }
                case AUTO: {
                  // a PID tune method use gradient descent, does not work well yet
                  //TODO: shall be improved.
                  pid_y.GDAutoTune(cte);
                  steer_value = pid_y.getControlValue(cte , -1, 1);
                  throttle_value = pid_s.getControlValue(speed - target_speed, -1, 1);
                  break;
                }
                default:
                cout << "No tune method specified ,error operation!!!"<<endl;
                break;
              }

              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle_value;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              //std::cout << msg << std::endl;
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
  h.onHttpRequest(
      [](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onConnection([&](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    //the websocket is shared to tune thread in order to reset the simulator
      connected_ws = ws;
      //after connect the pids values are reset.
      pid_y.Reset();
      pid_s.Reset();
    });

  h.onDisconnection(
      [&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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

  //in case tune, start a tune thread
  switch (tune_method) {
    case AUTO:
    case MANUAL: {
      tune_t = thread(tunePIDInteractiveFunc, ref(pid_y), ref(pid_s),
                      ref(connected_ws));
      cout << "start  Tune thread..." << endl;
      break;
    }
    default:
      break;
  }

  h.run();

  //join the tuning thread
  if (tune_t.joinable()) {
    tune_t.join();
  }

  //close log files. The log output is read by gnuplot for analysis.
  if (log.is_open()) {
    log.flush();
    log.close();
  }
}

void tunePIDInteractiveFunc(PID& pid_y, PID& pid_s,
                            uWS::WebSocket<uWS::SERVER> &ws) {
  char cmd;
  bool run = true;
  while (run) {
    cout
        << "Command(R=reset;Y= set PID for Y; S= Set PID for speed;T=set Speed;V=vbose;A=set alpha;X=exit):"
        << endl;
    cin >> cmd;
    switch (cmd) {
      case 'R': {
        string msg = "42[\"reset\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        pid_y.Reset();
        pid_s.Reset();
        break;
      }
      case 'Y': {
        cout << "values:";
        double Kp, Ki, Kd;
        cin >> Kp >> Ki >> Kd;
        pid_y.Init(Kp, Ki, Kd);
        break;
      }
      case 'S': {
        cout << "values:";
        double Kp, Ki, Kd;
        cin >> Kp >> Ki >> Kd;
        pid_s.Init(Kp, Ki, Kd);
        break;
      }
      case 'T': {
        cout << "Speed:";
        cin >> target_speed;
        cout << "Target Speed is:" << target_speed << endl;
        break;
      }
      case 'X': {
        run = false;
        cout << "Tuning PID exits" << endl;
        break;
      }

      case 'V': {
        pid_y.toggle_vbose();
        break;
      }

      case 'A': {
        cout << "Alpha:";
        double alpha;
        cin >> alpha;
        pid_y.setLearningRate(alpha);
        cout << "Alpha is:" << alpha << endl;
        break;
      }
      default: {
        cout << "!!!Unknown command:" << cmd << endl;
      }
    }
  }
}
