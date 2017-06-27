#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <algorithm>
#include <thread>
#include <fstream>
#include <chrono>

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

/***
 * help function to parse command line option
 */
string getCmdOption(char** begin, char** end, const string &option)
{
	char ** itr= std::find(begin,end,option);
	if(itr !=end && ++itr!= end) return string(*itr);
	return string();
}
bool cmdOptionExists(char** begin, char** end, const string &optoin){
	return std::find(begin,end,optoin)!=end;
}

/***
 * thread function for manual tune pid value,
 * param: pid_y the instance of pid for y position
 * param: pid_s the isntance of pid for speed
 * param: h the socket to send reset command to simulator
 */
void manualTunePIDFunc(PID& pid_y, PID& pid_s, uWS::WebSocket<uWS::SERVER> &ws);

int main(int argc, char * argv[])
{
	//pid parameters for position y
	double Kp_y(0),Ki_y(0),Kd_y(0);
	//pid parameters for speed
	double Kp_s(0),Ki_s(0),Kd_s(0);
	//target speed
	double target_speed = 100;
	//tune method
	bool manual = true;
	//output file name for errors
	string output_file="";
	/**
	 * do commandline parsing
	 */
	if(cmdOptionExists(argv,argv+argc,"-h")){
		cout<< "usage "<<endl;
		cout<< "  Option: --init \"Kp_position_y Ki_position_y Kd_position_y Kp_speed Ki_speed Kd_speed\""<<endl;
		cout<< "  Option: --target_speed [speed]  default:100"<<endl;
		cout<< "  Option: -o err_output_file_name default \"error.txt\""<<endl;
		cout<< "  Option: --auto  auto tune PID parameters" << endl;
		cout<< "  Option: --manual manual tune PID parameters" <<endl;
		cout<< "  Option: -h this message"<<endl;
		return 0;
	}
	string init_parameters = getCmdOption(argv,argv+argc,"--init");
	if(!init_parameters.empty()){
		stringstream ss(init_parameters);
		ss>>Kp_y>>Ki_y>>Kd_y>>Kp_s>>Ki_s>>Kd_s;
	}
	if(!getCmdOption(argv,argv+argc,"-o").empty()){
		output_file = getCmdOption(argv,argv+argc,"-o");
	}

	string speed = getCmdOption(argv, argv + argc, "--target_speed");
	if (!speed.empty()) {
		stringstream ss(speed);
		ss >> target_speed;
	}

	manual = !cmdOptionExists(argv,argv+argc,"--auto");

	cout << "Intial PID for Y Position:" <<Kp_y<<" "<<Ki_y<<" "<<Kd_y<<endl;
	cout << "Intial PID for      Speed:" <<Kp_s<<" "<<Ki_s<<" "<<Kd_s<<endl;
	cout << "Target Speed:" << target_speed <<endl;
	cout << "Tune method:"<< (manual?"manual":"auto")<<endl;


  uWS::Hub h;
  uWS::WebSocket<uWS::SERVER> connected_ws;
  thread tune_t;

  PID pid_y;
  PID pid_s;
  // TODO: Initialize the pid variable.
  pid_y.Init(Kp_y,Ki_y,Kd_y);
  //pid_speed.Init(5.78815/10,0,0);
  pid_s.Init(Kp_s,Ki_s,Kd_s);

  //open a file to write cte error
  ofstream log;
  if(!output_file.empty()){
	  log.open(output_file);
  }
  auto begin = chrono::system_clock::now();

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          steer_value =  pid_y.getControlValue(cte, -1, 1);
          throttle_value = pid_s.getControlValue(speed - target_speed, -1, 1);
          if(log.is_open()){
        	  chrono::milliseconds ms = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - begin);

        	  log << ms.count()<<" "<< cte << " "<< speed - target_speed<<" "<<steer_value <<" "<<throttle_value<<endl;
        	  log.flush();
          }
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
		  //std::cout << "CTE- Speed: " << speed -10 <<std::endl;
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

  h.onConnection([&](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    connected_ws = ws;
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
  else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
  }

	//start a thread for processing manual tune
	if (manual) {

		tune_t = thread(manualTunePIDFunc, ref(pid_y), ref(pid_s),
				ref(connected_ws));
		cout << "start Manual Tune thread..." << endl;
	}
  h.run();
  if(tune_t.joinable()){
	  tune_t.join();
  }
  log.close();

}

void manualTunePIDFunc(PID& pid_y, PID& pid_s, uWS::WebSocket<uWS::SERVER> &ws){
	char cmd;
	bool run=true;
	while(run){
		cout<<"Command(R=reset simulator;Y= pid values for Y; S= pid for speed;T=exit):"<<endl;
		cin >> cmd;
		switch(cmd){
		case 'R':{
			string msg = "42[\"reset\",{}]";
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			break;
		}
		case 'Y':{
			cout << "values:";
			double Kp, Ki, Kd;
			cin >> Kp >> Ki >> Kd;
			pid_y.Init(Kp, Ki, Kd);
			break;
		}
		case 'S':{
			cout << "values:";
			double Kp, Ki, Kd;
			cin >> Kp >> Ki >> Kd;
			pid_s.Init(Kp, Ki, Kd);
			break;
		}
		case 'T':{
			run = false;
			cout << "Tuning PID exits"<<endl;
			break;
		}
		default:{
			cout<< "!!!Unknown command:" <<cmd<<endl;
		}
		}
	}
}
