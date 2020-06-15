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
using std::string;
using std::vector;

// define all constants 
const double MAX_VELOCITY = 49.5;  // Maxminum velocity
const double MAX_ACCELARATION = 0.224; // Maximum Accelaration 
const double MAX_DECELERATION = 0.448; //Maximum Deceleration 

const int LEFT_LANE = 0;  // Left Lane 
const int MIDDLE_LANE = 1; // Ceter Lane
const int RIGHT_LANE = 2;  // Right Lane 
const int INVALID_LANE = -1;  // Invalid lane (Opposite lanes or out of the lanes)

const int LEFT_LANE_MAX = 4;   // Maximum distance from right 4 M *1-lane
const int MIDDLE_LANE_MAX = 8; // Maximum distance from right 4 M *2-lane
const int RIGHT_LANE_MAX = 12; // Maximum distance from right 4 M *3-lane

const int PROJECTION_IN_METERS = 30;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "/home/charan/Udacity/project_7/CarND-Path-Planning-Project/data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
    // Start in lane 1 

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

  int lane = 1;
  
  // Move a reference velocity of target 
  double ref_velocity = 0.0; //MPH
 
    bool car_ahead = false;
    bool car_right = false; 
    bool car_left = false;
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_velocity,&car_ahead,&car_left,&car_right]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
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
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Get the previous path size 
          int prev_size = previous_path_x.size();
          
          // if previous path size is not starting poistion then assign car end position will be the previous position 
          if (prev_size>0)
          {
            car_s = end_path_s;
          }

          //  1. PREDICTION : Analysing other cars positions.
          car_ahead = false;
          car_right = false;
          car_left = false;
          bool too_close = false;

          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            // car is in my lane 
            float d = sensor_fusion[i][6];

            int other_car_lane = INVALID_LANE;
            //Determine the car other lane 
            // If the car is in Left lane 
            if(d > 0 && d < LEFT_LANE_MAX){
              other_car_lane = LEFT_LANE;
            }
            // If the car is in Center lane 
            else if(d > LEFT_LANE_MAX && d < MIDDLE_LANE_MAX){
              other_car_lane = MIDDLE_LANE;
            }
            // If the car is in Right lane 
            else if(d > MIDDLE_LANE_MAX && d < RIGHT_LANE_MAX){
              other_car_lane = RIGHT_LANE;
            }
            // If the car is in Invalid lane 
            if(other_car_lane==INVALID_LANE){
              continue;
            }

            //if(d < (2+4*lane+2) && d > (2+4*lane-2)){
              // velocity of x,y axies 
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              // calculate the speed of the car
              double check_speed = sqrt(vx*vx+vy*vy);
              // get the car s element 
              double check_car_s = sensor_fusion[i][5];

              check_car_s += ((double)prev_size*0.02*check_speed);
              if(other_car_lane == lane){
                 // Other car is in the same lane
                 car_ahead |= check_car_s >car_s && check_car_s - car_s < PROJECTION_IN_METERS;
              }
              else if(other_car_lane - lane == -1){
                // Other car is on the left lane
                car_left |= car_s - PROJECTION_IN_METERS < check_car_s && car_s + PROJECTION_IN_METERS > check_car_s;
              }
              else if(other_car_lane - lane == 1){
                 // Other car is on the right lane
                car_right |=  car_s - PROJECTION_IN_METERS < check_car_s && car_s + PROJECTION_IN_METERS > check_car_s;
              } 

               //ref_velocity = 29.5; //MPH
                // too_close = true;
                // if(lane >0){
                //   lane =0;
            //}
          }


          //  Behavior : 
          double speed_diff = 0;

          if(car_ahead){
            if(!car_left && lane >0){
                // if there is no car left and there is a left lane.
                lane--; // Change lane left.
            }
            else if(!car_right && lane != 2){
              // if there is no car right and there is a right lane.
              lane++; // Change lane right.
            }
            else {
              // If cars are there left and right lane slow down 
              speed_diff -=MAX_ACCELARATION; 
            }
          }else {
            if(lane != 1) // if we are not on the center lane.
            {
              if((lane ==0 && !car_right) || (lane ==2 && !car_left)){
                lane =1; // back to the center lane 
              }
            }
            if(ref_velocity < MAX_VELOCITY){ // If velocity is less than max velcity increase acc
              speed_diff += MAX_ACCELARATION; 
            }
          }

          // if(too_close){
          //     ref_velocity -=0.224;
          // }
          // else if(ref_velocity < 49.5){
          //   ref_velocity +=0.224;
          // }
          
          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x  = car_x;
          double ref_y  = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If previous size is almost empty ,use the car as starting point 

          if(prev_size <2)
          {
            //Use two points that make the path tangent to the car 
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous path end point as starting reference 
          else{
            //redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];

            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

            //use two point that make the path tangent to the previous path's end point 
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          vector<double> next_way_point0 = getXY(car_s+PROJECTION_IN_METERS,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_way_point1 = getXY(car_s+(2*PROJECTION_IN_METERS),(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_way_point2 = getXY(car_s+(3*PROJECTION_IN_METERS),(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(next_way_point0[0]);
          ptsx.push_back(next_way_point1[0]);
          ptsx.push_back(next_way_point2[0]);

          ptsy.push_back(next_way_point0[1]);
          ptsy.push_back(next_way_point1[1]);
          ptsy.push_back(next_way_point2[1]);

          for (int i = 0; i < ptsx.size(); i++)
          {
            // shift car reference angle to 0 degrees 
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

          }


          // Create a spline 
          tk::spline s;

          // set (x,y) points to the spline 
          s.set_points(ptsx,ptsy);

          
          // define actual (x,y) points. we will use for the planner 
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }


          //calculate how to break up spline points so that we travel at our desired reference velocity 
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          // fill up the rest of our path planner after filling with previous points. here we will always output is 50 points 
          for (int i = 0; i < 50 - previous_path_x.size(); i++)
          {
            ref_velocity +=speed_diff;
            if(ref_velocity > MAX_VELOCITY){
              ref_velocity = MAX_VELOCITY;
            } else if(ref_velocity < MAX_ACCELARATION){
               ref_velocity = MAX_ACCELARATION;
            }

            double N = (target_dist/(0.02*ref_velocity/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point =s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //rorate back to normal after rotating it to earlier 
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point +=ref_x;
            y_point +=ref_y;
          

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          
          }
          
          
          /**
           * Define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
             */

          // double dist_inc = 0.5;
          // for (int i = 0; i < 50; i++)
          // {
          //   double next_s = car_s+(i+1)*dist_inc;
          //   double next_d = 6;
          //   vector<double> xy = getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          //   next_x_vals.push_back(xy[0]);
          //   next_y_vals.push_back(xy[1]);
          // }
          

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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