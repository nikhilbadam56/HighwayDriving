#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include <map>
#include "spline.h"
#include <sstream>
#include <utility>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int get_lane(double d)
{
  /*
  @params d : frenet d coordinate of the vehicle 
  return 
  respective lane of the car
   0<=d<4 lane 0
   4<=d<8 lane 1
   8<=d<12 lane 2
  */
  int car_lane = -1;
   if(d > 0 && d < 4)
   {
     car_lane = 0;
   }else if(d > 4 && d < 8)
   {
     car_lane = 1;
   }else if(d > 8 && d < 12)
   {
     car_lane = 2;
   }
  return car_lane;
}
int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  string file = "../data/highway_map.csv";
  string line;
  std::ifstream in_map_(file);
  if (in_map_.is_open())
  {
	while (getline(in_map_,line))
    {
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
    in_map_.close();
  }

  else std::cout << "Unable to open file"<<std::endl; 
  //initial lane center lane
  int lane = 1;
  double ref_vel = 0; //initial velocity (must be 50MPH as per the norms)
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel]
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
          double preferred_buffer  = 25.0;
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          int prev_size = previous_path_x.size();
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          bool car_front = false;
          bool car_left = false;
          bool car_right = false;
          

          
          //detecting whether there are cars around the car present lane based on the sensor fusion data received
          for(int i =0;i<sensor_fusion.size();i++)
          {
            //get the current neighbouring car s , d , present lane 
            double nei_car_s = sensor_fusion[i][5];
            double nei_car_d = sensor_fusion[i][6];
            int nei_car_lane  = get_lane(nei_car_d);
             if(nei_car_lane < 0)
             {
               continue;
             }
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double nei_car_speed = sqrt(vx*vx + vy*vy);
            /*check whether the neighbouring car is 
            1.Too close(based on the preferred buffer distance between the two)
            2.In the relative left lane 
            3.In the relative right lane 
            */
            
            nei_car_s+=((double)nei_car_speed * 0.02 * prev_size); //observed car might have travelled further within the tema that we have received the measurement .
            if(nei_car_lane == lane)
            {
              //car in the same lane as that of the neighbouring car
              if (nei_car_s>car_s)
              {
                //making sure that we are not calculating teh closeness to the previous car .
                if(nei_car_s - car_s <=preferred_buffer)
                {
                  //infront car is too close to the ego car
                  car_front = true;
                }
              }
            }
            else
            {
              //if car is not in the same lane
              if(nei_car_lane  == lane-1)
              {
                /*
                inorder to flag a neighbouring car present to the left of the car it has to satisfy the following condition
                the neighbouring car's s coordinates must be within the ego car +/- preferred buffer
                if not this means that we have enough space to make a transition to the left.
                	"car_s-preferred_buffer < nei_car_s < car_s+preferred_buffer"
                */
                //neighbouring car is left to the current car
                if(car_s-preferred_buffer<nei_car_s && car_s+preferred_buffer>nei_car_s)
                {
                  //this means that there is clearly a car and we cannot make a transition to the left.
                	car_left = true;
                }
              }else if(nei_car_lane == lane+1)
              {
                 /*
                inorder to flag a neighbouring car present to the left of the car it has to satisfy the following condition
                the neighbouring car's s coordinates must be within the ego car +/- preferred buffer
                if not this means that we have enough space to make a transition to the left.
                	"car_s-preferred_buffer < nei_car_s < car_s+preferred_buffer"
                */
                //neighbouring car is right to the current car
                if(car_s-preferred_buffer<nei_car_s && car_s+preferred_buffer>nei_car_s)
                {
                  //this means that there is clearly a car and we cannot make a transition to the left.
                	car_right = true;
                }
              }
            }
            
          }

          /*we now have enough knowledge on the cars around us , we plan the behaviour
          behaviour include :
          1. if there is a car infront and too close then we have to change the lanes either to the left or right based on avaialability , when none is availabe we decrese our velocity inorder not to collide;
          2. if ther is no car infront and too far away then, we can maintain the lane speeding up or for preference we can change to the center lane based on the avialability
          */
          
          if(car_front)
          {
            if(!car_left && lane>0)
            {
              //there is a space to the left and we are not crossing the highway yellow line
              lane-=1;
            }else if(!car_right && lane < 2)
            {
              //there is a space to the right and we are not crossing the highway boundaries
              lane+=1;
            } 
                
            std::cout<<"in changing acceleration slowly"<<std::endl;
            ref_vel-=0.3584;
          }
          else
          {
            
            if(lane !=1)
            {
              //car not in the middle lane , 
              if((lane  == 0 && !car_right) || (lane  == 2 && !car_left))
              {
                //then we can change to left lane that is lane-=1
                lane = 1;
              }
            }
            if(ref_vel < 49.5)
            {
              ref_vel+=0.3584; //acelerating if the reference velocity is less that some velocity ,,since velocity can read 49.5+/-5.00

            }
          }
          /*
          We have the correct lane that we want to travel to , then we have to generate the jerk minimization trajectory by using some way points in the middle , this way point involves some way points from the previous path , this ensures for a smooth transition between the paths .
          */
          
          //variables to store the sample points to define a spline
          vector<double> point_x;
          vector<double> point_y;

          
          double reference_yaw = deg2rad(car_yaw);
          double reference_x = car_x;
          double reference_y = car_y;
          //including two previous points for smooth tansition

          if(!(prev_size < 2))
          {
            // we have enough points to define a yaw according to the slope of the points
            
            reference_x = previous_path_x[prev_size - 1];
            reference_y = previous_path_y[prev_size - 1];
            double pre_prev_x = previous_path_x[prev_size - 2];
            double pre_prev_y = previous_path_y[prev_size - 2];
			//getting the reference yaw
            reference_yaw = atan2(reference_y - pre_prev_y, reference_x - pre_prev_x);

            point_x.push_back(pre_prev_x);
            point_x.push_back(reference_x);

            point_y.push_back(pre_prev_y);
            point_y.push_back(reference_y);
          }
          else
          {
            //there are not enough previous path points to include in the points
            //so we extrapolate backwards based on the car present yaw
            double poin_x = car_x - cos(car_yaw);
            double poin_y = car_y - sin(car_yaw);
            point_x.push_back(poin_x);
            point_x.push_back(car_x);

            point_y.push_back(poin_y);
            point_y.push_back(car_y);
             
          }

          //next we have to generate some more points into future using the lane changed and also the 30 gapped s values
          for(int i =30;i<=90;i+=30)
          {
            vector<double> wp = getXY(car_s+i, 2+4*lane, map_waypoints_s,map_waypoints_x,map_waypoints_y);
          point_x.push_back(wp[0]);

          point_y.push_back(wp[1]);
          }
          
          /*now we got the way points(discretizing the trajectory) in map space ...now have to transform them into car reference frame for easy calculation of the spline points before we transform them again in mapspace for plotting in the simulation*/
           
          for(int i =0;i<point_x.size();i++)
           {
			 double transl_x = point_x[i] - reference_x;
             double transl_y = point_y[i] - reference_y;
             point_x[i] = transl_x*cos(0-reference_yaw) - transl_y*sin(0-reference_yaw);
             point_y[i] = transl_x*sin(0-reference_yaw) + transl_y*cos(0-reference_yaw);
           }
           
           //now we got the transformed coordinates of the waypoints on the trajectory
           //now inorder to make them match the speed specification given that is 50MPH ..
           //we need to get the sampled points on interpolated polynomial at intervals
           //equal to total_end_dist/(0.02*target_speed(in m/s ..divide by 2.24))
           
           //first keep the previous path points for smooth transition
          for(int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
           
           //interpolating the waypoints using cubic spline technique
           // it is under tk namespace in spline.h header file
           tk::spline spl;
           spl.set_points(point_x,point_y);
           
           //now we have to sample the interpolated polynomial and include enough of them so that we have 50 points.
           double x = 30;
           double y = spl(x);
           double dist = sqrt(x*x+y*y); //becuase we have shifted the points to make the car position as origin 
           double dummy_x = 0;
          double N = dist / (0.02 * ref_vel/2.24);
           for(int i=0;i<50 -prev_size;i++)
           {
            double x_point = dummy_x + x / N;
            double y_point = spl(x_point);

            dummy_x = x_point;

            double x_dum = x_point;
            double y_dum = y_point;

            // Rotating back to normal after rotating it earlier.
            x_point = x_dum * cos(reference_yaw) - y_dum * sin(reference_yaw);
            y_point = x_dum * sin(reference_yaw) + y_dum * cos(reference_yaw);

            x_point += reference_x;
            y_point += reference_y;   
             next_x_vals.push_back(x_point);
             next_y_vals.push_back(y_point);
           }
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