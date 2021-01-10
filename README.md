# Highway Driving
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```
#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
#### Overview

“Highway Driving”, this project involves making a toy car navigate autonomously avoiding
potential obstacles, safely maneuvering around them avoiding collision, changing lanes
safely by following a quintic polynomial with optimal cost and that minimizes jerk. “Sensor
Fusion” data is provided from the simulator, predictions over the neighboring cars are
made, then the behavior is being calculated, then choosing the optimal path (that is to keep
the lane or to take left or to take right lanes) for a particular behavior.


### Goals

```
● Collision Free maneuvers.
● Jerk minimized Lane changing trajectories.
● Doesn’t exceed the maximum acceleration of 1g.
● Able to change lanes.
● Doesn’t exceed the speed limit.
```
## Brief

The code has to implement the following parts :

1. Predictions.
2. Behaviour Planning.
3. Trajectory Generation (using spline technique for numerical interpolation).

## Predictions:
Based on the sensor fusion optimal estimate data we get useful data about the
objects around us. For this, we get data about the neighboring cars around the ego vehicle,
this data includes the car’s id, x,y,s,d,vx,vy , representing identification number, map
position, fernet coordinates, velocity components respectively. Using the following code we
are able to find whether there are cars to the left, front, and right of the ego vehicle for
modeling the behavior (143 - 210 lines from the main.cpp code).
```
for​(​int​ i =​ 0 ​;i<sensor_fusion.size();i++)
{
​//get the current neighbouring car s , d , present lane
​double​ nei_car_s = sensor_fusion[i][​ 5 ​];
​double​ nei_car_d = sensor_fusion[i][​ 6 ​];
​int​ nei_car_lane = get_lane(nei_car_d);
​if​(nei_car_lane < ​ 0 ​)
{
​continue​;
}
​double​ vx = sensor_fusion[i][​ 3 ​];
​double​ vy = sensor_fusion[i][​ 4 ​];
​double​ nei_car_speed = ​sqrt​(vx*vx + vy*vy);


​/*check whether the neighbouring car is
1.Too close(based on the preferred buffer distance between the
two)
2.In the relative left lane
3.In the relative right lane
*/
nei_car_s+=((​double​)nei_car_speed * ​0.02​ * prev_size);
//observed car might have travelled further within the tema that we have
received the measurement.
​if​(nei_car_lane == lane)
{
​//car in the same lane as that of the neighbouring car
​if​ (nei_car_s>car_s)
{
​//making sure that we are not calculating teh closeness to
the previous car.
​if​(nei_car_s - car_s <=preferred_buffer)
{
​//infront car is too close to the ego car
car_front = ​true​;
}
}
}
​else
{
​//if car is not in the same lane
​if​(nei_car_lane == lane​-1​)
{
​/*
inorder to flag a neighbouring car present to the left of
the car it has to satisfy the following condition
the neighbouring car's s coordinates must be within the ego
car +/- preferred buffer
if not this means that we have enough space to make a
transition to the left.
"car_s-preferred_buffer < nei_car_s <
car_s+preferred_buffer"
*/
​//neighboring car is left to the current car
​if​(car_s-preferred_buffer<nei_car_s &&
car_s+preferred_buffer>nei_car_s)
{
​//this means that there is clearly a car and we cannot
make a transition to the left.
car_left = ​true​;
}
}​else​ ​if​(nei_car_lane == lane+​ 1 ​)
{
​/*
inorder to flag a neighbouring ​car ​present to the left of
the car it has to satisfy the following condition
the neighbouring car's s coordinates must be within the ego
car +/- preferred buffer
if not this means that we have enough space to make a
transition to the left.
"car_s-preferred_buffer < nei_car_s <
car_s+preferred_buffer"
*/
​//neighbouring car is right to the current car
​if​(car_s-preferred_buffer<nei_car_s &&
car_s+preferred_buffer>nei_car_s)
{
​//this means that there is clearly a car and we cannot
make a transition to the left.
car_right = ​true​;
}
}
}
}
```
## Behavioral Planning 
After the above predictions, we might want to accelerate or wait for lane change
either to the left or right lane. A programmed FSM‘s are being used for this. Following code
is used for behavior planning (lines 212 - 250 in main. cpp are used for this).
```
​/*we now have enough knowledge on the cars around us , we plan the behavior 
behavior include :

1. if there is a car infront and too close then we have to change
the lanes either to the left or right based on avaialability , when none is
availabe we decrese our velocity inorder not to collide;
2. if ther is no car infront and too far away then, we can maintain the lane speeding up or for preference we can change to the center
lane based on the avialability
*/
​if​(car_front)
{
​if​(!car_left && lane>​ 0 ​)
{
​//there is a space to the left and we are not crossing the
highway yellow line
lane-=​ 1 ​;
}​else​ ​if​(!car_right && lane < ​ 2 ​)
{
​//there is a space to the right and we are not crossing the
highway boundaries
lane+=​ 1 ​;
}
​std​::​cout​<<​"in changing acceleration slowly"​<<​std​::​endl​;
ref_vel-=​0.3584​;
}
​else
{
​if​(lane !=​ 1 ​)
{
​//car not in the middle lane ,
​if​((lane == ​ 0 ​ && !car_right) || (lane == ​ 2 ​ && !car_left))
{
​//then we can change to left lane that is lane-=
lane = ​ 1 ​;
}
}
if​(ref_vel < ​49.5​)
{
ref_vel+=​0.3584​; ​//acelerating if the reference velocity is
less that some velocity ,,since velocity can read 49.5+/-5.
}
}
```

## Trajectory Generation:

After the above behavior planning, we need to plan a perfect trajectory from the current
car position to the respective point in the same lane or in some other lane either to the left

or right, a perfect trajectory minimizes the jerk involved in lane changing, avoids collision,
smooth enough.
For a smooth transition function generation between the current following trajectory and
the trajectory we intended to follow, we include the sample points from the previous
trajectory and calculate the ego car yaw from these points. We generate a spline
interpolating the waypoints from the current car position (transformed map coordinates
from map reference frame to the car frame i.e., taking origin with respect to the car) to the
waypoints on the intended lane. In order to maintain a constant 50MPH speed along the
maneuver, we need to sample the points on the spline at a particular interval. We then
transform the points to the map coordinate space again, in order to make the ego car
follow this trajectory in the real simulation world.
Following code used for trajectory generation(lines from 255 - 356 main.cpp is used for
this task).
```
​//variables to store the sample points to define a spline
​vector​<​double​> point_x;
​vector​<​double​> point_y;
​double​ reference_yaw = deg2rad(car_yaw);
​double​ reference_x = car_x;
​double​ reference_y = car_y;
​//including two previous points for smooth tansition
​if​(!(prev_size < ​ 2 ​))
{
​// we have enough points to define a yaw according to the slope
of the points
reference_x = previous_path_x[prev_size - ​ 1 ​];
reference_y = previous_path_y[prev_size - ​ 1 ​];
​double​ pre_prev_x = previous_path_x[prev_size - ​ 2 ​];
​double​ pre_prev_y = previous_path_y[prev_size - ​ 2 ​];
//getting the reference yaw
reference_yaw = ​atan2​(reference_y - pre_prev_y, reference_x -
pre_prev_x);
point_x.push_back(pre_prev_x);
point_x.push_back(reference_x);
point_y.push_back(pre_prev_y);
point_y.push_back(reference_y);


}
​else
{
​//there are not enough previous path points to include in the
points
​//so we extrapolate backwards based on the car present yaw
​double​ poin_x = car_x - ​cos​(car_yaw);
​double​ poin_y = car_y - ​sin​(car_yaw);
point_x.push_back(poin_x);
point_x.push_back(car_x);
point_y.push_back(poin_y);
point_y.push_back(car_y);
}
​//next we have to generate some more points into future using the
lane changed and also the 30 gapped s values
​for​(​int​ i =​ 30 ​;i<=​ 90 ​;i+=​ 30 ​)
{
​vector​<​double​> wp = getXY(car_s+i, ​ 2 ​+​ 4 ​*lane,
map_waypoints_s,map_waypoints_x,map_waypoints_y);
point_x.push_back(wp[​ 0 ​]);
point_y.push_back(wp[​ 1 ​]);
}
​/*now we got the way points(discretizing the trajectory) in map
space ...now have to transform them into car reference frame for easy
calculation of the spline points before we transform them again in mapspace
for plotting in the simulation*/
​for​(​int​ i =​ 0 ​;i<point_x.size();i++)
{
​double​ transl_x = point_x[i] - reference_x;
​double​ transl_y = point_y[i] - reference_y;
point_x[i] = transl_x*​cos​(​ 0 ​-reference_yaw) -
transl_y*​sin​(​ 0 ​-reference_yaw);
point_y[i] = transl_x*​sin​(​ 0 ​-reference_yaw) +
transl_y*​cos​(​ 0 ​-reference_yaw);
}


​//now we got the transformed coordinates of the waypoints on the
trajectory
​//now inorder to make them match the speed specification given
that is 50MPH ..
​//we need to get the sampled points on interpolated polynomial
at intervals
​//equal to total_end_dist/(0.02*target_speed(in m/s ..divide by
2.24))
​//first keep the previous path points for smooth transition
​for​(​int​ i = ​ 0 ​; i < prev_size; i++)
{
next_x_vals.push_back(previous_path_x[i]);
next_y_vals.push_back(previous_path_y[i]);
}
​//interpolating the waypoints using cubic spline technique
​// it is under tk namespace in spline.h header file
tk::spline spl;
spl.set_points(point_x,point_y);
​//now we have to sample the interpolated polynomial and include
enough of them so that we have 50 points.
​double​ x = ​ 30 ​;
​double​ y = spl(x);
​double​ dist = ​sqrt​(x*x+y*y); ​//becuase we have shifted the
points to make the car position as origin
​double​ dummy_x = ​ 0 ​;
​double​ N = dist / (​0.02​ * ref_vel/​2.24​);
​for​(​int​ i=​ 0 ​;i<​ 50 ​ -prev_size;i++)
{
​double​ x_point = dummy_x + x / N;
​double​ y_point = spl(x_point);
dummy_x = x_point;
​double​ x_dum = x_point;
​double​ y_dum = y_point;
​// Rotating back to normal after rotating it earlier.
x_point = x_dum * ​cos​(reference_yaw) - y_dum *
sin​(reference_yaw);
y_point = x_dum * ​sin​(reference_yaw) + y_dum *
cos​(reference_yaw);
x_point += reference_x;
y_point += reference_y;
next_x_vals.push_back(x_point);
next_y_vals.push_back(y_point);
}
```
## Output :


● Output pictures of ego vehicle doing a tight maneuver

![](https://github.com/nikhilbadam56/HighwayDriving/blob/master/output/TurboVNC_%20unix_1%20-%20noVNC%20-%20Google%20Chrome%202020-12-26%2010-49-53_Trim_Moment.jpg)

![](https://github.com/nikhilbadam56/HighwayDriving/blob/master/output/TurboVNC_%20unix_1%20-%20noVNC%20-%20Google%20Chrome%202020-12-26%2010-49-53_Trim_Moment(2).jpg)

![](https://github.com/nikhilbadam56/HighwayDriving/blob/master/output/TurboVNC_%20unix_1%20-%20noVNC%20-%20Google%20Chrome%202020-12-26%2010-49-53_Trim_Moment(3).jpg)

![](https://github.com/nikhilbadam56/HighwayDriving/blob/master/output/TurboVNC_%20unix_1%20-%20noVNC%20-%20Google%20Chrome%202020-12-26%2010-49-53_Trim_Moment(4).jpg)

## To Improve:
To improve the current algorithm :

● we can further add the cost optimization function that calculates the cost for speed,
cost for acceleration limiting, cost for respective lane changing.

● In my opinion, we can use adaptive speed controlling (PID controllers) to control the
speed continuously through an adaptive amount rather than a fixed decrease in the
velocity.

## References :

● Referred the Q/A video section code for developing the above algorithm.


