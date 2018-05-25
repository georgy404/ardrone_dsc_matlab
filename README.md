# ardrone_dsc_matlab    
MATLAB realisation of ardrone_dcs ROS package      

This is MATLAB realisation of [ardrone_dcs](https://github.com/georgy404/ardrone_dcs) - ROS package for control Parrot AR.Drone 2.0.      

Demonstration with help tum_simulator:         
https://www.youtube.com/watch?v=GFN0oyjk54c       
Demonstration of real drone:    
https://www.youtube.com/watch?v=hncuU-by7Gc     

![trajectory](imgs/traj.png?raw=true "Trajectory")
![fly_data](imgs/graph.png?raw=true "Fly data")


### Required components    
   
- [ROS Indigo](http://wiki.ros.org/indigo)     
- [Gazebo 2](http://gazebosim.org/)     
- [ardrone_autonomy](http://wiki.ros.org/ardrone_autonomy) ROS package     
- [tum_simulator](http://wiki.ros.org/tum_simulator) ROS package     
- [MATLAB](https://www.mathworks.com/products/matlab.html)    
- [Robotics System Toolbox](https://www.mathworks.com/products/robotics.html)  for MATLAB    

### Configuration    
- IP address of ROS master:    
```     
rosMasterIP='127.0.0.1';    
```     
- List of waypoints:     
```   
waypoints = [ [1 1 1]    
              [1 4 1]   
              [4 4 1]   
              [4 1 1]   
              [1 1 1] ];   
```    
- Waypoint radius. This is distance at which it switches to next waypoint. For real drone it's value more then value for simulator, because odometry data of real drone is not stable:   
```   
waypointRadius = 0.2; % for simulator    
% waypointRadius = 0.4; % for real drone    
```   
- Name of configuration file:   
```    
cfgFileName='pid_params_sim.mat';       
% cfgFileName='pid_params_real.mat';    
```           
      

