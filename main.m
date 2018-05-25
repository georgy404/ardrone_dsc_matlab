clear
close all

% Init matlab ros node
rosMasterIP='127.0.0.1';
rosinit(rosMasterIP);

% Init tf for get position
tftree = rostf;
pause(1);
tftree.AvailableFrames;

% Init sub/pub topics
% For get linear velocity and orientation
navDataTopic = rossubscriber('/ardrone/navdata');
% For get angular velocity
imuDataTopic = rossubscriber('/ardrone/imu');
% For get image
imageTopic = rossubscriber('/ardrone/image_raw');
% For send reset
resetArdrone = rospublisher('/ardrone/reset');
% For send takeoff
takeoffArdrone = rospublisher('/ardrone/takeoff');
% For send land
landArdrone = rospublisher('/ardrone/land');
% For send velocity control commands
cmdVelArdrone = rospublisher('/cmd_vel');

% Set waypoints       
waypoints = [ [1 1 1]
              [4 1 1] 
              [4 4 1]
              [1 4 1]
              [1 1 1]
            ];
waypointsCount = size(waypoints, 1);

% Distance at which make switching waypoints 
waypointRadius = 0.2; % for simulator
% waypointRadius = 0.4; % for real drone

% Choose name of cfg file
cfgFileName='pid_params_sim.mat';
% cfgFileName='pid_params_real.mat';

% Load PID controller params
load(cfgFileName, ...
     ... % PID params for yaw
     'kpYaw', ...  % 0.0 < kpYaw < 1.0
     'kiYaw', ...  % 0.0 < kiYaw < 1.0
     'kdYaw', ...  % 0.0 < kdYaw < 1.0
     'maxYaw', ... % 0.0 < maxYaw < 1.0
     ... % PID params for gaz
     'kpGaz', ...      % 0.0 < kpGaz < 1.0
     'kiGaz', ...      % 0.0 < kiGaz < 1.0
     'kdGaz', ...      % 0.0 < kdGaz < 1.0
     'riseFac', ...    % 0.0 < riseFac < 5.0
     'maxGazRise', ... % 0.0 < maxRiseFac < 1.0
     'maxGazDrop', ... % -1.0 < maxGazDrop < 0.0
     ... % PID params for roll/pitch
     'kpRP', ... % 0.0 < kpRP < 1.0
     'kiRP', ... % 0.0 < kiRP < 1.0
     'kdRP', ... % 0.0 < kdRP < 1.0
     'maxRP');   % 0.0 < maxRP < 1.0

% Send takeoff command
send(takeoffArdrone, rosmessage(takeoffArdrone));

% Init variables for integration magic
lastPError = [0 0 0 0];
iTerm = [0 0 0 0];
newTarget = [1 1 1 1];
navData = receive(navDataTopic, 40);
lastTime = navData.Header.Stamp.Sec + navData.Header.Stamp.Nsec/1000000000;
timeGetTarget = lastTime;

% Create window for plots
figure
i = 1; % iterator for navigation data
% Variables for saving navigation data
poseX=[]; poseY=[]; poseZ=[];
roll=[]; pitch=[]; yaw=[];
linearVelX=[]; linearVelY=[]; linearVelZ=[];
angularVelX=[]; angularVelY=[]; angularVelZ=[]; 

currentWaypointNum = 1;
while (currentWaypointNum < (waypointsCount+1))
    % Get position
    ardroneTarnsform = getTransform(tftree, 'odom', 'base_link');
    pose = [ardroneTarnsform.Transform.Translation.X ...
            ardroneTarnsform.Transform.Translation.Y ...
            ardroneTarnsform.Transform.Translation.Z];
    poseX = [poseX; pose(1)]; poseY = [poseY; pose(2)]; poseZ = [poseZ; pose(3)]; 
    
    % Get linear vel and orientation
    navData = receive(navDataTopic, 40);
    linearVel = [navData.Vx/1000. navData.Vy/1000. navData.Vz/1000.];
    orientation = [navData.RotX navData.RotY navData.RotZ];
    droneYaw = NormalizeAngle(degtorad(navData.RotZ));
    roll = [roll; orientation(1)]; pitch = [pitch; orientation(2)]; yaw = [yaw; orientation(3)];
    linearVelX = [linearVelX; linearVel(1)]; 
    linearVelY = [linearVelY; linearVel(2)]; 
    linearVelZ = [linearVelZ; linearVel(3)];
    
    % Get angular vel
    imuData = receive(imuDataTopic, 40);
    angularVel = [imuData.AngularVelocity.X imuData.AngularVelocity.Y imuData.AngularVelocity.Z];
    angularVelX = [angularVelX; angularVel(1)]; 
    angularVelY = [angularVelY; angularVel(2)]; 
    angularVelZ = [angularVelZ; angularVel(3)];
    
    % Get current waypoint
    waypoint = waypoints(currentWaypointNum, :);
    
    % Calculate course
    course = atan2(waypoint(2)-pose(2), waypoint(1)-pose(1))*180/pi;
    
    % Calculate proportional error
    pError = [waypoint-pose (course-orientation(3))];
    
    % Calculate differential error
    dError = [-linearVel(1) -linearVel(2) -linearVel(3) -angularVel(3)];
    
    % Calculate integral error
    % Get current time in seconds, calculate time of integration
    currentTime = navData.Header.Stamp.Sec + navData.Header.Stamp.Nsec/1000000000;
    time = 0;%currentTime - lastTime;
    lastTime = currentTime;
    % Integrate and cap
    iTerm(3) = ITermIncrease(iTerm(3), pError(3) * time, 0.2 / kiGaz);
    iTerm(2) = ITermIncrease(iTerm(2), pError(2) * time, 0.1 / kiRP + (1e-10));
    iTerm(1) = ITermIncrease(iTerm(1), pError(1) * time, 0.1 / kiRP + (1e-10));
    % Kill integral term when first crossing target
    % That is, thargetNew is set, it was set at least 100ms ago, and err changed sign
    for j=1:4
        err = lastPError(j) * pError(j);
        if (newTarget(j) > 0.5 && timeGetTarget > 0.1 && err < 0)
            iTerm(j) = 0;
            newTarget(j) = 0;
        end
    end
    
    % Calculate proportional term
    pTerm = pError;
    pTerm(1) = cos(droneYaw) * pError(1) + sin(droneYaw) * pError(2);
    pTerm(2) = -sin(droneYaw) * pError(1) + cos(droneYaw) * pError(2);
    
    % Calculate differentian term
    dTerm = dError;
    dTerm(1) = -sin(droneYaw) * dError(1) + cos(droneYaw) * dError(2);
    dTerm(2) = -cos(droneYaw) * dError(1) - sin(droneYaw) * dError(2);
             
    % Create command msg
    cmdVelMsg = rosmessage(cmdVelArdrone);
    
    % Calculate cmd for forward/backward vel
    cmdX = kpRP * pTerm(1) + kiRP * iTerm(1) + kdRP * dTerm(1);
    cmdX = min(maxRP, max(-maxRP, cmdX));
    cmdVelMsg.Linear.X = cmdX;
    
    % Calculate cmd for left/right vel
    cmdY = kpRP * pTerm(2) + kiRP * iTerm(2) + kdRP * dTerm(2);
    cmdY = min(maxRP, max(-maxRP, cmdY));
    cmdVelMsg.Linear.Y = cmdY;
    
    % Calculate cmd for up/down vel
    cmdZ = kpGaz * pTerm(3) + kiGaz * iTerm(3) + kdGaz * dTerm(3);
    cmdZ = min(maxGazRise/riseFac, max(maxGazDrop, cmdZ));
    if (cmdZ > 0) cmdZ = cmdZ * riseFac; end
    cmdVelMsg.Linear.Z = cmdZ;
   
    % Calculate cmd for yaw 
    cmdYaw = kpYaw * pTerm(4) + kdYaw * dTerm(4);
    cmdYaw = min(maxYaw, max(-maxYaw, cmdYaw));
    cmdVelMsg.Angular.Z = cmdYaw;
    
    % Send control commnad
    send(cmdVelArdrone, cmdVelMsg);

    % Set error as last error
    lastPError = pError;
    
    % Switch to new waypoint if then need
    distToWaypoint = sqrt((waypoint(1)-pose(1))*(waypoint(1)-pose(1)) + ...
                          (waypoint(2)-pose(2))*(waypoint(2)-pose(2)) + ...
                          (waypoint(3)-pose(3))*(waypoint(3)-pose(3)));
    if (distToWaypoint < waypointRadius) % if distance <= waypointRadius, going to next waypoint
        currentWaypointNum = currentWaypointNum + 1;
        timeGetTarget = currentTime;
        newTarget = [1 1 1 1];
        lastPError = [0 0 0 0];
        iTerm = [0 0 0 0];
    end
              
    % Draw image
    imageData = receive(imageTopic, 40);
    image = readImage(imageData);
    imshow(image);
    
    i = i+1;
    pause(0.01);
end

% Send land command
send(landArdrone, rosmessage(landArdrone));
pause(1);

% Shutdown matlab ROS node
rosshutdown;

% Draw plots
figure
% Draw trajectory
subplot(2,2,1);
ardroneTrajectory = [poseX poseY poseZ];
plot3(poseX, poseY, poseZ);
title('Trajectory [m]'); grid on;
   
% Draw plot of orientations
subplot(2,2,2);
ardroneAngles = [roll pitch yaw];
plot(ardroneAngles);
grid on; title('Angles [deg]'); legend('roll', 'pitch', 'yaw');
    
% Draw plot of linear velocity
subplot(2,2,3);
ardroneLinearVel = [linearVelX linearVelY linearVelZ];
plot(ardroneLinearVel);
grid on; title('Linear velocity [m/s]'); legend('x', 'y', 'z');
    
% Draw plot of angular velocity
subplot(2,2,4);
ardroneAngularVel = [angularVelX angularVelY angularVelZ];
plot(ardroneAngularVel);
grid on; title('Angular velocity [rad/s]'); legend('x', 'y', 'z');

% Save navigation data as *.mat file
c = clock;
nameMask = './log_files/ardrone_data_%d-%d-%d_%d-%d-%d.mat';
fileName = sprintf(nameMask, c(1), c(2), c(3), c(4), c(5), round(c(6)));
save(fileName, 'ardroneTrajectory', 'ardroneAngles', 'ardroneLinearVel', 'ardroneAngularVel');

% If yout want update pid params, you can rewrite cfg file
% save(cfgFileName, 'kpYaw', 'kiYaw', 'kdYaw', 'maxYaw', ... 
%                   'kpGaz', 'kiGaz', 'kdGaz', 'riseFac', 'maxGazRise', 'maxGazDrop', ... 
%                   'kpRP', 'kiRP', 'kdRP', 'maxRP');
