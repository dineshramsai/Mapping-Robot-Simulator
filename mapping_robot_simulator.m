clc; clear all; close all;
rosshutdown %Shutting Down the ROS Interface
rosinit %Initiating the ROS Interface
sim = ExampleHelperRobotSimulator( 'complexMap' ); %Creating the map for robot path
setRobotPose(sim, [2 4 -pi/2]); %Setting the Initial Robot Pose
enableROSInterface(sim, true);%Enabling the ROS Interface for the map
sim.LaserSensor.NumReadings = 50;%Setting the readings from the Lidar Sensor per update
scanSub = rossubscriber( 'scan' ); %Initiating the Environment Scanning Operation
[velPub, velMsg] = rospublisher( '/mobile_base/commands/velocity' ); %Seeking the Velocity Coordinates at a instant
tftree = rostf;
pause(1); %delay time
path = [4.5 2; 3.5 4; 8 5.75; 1 10; 1 15; 9 15; 5 18; 9 15; 6.5 12; 9 15; 14 15; 12 18; 14 15; 13 12; 14 15; 16 15; 16 12; 16 15; 20 19.5; 25 16; 20 9;
    15.5 5.5; 25 4; 16 4; 16 1; 16 4; 13.5 4; 13.5 2; 13.5 8; 13.5 4; 8 4; 8 2]; %Arranging the path for the robot in the given 
plot(path(:,1), path(:,2), 'k--d' ); %Plotting the robot path inside the map
controller = robotics.PurePursuit( 'Waypoints' , path); %Declaring a parameter for tracking robot per pursuit
controller.DesiredLinearVelocity = 0.4; %Setting Desired Linear Velocity
controlRate = robotics.Rate(10); %Setting the Rate, at which the parameters associated with the Robot change
goalRadius = 0.1; %Setting the value to Goal Radius
robotCurrentLocation = path(1,:); %Parameter for calculating the current location
robotGoal = path(end,:); %parameter for Robot's Goal
distanceToGoal = norm(robotCurrentLocation - robotGoal); %Parameter for the Distance of the Goal from it's current location
map = robotics.OccupancyGrid(25,20,20); %Creating Occupancy Grid
figureHandle = figure( 'Name' , 'Map' ); %Naming the Graph Figures
axesHandle = axes( 'Parent' , figureHandle); %Naming the appropriate Axes
mapHandle = show(map, 'Parent' , axesHandle); %Naming the map
title(axesHandle, 'OccupancyGrid: Update 0' );
updateCounter = 1; %Starting number Update Counter. There will be 50 Lidar Readings per update. Update Counter is dependent on Map and Path
while ( distanceToGoal > goalRadius ) %Set of Commands to be executed for each update
    scan = receive(scanSub); %Scanning the Environment
    pose = getTransform(tftree, 'map' , 'robot_base' , scan.Header.Stamp, 'Timeout' , 2); %Setting the Pose
    position = [pose.Transform.Translation.X,pose.Transform.Translation.Y]; %Parameter for position of Robot
    orientation = quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX' ); %Orientation of the Robot
    robotPose = [position, orientation(1)]; %Configuring the Robot Pose
    ranges = scan.Ranges; %Parameter for scanning Range
    angles = scan.readScanAngles; %Parameter for Scanning Angles
    ranges(isnan(ranges)) = sim.LaserSensor.MaxRange; %Fixing the Scanning Rangr to High(Max)
    insertRay(map, robotPose, ranges, angles,sim.LaserSensor.MaxRange) %Configuring Robot Motion and Scanning Process
    [v, w] = controller(robotPose); %Extracting the velocity Parameters of robot
    velMsg.Linear.X = v;
    velMsg.Angular.Z = w;
    send(velPub, velMsg);
    if ~mod(updateCounter,50) 
        mapHandle.CData = occupancyMatrix(map); %Plotting the Scanned Data into Occupancy Grid
        title(axesHandle, [ 'OccupancyGrid: Update ' num2str(updateCounter)]);
    end
    updateCounter = updateCounter+1; %Incrementing the Update Counter for next iteration
    distanceToGoal = norm(robotPose(1:2) - robotGoal);%Setting the Distance Goal of Robot for next iteration
    waitfor(controlRate); %Waiting till the Control Rate has been set up
end
show(map, 'Parent' , axesHandle); %Displaying the Map
title(axesHandle, 'OccupancyGrid: Final Map' ); %Title for the map
rosshutdown %Shutting Down ROS Interface
displayEndOfDemoMessage(mfilename)
