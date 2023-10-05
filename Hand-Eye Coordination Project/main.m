close;
clear;
clc;

% Image Processing
greenThresholds = [0.000, 35.430, -22.572, -5.655, 1.116, 69.530];
blackThresholds = [0.000, 33.753, -3.285, 10.697, -10.624, 2.024];
yellowThresholds = [45.493, 100.000, -22.572, 55.141, 30.885, 69.530];
redThresholds = [0.000, 100.000, 30.258, 55.141, -21.106, 69.530];
blueThresholds = [0.000, 31.027, -22.572, 55.141, -63.035, -18.101];

x = imread('Image2.jpg');

[temp, temp1] = findCoordinate(x, yellowThresholds);

allCoor = [];
robotArray = [];

% Retrieving all information of all objects
for i = 1:temp1
    temp2 = temp(i).Centroid;
    allCoor = [allCoor; temp2(1), temp2(2), temp(i).Orientation];
end

% Setting up the movement coordinates of the robot
for i = 1:temp1
    robotArray = [robotArray; 0 0 0 0];
    step = 10;
    a1 = 0;
    a2 = 0;
    a3 = 0;
    a4 = 0;
    temp = robotParam(allCoor(i,:))
    for z = 1:step
        a1 = a1 + (temp(1)/step);
        a2 = a2 + (temp(2)/step);
        a3 = a3 + (temp(3)/step);
        a4 = a4 + (temp(4)/step);
        robotArray = [robotArray; a1 a2 a3 a4];
    end
    for y = 1:step
        a1 = a1 - (temp(1)/step);
        a2 = a2 - (temp(2)/step);
        a3 = a3 - (temp(3)/step);
        a4 = a4 - (temp(4)/step);
        robotArray = [robotArray; a1 a2 a3 a4]; 
    end
end

% Robot known parameters according to the robot specification stated in the report (in mm)
l1 = 400;
l2 = 250;
d1 = 257.7;
d4 = 50;

% Link Definition
L1 = Link([0 d1 l1 0]);
L2 = Link([0 0 l2 pi]);
L3 = Link([0 0 0 0 1]); %prismatic joint
L3.qlim = [0 257.7];
L4 = Link([0 d4 0 0]);

% Robot Model
robot = SerialLink([L1 L2 L3 L4]);

% Plot the robot using the created movement coordinates
robot.plot(robotArray);

