close;
clear;
clc;

%derive the inverse kinematic solution
%input to this program is the coordinate of the object you need to pick up

% Input:
inputC = [400 -100 0 90];
px = inputC(1);
py = inputC(2);
pz = inputC(3);
rollAngle = ((inputC(4))*pi)/180;

% Robot known parameters according to the robot specification stated in the report (in mm)
l1 = 400;
l2 = 250;
d1 = 257.7;
d4 = 50;

% Inverse Kinematics
theta2 = acos((px^2 + py^2 - l1^2 - l2^2) / (2 * l1 * l2));
dC1 = det([px -(l2*sin(theta2)); py ((l2*cos(theta2))+l1)]);
dS1 = det([(l1+(l2*cos(theta2))) px; (l2*sin(theta2)) py]);
theta1 = atan(dS1/dC1);
theta4 = theta1 + theta2 - rollAngle;

d3 = d1 - d4 - pz;

% Link Definition
L1 = Link([0 d1 l1 0]);
L2 = Link([0 0 l2 pi]);
L3 = Link([0 0 0 0 1]); %prismatic joint
L3.qlim = [0 257.7];
L4 = Link([0 d4 0 0]);

% Robot Model
robot = SerialLink([L1 L2 L3 L4]);

% Position Plot
step = 50;
a1 = 0;
a2 = 0;
a3 = 0;
a4 = 0;
cArray = [0 0 0 0];

% Creating the movement steps
for a = 1:2
    for i = 1:step
        a1 = a1 + (theta1/step);
        a2 = a2 + (theta2/step);
        a3 = a3 + (d3/step);
        a4 = a4 + (theta4/step);
        cArray = [cArray; a1 a2 a3 a4];
    end
    for i = 1:step
        a1 = a1 - (theta1/step);
        a2 = a2 - (theta2/step);
        a3 = a3 - (d3/step);
        a4 = a4 - (theta4/step);
        cArray = [cArray; a1 a2 a3 a4]; 
    end
end

% Plotting the robot with the created set of coordinates
robot.plot(cArray);

% Robot Forward Kinematics
rfk = robot.fkine([theta1 theta2 d3 theta4]);