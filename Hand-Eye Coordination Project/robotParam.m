function [outputArray] = robotParam(inputCoor)
    %Input:
    px = inputCoor(1);
    py = inputCoor(2);
    pz = 0;
    rollAngle = ((inputCoor(3))*pi)/180;

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

    outputArray = [theta1, theta2, d3, theta4];
end