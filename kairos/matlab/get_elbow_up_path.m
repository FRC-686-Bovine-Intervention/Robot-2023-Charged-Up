function [theta1, theta2] = get_elbow_up_trajectory(start_theta1, start_theta2, final_theta1, final_theta2)

% shoulder forward
shoulderFwd.theta1 = -130.0 * pi/180;
shoulderFwd.theta2 = start_theta2;

% now that we've cleared the superstructure, 
% start bringing elbow up while continuing
% to move the shoulder to it's final angle

elbowOut.theta1 = final_theta1;
elbowOut.theta2 = final_theta1 + 140.0 * pi/180;

corner_radius = 0.2;

% elbow out 1
elbowOut1.theta1 = elbowOut.theta1 - corner_radius;
elbowOut1.theta2 = elbowOut.theta2 - corner_radius;

% elbow out 2
elbowOut2.theta1 = elbowOut.theta1;
elbowOut2.theta2 = elbowOut.theta2;

% elbow out 3
elbowOut3.theta1 = elbowOut.theta1;
elbowOut3.theta2 = elbowOut.theta2 - corner_radius;

% angular trajectory
theta1 = [start_theta1  elbowOut1.theta1  elbowOut2.theta1  elbowOut3.theta1  final_theta1];
theta2 = [start_theta2  elbowOut1.theta2  elbowOut2.theta2  elbowOut3.theta2  final_theta2];

