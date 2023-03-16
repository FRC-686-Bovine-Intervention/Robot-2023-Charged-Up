function [theta1, theta2] = get_elbow_up_path(start_theta1, start_theta2, final_theta1, final_theta2)

% shoulder forward
shoulderFwd.theta1 = -125.0 * pi/180;   % forward enough that elbow can come up
shoulderFwd.theta2 =   10.0 * pi/180;

% now that we've cleared the superstructure, 
% bring the elbow up

elbowUp.theta1 = shoulderFwd.theta1;
elbowUp.theta2 = elbowUp.theta1 + 140.0 * pi/180;

% now that the elbow is fully up,
% bring the shoulder out to its final position

shoulderOut.theta1 = final_theta1;
shoulderOut.theta2 = shoulderOut.theta1 + 140.0 * pi/180;

% if the elbow ends up going beyond 90 degrees,
% stop it at 90 degrees by adding another waypoint

elbow_max_angle = 75 * pi/180;

if (shoulderOut.theta2 > elbow_max_angle)
    delta = shoulderOut.theta2 - elbow_max_angle;
    shoulderOut.theta1 = [shoulderOut.theta1 - delta, shoulderOut.theta1];
    shoulderOut.theta2 = [shoulderOut.theta2 - delta, elbow_max_angle];
end

% and finally,
% extend the elbow


% elbowOut.theta1 = final_theta1;
% elbowOut.theta2 = final_theta1 + 140.0 * pi/180;
% 
% corner_radius = 0.2;
% 
% % elbow out 1
% elbowOut1.theta1 = elbowOut.theta1 - corner_radius;
% elbowOut1.theta2 = elbowOut.theta2 - corner_radius;
% 
% % elbow out 2
% elbowOut2.theta1 = elbowOut.theta1;
% elbowOut2.theta2 = elbowOut.theta2;
% 
% % elbow out 3
% elbowOut3.theta1 = elbowOut.theta1;
% elbowOut3.theta2 = elbowOut.theta2 - corner_radius;

% angular trajectory
theta1 = [start_theta1  elbowUp.theta1  shoulderOut.theta1 final_theta1];
theta2 = [start_theta2  elbowUp.theta2  shoulderOut.theta2 final_theta2];

