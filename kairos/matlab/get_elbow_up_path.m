function [theta1, theta2] = get_elbow_up_path(start_theta1, start_theta2, final_theta1, final_theta2)

minimum_shoulder_elbow_angle_deg = 32.5;

% shoulder forward
shoulderFwd.theta1 = -125.0 * pi/180;   % forward enough that elbow can come up
shoulderFwd.theta2 =   10.0 * pi/180;

% now that we've cleared the superstructure, 
% bring the elbow up

elbowUp.theta1 = shoulderFwd.theta1;
elbowUp.theta2 = elbowUp.theta1 + (180.0 - minimum_shoulder_elbow_angle_deg) * pi/180;

% now that the elbow is fully up,
% bring the shoulder out to its final position

shoulderOut.theta1 = final_theta1;
shoulderOut.theta2 = shoulderOut.theta1 + (180.0 - minimum_shoulder_elbow_angle_deg) * pi/180;

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



% angular trajectory
theta1 = [start_theta1  elbowUp.theta1  shoulderOut.theta1 final_theta1];
theta2 = [start_theta2  elbowUp.theta2  shoulderOut.theta2 final_theta2];

