% Read trajectory
startIdx = 1;
finalIdx = 6;
filename = sprintf('arm_path_%d_%d.json', startIdx, finalIdx);
s = jsondecode(fileread(['..\..\src\main\deploy\paths\' filename]));
orig_T = max(s.totalTime, 0);
orig_points = [s.theta1.'; s.theta2.'];
orig_theta1 = orig_points(1,:);
orig_theta2 = orig_points(2,:);

theta1 = [orig_theta1(1)*180/pi  -118.2   orig_theta1(end)*180/pi] * pi/180;
theta2 = [orig_theta2(1)*180/pi     8.2   orig_theta2(end)*180/pi] * pi/180;

points = [theta1; theta2];


% Make Trajectory ====================================

points = make_spline_trajectory(points, orig_points, orig_T);

% write new spline trajectory
s.theta1 = points(1,:);
s.theta2 = points(2,:);

write_spline_trajectory(startIdx, finalIdx, s);