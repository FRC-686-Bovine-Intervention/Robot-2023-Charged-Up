% Read trajectory
startIdx = 8;
finalIdx = 7;
filename = fullfile('..\..\src\main\deploy\paths\', sprintf('arm_path_%d_%d_orig.json', startIdx, finalIdx));
if ~exist(filename, 'file')
    filename = fullfile('..\..\src\main\deploy\paths\', sprintf('arm_path_%d_%d.json', startIdx, finalIdx));
end
s = jsondecode(fileread(filename));
orig_T = max(s.totalTime, 0);
orig_points = [s.theta1.'; s.theta2.'];
% remove extraneous loop
orig_points = orig_points(:,[1 6:end]);

[corner1_theta1, corner1_theta2, corner2_theta1, corner2_theta2] = get_safe_path_corners();

theta1 = [orig_points(1,1)  -110.0*pi/180 corner2_theta1  orig_points(1,end)];
theta2 = [orig_points(2,1)    43.0*pi/180 corner2_theta2  orig_points(2,end)];

points = [theta1; theta2];

% Make Trajectory ====================================

points = make_spline_trajectory(startIdx, finalIdx, points, orig_points, orig_T);

% write new spline trajectory
s.theta1 = points(1,:);
s.theta2 = points(2,:);

write_spline_trajectory(startIdx, finalIdx, s);
