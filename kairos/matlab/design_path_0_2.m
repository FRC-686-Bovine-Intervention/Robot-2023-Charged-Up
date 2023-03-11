% Read trajectory
startIdx = 0;
finalIdx = 2;
filename = fullfile('..\..\src\main\deploy\paths\', sprintf('arm_path_%d_%d_orig.json', startIdx, finalIdx));
if ~exist(filename, 'file')
    filename = fullfile('..\..\src\main\deploy\paths\', sprintf('arm_path_%d_%d.json', startIdx, finalIdx));
end
s = jsondecode(fileread(filename));
orig_T = max(s.totalTime, 0) * 1.0;
orig_points = [s.theta1.'; s.theta2.'];
orig_theta1 = orig_points(1,:);
orig_theta2 = orig_points(2,:);

[corner1_theta1, corner1_theta2, corner2_theta1, corner2_theta2] = get_safe_path_corners();

theta1 = [orig_theta1(1)  corner2_theta1   orig_theta1(end)   orig_theta1(end)];
theta2 = [orig_theta2(1)  corner2_theta2   corner2_theta2     orig_theta2(end)];

points = [theta1; theta2];


% Make Trajectory ====================================

spline_T = 0.6;
points = make_spline_trajectory(startIdx, finalIdx, points, orig_points, spline_T);

% write new spline trajectory
s.theta1 = points(1,:);
s.theta2 = points(2,:);

write_spline_trajectory(startIdx, finalIdx, s);