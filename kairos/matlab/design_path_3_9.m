% Read trajectory
startIdx = 3;
finalIdx = 9;
filename = fullfile('..\..\src\main\deploy\paths\', sprintf('arm_path_%d_%d_orig.json', startIdx, finalIdx));
if ~exist(filename, 'file')
    filename = fullfile('..\..\src\main\deploy\paths\', sprintf('arm_path_%d_%d.json', startIdx, finalIdx));
end
s = jsondecode(fileread(filename));
orig_T = s.totalTime;
orig_points = [s.theta1.'; s.theta2.'];
orig_theta1 = orig_points(1,:);
orig_theta2 = orig_points(2,:);

[corner1_theta1, corner1_theta2, corner2_theta1, corner2_theta2] = get_safe_path_corners();

theta1 = [orig_theta1(1)  -135*pi/180  -160*pi/180  -120*pi/180  orig_theta1(end)];
theta2 = [orig_theta2(1)   -45*pi/180   -20*pi/180   +40*pi/180  orig_theta2(end)];

points = [theta1; theta2];


% Make Trajectory ====================================

spline_T = 2.0;    % slowing down
points = make_spline_trajectory(startIdx, finalIdx, points, orig_points, spline_T);

% write new spline trajectory
s.grannyFactor = 1.25;
s.theta1 = points(1,:);
s.theta2 = points(2,:);


write_spline_trajectory(startIdx, finalIdx, s, false);