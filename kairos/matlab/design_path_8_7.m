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

theta1 = [orig_points(1,1)*180/pi  -143.3 -58.8  orig_points(1,end)*180/pi] * pi/180;
theta2 = [orig_points(2,1)*180/pi    16.7  46.9  orig_points(2,end)*180/pi] * pi/180;

points = [theta1; theta2];

% Make Trajectory ====================================

points = make_spline_trajectory(startIdx, finalIdx, points, orig_points, orig_T);

% write new spline trajectory
s.theta1 = points(1,:);
s.theta2 = points(2,:);

write_spline_trajectory(startIdx, finalIdx, s);
