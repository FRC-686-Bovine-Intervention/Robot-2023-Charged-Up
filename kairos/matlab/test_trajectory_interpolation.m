s = jsondecode(fileread('..\..\src\main\deploy\arm_config.json'));

arm_length(1) = s.shoulder.length;
arm_length(2) = s.elbow.length + s.wrist.length;
origin.x = s.origin(1);
origin.y = s.origin(2);

kinematics = arm_kinematics(origin.x, origin.y, arm_length);

presetEnum = get_preset_enum;

s = jsondecode(fileread('..\..\src\main\deploy\arm_preset_poses.json'));

preset(presetEnum.DEFENSE).info = s.defense;                    preset(presetEnum.DEFENSE).name = "defense";
preset(presetEnum.SCORE_HIGH_CONE).info = s.score_high_cone;    preset(presetEnum.SCORE_HIGH_CONE).name = "score_high_cone";    

obstaclesJson = jsondecode(fileread('..\..\src\main\deploy\constraints.json'));





startPreset = presetEnum.DEFENSE;
finalPreset = presetEnum.SCORE_HIGH_CONE;

[theta1, theta2] = get_elbow_up_path(preset(startPreset).info.theta1, preset(startPreset).info.theta2, ...
                                     preset(finalPreset).info.theta1, preset(finalPreset).info.theta2);
points = [theta1; theta2];
[points, totalTime] = make_spline_trajectory2(points, strrep(sprintf('%s to %s', preset(startPreset).name, preset(finalPreset).name), '_','\_'));
[elbow_loc.x, elbow_loc.y, tip_loc.x, tip_loc.y] = kinematics.forward_kinematics(points(1,:), points(2,:));



% adjust extension

[~,~,x,z] = kinematics.forward_kinematics(theta1(end), theta2(end));

x = x + 6.0 * 2.54/100;

[adj_theta1, adj_theta2] = kinematics.inverse_kinematics(x,z);

[theta1_adj, theta2_adj] = get_elbow_up_path(preset(startPreset).info.theta1, preset(startPreset).info.theta2, ...
                                     adj_theta1, adj_theta2);
points_adj = [theta1_adj; theta2_adj];
[points_adj, totalTime] = make_spline_trajectory2(points_adj, strrep(sprintf('%s to %s', preset(startPreset).name, preset(finalPreset).name), '_','\_'));
[elbow_adj.x, elbow_adj.y, tip_adj.x, tip_adj.y] = kinematics.forward_kinematics(points_adj(1,:), points_adj(2,:));



inchesPerMeter = 39.3701;
xyScale = inchesPerMeter;
angleScale = 180/pi;
title_str = 'Trajectory Interp Test';

figure(88);
plot_xy_space(gca, obstaclesJson, xyScale, title_str);
plot_xy_path(gca, origin, elbow_loc, tip_loc, xyScale);
plot_xy_path(gca, origin, elbow_adj, tip_adj, xyScale);
        
figure(99);
plot_angle_space(gca, obstaclesJson, kinematics, angleScale, title_str);
plot_angle_path(gca, points(1,:), points(2,:), angleScale);
plot_angle_path(gca, points_adj(1,:), points_adj(2,:), angleScale);


% new interpolation algorithm
[~, idx] = max(points(1,:));    % find max shoulder angle

points_delta = [adj_theta1(end) - theta1(end)
                adj_theta2(end) - theta2(end)];

% start remains the same
points_interp = points(:,1:idx-2);

% linearly interpolate the middle
dist = norm(points_delta, 2);
Vmax = norm(points(:,idx) - points(:,idx-1),2); % rad/sec
npoints = ceil(dist / Vmax);

points_interp = [points_interp, points(:,idx-1) + points_delta*(0:npoints)/npoints];

% adjust the end to go to adjusted position
points_interp = [points_interp,  points(:,idx:end) + points_delta];


[elbow_interp.x, elbow_interp.y, tip_interp.x, tip_interp.y] = kinematics.forward_kinematics(points_interp(1,:), points_interp(2,:));

figure(88);
plot_xy_path(gca, origin, elbow_interp, tip_interp, xyScale);

figure(99);
plot_angle_path(gca, points_interp(1,:), points_interp(2,:), angleScale);
