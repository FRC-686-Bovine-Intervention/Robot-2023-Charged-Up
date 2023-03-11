function [corner1_theta1, corner1_theta2, corner2_theta1, corner2_theta2] = get_safe_path_corners()

s = jsondecode(fileread('..\..\src\main\deploy\arm_config.json'));
arm_length(1) = s.shoulder.length;
arm_length(2) = s.elbow.length + s.wrist.length;
origin.x = s.origin(1);
origin.y = s.origin(2);


obstaclesJson = jsondecode(fileread('..\..\src\main\deploy\constraints.json'));

corner1_x = obstaclesJson.MID_NODE_MARGIN.args(1);
corner1_y = obstaclesJson.MID_NODE_MARGIN.args(4);

corner2_x = obstaclesJson.HIGH_NODE_MARGIN.args(1);
corner2_y = obstaclesJson.HIGH_NODE_MARGIN.args(4);


kinematics = arm_kinematics(origin.x, origin.y, arm_length);

[corner1_theta1, corner1_theta2, x2, y2, valid] = kinematics.inverse_kinematics(corner1_x, corner1_y);
[corner2_theta1, corner2_theta2, x2, y2, valid] = kinematics.inverse_kinematics(corner2_x, corner2_y);
