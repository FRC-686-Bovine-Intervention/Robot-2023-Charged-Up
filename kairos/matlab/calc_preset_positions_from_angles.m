intake_theta = [-2.21 -0.53];
defense_theta = [-2.32 +0.145];
auto_start_theta = [-2.25 0.47];

s = jsondecode(fileread('..\..\src\main\deploy\arm_config.json'));

arm_length(1) = s.shoulder.length;
arm_length(2) = s.elbow.length + s.wrist.length;
origin.x = s.origin(1);
origin.y = s.origin(2);

kinematics = arm_kinematics(origin.x, origin.y, arm_length);


[~,~,intake_x, intake_y] = kinematics.forward_kinematics(intake_theta(1), intake_theta(2));
[~,~,defense_x, defense_y] = kinematics.forward_kinematics(defense_theta(1), defense_theta(2));
[~,~,auto_start_x, auto_start_y] = kinematics.forward_kinematics(auto_start_theta(1), auto_start_theta(2));


fprintf('intake_x = %.3f\n', intake_x * 100/2.54);
fprintf('intake_y = %.3f\n', intake_y * 100/2.54);
fprintf('defense_x = %.3f\n', defense_x * 100/2.54);
fprintf('defense_y = %.3f\n', defense_y * 100/2.54);
fprintf('auto_start_x = %.3f\n', auto_start_x * 100/2.54);
fprintf('auto_start_y = %.3f\n', auto_start_y * 100/2.54);
