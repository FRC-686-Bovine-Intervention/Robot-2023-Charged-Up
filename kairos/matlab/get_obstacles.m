function [obstacle, obstacleEnum] = get_obstacles(bumper_width_inches, fieldPos, margin, intake_margin, calc)
%=====================================
% Define obstacles
%=====================================

obstacleEnum.ROBOT_BODY          = 1;
obstacleEnum.INTAKE              = 2;
obstacleEnum.BONKER_BAR_TM       = 3;
obstacleEnum.MID_NODE           = 4;
obstacleEnum.HIGH_NODE          = 5;
obstacleEnum.MID_POLE            = 6;
obstacleEnum.HIGH_POLE           = 7;
obstacleEnum.MID_NODE_MARGIN    = 8;
obstacleEnum.HIGH_NODE_MARGIN   = 9;


obstacle{obstacleEnum.ROBOT_BODY}.x = [-bumper_width_inches/2    -bumper_width_inches/2  +bumper_width_inches/2  +bumper_width_inches/2];
obstacle{obstacleEnum.ROBOT_BODY}.y = [0.0                       9.0 + intake_margin.y     9.0 + intake_margin.y     0.0];

obstacle{obstacleEnum.INTAKE}.x = [0.0  0.0  12.0  12.0];
obstacle{obstacleEnum.INTAKE}.y = [0.0  12.0 + intake_margin.y    12.0 + intake_margin.y     0.0];

obstacle{obstacleEnum.BONKER_BAR_TM}.x = [0.0  0.0  6.5  6.5];
obstacle{obstacleEnum.BONKER_BAR_TM}.y = [0.0   21.0 + intake_margin.y    21.0 + intake_margin.y     0.0];

obstacle{obstacleEnum.MID_NODE}.x =  [fieldPos.mid_shelf_front_corner.x  fieldPos.mid_shelf_front_corner.x    fieldPos.back_wall.x     fieldPos.back_wall.x];
obstacle{obstacleEnum.MID_NODE}.y =  [0                         fieldPos.mid_shelf.y                 fieldPos.mid_shelf.y     0];

obstacle{obstacleEnum.HIGH_NODE}.x =  [fieldPos.high_shelf_front_corner.x fieldPos.high_shelf_front_corner.x   fieldPos.back_wall.x     fieldPos.back_wall.x];
obstacle{obstacleEnum.HIGH_NODE}.y =  [0                         fieldPos.high_shelf.y                fieldPos.high_shelf.y    0];

obstacle{obstacleEnum.MID_POLE}.x = [fieldPos.mid_pole.x - fieldPos.pole_dia/2     fieldPos.mid_pole.x - fieldPos.pole_dia/2     fieldPos.mid_pole.x + fieldPos.pole_dia/2     fieldPos.mid_pole.x + fieldPos.pole_dia/2];
obstacle{obstacleEnum.MID_POLE}.y = [0                           fieldPos.mid_pole.y                  fieldPos.mid_pole.y                  0];

obstacle{obstacleEnum.HIGH_POLE}.x = [fieldPos.high_pole.x - fieldPos.pole_dia/2   fieldPos.high_pole.x - fieldPos.pole_dia/2    fieldPos.high_pole.x + fieldPos.pole_dia/2   fieldPos.high_pole.x + fieldPos.pole_dia/2];
obstacle{obstacleEnum.HIGH_POLE}.y = [0                          fieldPos.high_pole.y                 fieldPos.high_pole.y                 0];

obstacle{obstacleEnum.MID_NODE_MARGIN}.x = obstacle{obstacleEnum.MID_NODE}.x - margin.x*[1 1 0 0];
obstacle{obstacleEnum.MID_NODE_MARGIN}.y = obstacle{obstacleEnum.MID_NODE}.y + margin.y*[0 1 1 0];

obstacle{obstacleEnum.HIGH_NODE_MARGIN}.x = obstacle{obstacleEnum.HIGH_NODE}.x - margin.x*[1 1 0 0];
obstacle{obstacleEnum.HIGH_NODE_MARGIN}.y = obstacle{obstacleEnum.HIGH_NODE}.y + margin.y*[0 1 1 0];


obstacle_lines_res = 100;
O = numel(obstacle);
for o=1:O
    obstacle{o}.x_lines = [];
    obstacle{o}.y_lines = [];
    for k=2:numel(obstacle{o}.x)
        obstacle{o}.x_lines = [obstacle{o}.x_lines linspace(obstacle{o}.x(k-1), obstacle{o}.x(k), obstacle_lines_res)];
        obstacle{o}.y_lines = [obstacle{o}.y_lines linspace(obstacle{o}.y(k-1), obstacle{o}.y(k), obstacle_lines_res)];
    end
    obstacle{o}.x_lines = [obstacle{o}.x_lines linspace(obstacle{o}.x(end), obstacle{o}.x(1), obstacle_lines_res)];
    obstacle{o}.y_lines = [obstacle{o}.y_lines linspace(obstacle{o}.y(end), obstacle{o}.y(1), obstacle_lines_res)];

    [obstacle{o}.theta1,   obstacle{o}.theta2]   = calc.inverse_kinematics(obstacle{o}.x_lines, obstacle{o}.y_lines);
    [obstacle{o}.theta1_alt, obstacle{o}.theta2_alt] = calc.inverse_kinematics(obstacle{o}.x_lines, obstacle{o}.y_lines, 1);
    
    obstacle{o}.theta1 = obstacle{o}.theta1(~isnan(obstacle{o}.theta1));
    obstacle{o}.theta2 = obstacle{o}.theta2(~isnan(obstacle{o}.theta2));
    obstacle{o}.theta1_alt = obstacle{o}.theta1_alt(~isnan(obstacle{o}.theta1_alt));
    obstacle{o}.theta2_alt = obstacle{o}.theta2_alt(~isnan(obstacle{o}.theta2_alt));
end