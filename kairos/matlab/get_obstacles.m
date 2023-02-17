function [obstacle, obstacleEnum] = get_obstacles(center_to_side_bumper, fieldPos, margin, intake_margin, calc)
%=====================================
% Define obstacles
%=====================================

obstacleEnum.ROBOT_BODY          = 1;
obstacleEnum.INTAKE              = 2;
obstacleEnum.MID_SHELF           = 3;
obstacleEnum.HIGH_SHELF          = 4;
obstacleEnum.MID_POLE            = 5;
obstacleEnum.HIGH_POLE           = 6;
obstacleEnum.MID_SHELF_VERT_MARGIN  = 7;
obstacleEnum.MID_SHELF_HORIZ_MARGIN  = 8;
obstacleEnum.HIGH_SHELF_VERT_MARGIN = 9;
obstacleEnum.HIGH_SHELF_HORIZ_MARGIN = 10;
obstacleEnum.BACK_WALL_VERT_MARGIN  = 11;
obstacleEnum.MID_POLE_VERT_MARGIN   = 12;
obstacleEnum.HIGH_POLE_VERT_MARGIN  = 13;


obstacle{obstacleEnum.ROBOT_BODY}.x = [-center_to_side_bumper    -center_to_side_bumper  +center_to_side_bumper  +center_to_side_bumper];
obstacle{obstacleEnum.ROBOT_BODY}.y = [0.0                       9.0 + intake_margin.y     9.0 + intake_margin.y     0.0];

obstacle{obstacleEnum.INTAKE}.x = [6.5    6.5-intake_margin.x       9.0 + intake_margin.x    12.0 + intake_margin.x];
obstacle{obstacleEnum.INTAKE}.y = [0.0   18.0 + intake_margin.y    18.0 + intake_margin.y     0.0];

obstacle{obstacleEnum.MID_SHELF}.x =  [fieldPos.mid_shelf_front_corner.x  fieldPos.mid_shelf_front_corner.x    fieldPos.back_wall.x     fieldPos.back_wall.x];
obstacle{obstacleEnum.MID_SHELF}.y =  [0                         fieldPos.mid_shelf.y                 fieldPos.mid_shelf.y     0];

obstacle{obstacleEnum.HIGH_SHELF}.x =  [fieldPos.high_shelf_front_corner.x fieldPos.high_shelf_front_corner.x   fieldPos.back_wall.x     fieldPos.back_wall.x];
obstacle{obstacleEnum.HIGH_SHELF}.y =  [0                         fieldPos.high_shelf.y                fieldPos.high_shelf.y    0];

obstacle{obstacleEnum.MID_POLE}.x = [fieldPos.mid_pole.x - fieldPos.pole_dia/2     fieldPos.mid_pole.x - fieldPos.pole_dia/2     fieldPos.mid_pole.x + fieldPos.pole_dia/2     fieldPos.mid_pole.x + fieldPos.pole_dia/2];
obstacle{obstacleEnum.MID_POLE}.y = [0                           fieldPos.mid_pole.y                  fieldPos.mid_pole.y                  0];

obstacle{obstacleEnum.HIGH_POLE}.x = [fieldPos.high_pole.x - fieldPos.pole_dia/2   fieldPos.high_pole.x - fieldPos.pole_dia/2    fieldPos.high_pole.x + fieldPos.pole_dia/2   fieldPos.high_pole.x + fieldPos.pole_dia/2];
obstacle{obstacleEnum.HIGH_POLE}.y = [0                          fieldPos.high_pole.y                 fieldPos.high_pole.y                 0];

obstacle{obstacleEnum.MID_SHELF_VERT_MARGIN}.x = fieldPos.mid_shelf_front_corner.x + margin.x * [-1 -1 0 0];
obstacle{obstacleEnum.MID_SHELF_VERT_MARGIN}.y = (fieldPos.mid_shelf.y + margin.y) * [0 1 1 0];

obstacle{obstacleEnum.MID_SHELF_HORIZ_MARGIN}.x = [fieldPos.mid_shelf_front_corner.x fieldPos.mid_shelf_front_corner.x fieldPos.high_shelf_front_corner.x fieldPos.high_shelf_front_corner.x];
obstacle{obstacleEnum.MID_SHELF_HORIZ_MARGIN}.y = fieldPos.mid_shelf.y + margin.y* [0 1 1 0];

obstacle{obstacleEnum.HIGH_SHELF_VERT_MARGIN}.x = fieldPos.high_shelf_front_corner.x + margin.x * [-1 -1 0 0];
obstacle{obstacleEnum.HIGH_SHELF_VERT_MARGIN}.y = [fieldPos.mid_shelf.y fieldPos.high_shelf.y+margin.y fieldPos.high_shelf.y+margin.y fieldPos.mid_shelf.y];

obstacle{obstacleEnum.HIGH_SHELF_HORIZ_MARGIN}.x = [fieldPos.high_shelf_front_corner.x fieldPos.high_shelf_front_corner.x fieldPos.back_wall.x fieldPos.back_wall.x];
obstacle{obstacleEnum.HIGH_SHELF_HORIZ_MARGIN}.y = fieldPos.high_shelf.y + margin.y * [0 1 1 0];

obstacle{obstacleEnum.BACK_WALL_VERT_MARGIN}.x = fieldPos.back_wall.x + margin.x * [-1 -1 0 0];
obstacle{obstacleEnum.BACK_WALL_VERT_MARGIN}.y = fieldPos.high_shelf.y + 24 * [0 1 1 0];

obstacle{obstacleEnum.MID_POLE_VERT_MARGIN}.x =  fieldPos.mid_pole.x + [-2 -2 -1 -1]*margin.x;
obstacle{obstacleEnum.MID_POLE_VERT_MARGIN}.y =  (fieldPos.mid_pole.y + margin.y) * [0 1 1 0];

obstacle{obstacleEnum.HIGH_POLE_VERT_MARGIN}.x =  fieldPos.high_pole.x + [-2 -2 -1 -1]*margin.x;
obstacle{obstacleEnum.HIGH_POLE_VERT_MARGIN}.y =  (fieldPos.high_pole.y + margin.y) * [0 1 1 0];

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

    [obstacle{o}.theta1,   obstacle{o}.theta2]   = calc.reverse_kinematics(obstacle{o}.x_lines, obstacle{o}.y_lines);
    [obstacle{o}.theta1_alt, obstacle{o}.theta2_alt] = calc.reverse_kinematics(obstacle{o}.x_lines, obstacle{o}.y_lines, 1);
    
    obstacle{o}.theta1 = obstacle{o}.theta1(~isnan(obstacle{o}.theta1));
    obstacle{o}.theta2 = obstacle{o}.theta2(~isnan(obstacle{o}.theta2));
    obstacle{o}.theta1_alt = obstacle{o}.theta1_alt(~isnan(obstacle{o}.theta1_alt));
    obstacle{o}.theta2_alt = obstacle{o}.theta2_alt(~isnan(obstacle{o}.theta2_alt));
end