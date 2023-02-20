s = jsondecode(fileread('..\..\src\main\deploy\arm_config.json'));
arm_length(1) = s.proximal.length * 39.3701;
arm_length(2) = s.distal.length * 39.3701;
app.shoulder.x = s.shoulder(1) * 39.3701;
app.shoulder.y = s.shoulder(2) * 39.3701;
bumper_width_inches = s.bumper_width_inches;
frame_width_inches = s.frame_width_inches;

app.calc = arm_kinematics(app.shoulder.x, app.shoulder.y, arm_length);

margin.x = 3;
margin.y = 3;
intake_margin.x = 3;
intake_margin.y = 3;

field = get_field_positions(bumper_width_inches);
[obstacles, obstacleEnum] = get_obstacles(bumper_width_inches, field, margin, intake_margin, app.calc);            

names = fieldnames(obstacleEnum);
enabledObstacles = [obstacleEnum.INTAKE obstacleEnum.MID_SHELF obstacleEnum.HIGH_SHELF obstacleEnum.MID_SHELF_VERT_MARGIN obstacleEnum.MID_SHELF_HORIZ_MARGIN];
% obstacleEnum.MID_SHELF_VERT_MARGIN  = 7;
% obstacleEnum.MID_SHELF_HORIZ_MARGIN  = 8;
% obstacleEnum.HIGH_SHELF_VERT_MARGIN = 9;
% obstacleEnum.HIGH_SHELF_HORIZ_MARGIN = 10;
% obstacleEnum.BACK_WALL_VERT_MARGIN  = 11;
% obstacleEnum.MID_POLE_VERT_MARGIN   = 12;
% obstacleEnum.HIGH_POLE_VERT_MARGIN  = 13;



json_struct = [];
q.type = 'minX';
q.args = [-1 0];
json_struct.('no_thru_arm') = q;
q.type = 'maxX';
q.args = [frame_width_inches/2 + 48.0 0];
json_struct.('max_extension') = q;
q.type = 'minY';
q.args = [0 0];
json_struct.('floor') = q;
q.type = 'maxY';
q.args = [72 0];
json_struct.('max_height') = q;
for k = enabledObstacles
    q.type = 'rectangle';
    q.args = [min(obstacles{k}.x) min(obstacles{k}.y) max(obstacles{k}.x) max(obstacles{k}.y)];
    json_struct.(names{k}) = q;
end

json_txt = jsonencode(json_struct, PrettyPrint=true);

fid = fopen('..\..\src\main\deploy\constraints.json','w');
fprintf(fid, "%s", json_txt);
fclose(fid);