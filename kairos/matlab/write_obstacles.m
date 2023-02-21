inchesPerMeter = 39.3701;

s = jsondecode(fileread('..\..\src\main\deploy\arm_config.json'));
arm_length(1) = s.shoulder.length * inchesPerMeter;
arm_length(2) = (s.elbow.length + s.wrist.length) * inchesPerMeter;
app.origin.x = s.origin(1) * inchesPerMeter;
app.origin.y = s.origin(2) * inchesPerMeter;
bumper_width_inches = s.bumper_width_inches;
frame_width_inches = s.frame_width_inches;

app.calc = arm_kinematics(app.origin.x, app.origin.y, arm_length);

margin.x = 6;
margin.y = 6;
intake_margin.x = 3;
intake_margin.y = 3;

field = get_field_positions(bumper_width_inches);
[obstacles, obstacleEnum] = get_obstacles(bumper_width_inches, field, margin, intake_margin, app.calc);            

names = fieldnames(obstacleEnum);
enabledObstacles = [obstacleEnum.ROBOT_BODY obstacleEnum.INTAKE obstacleEnum.BONKER_BAR_TM obstacleEnum.MID_NODE obstacleEnum.HIGH_NODE obstacleEnum.MID_NODE_MARGIN obstacleEnum.HIGH_NODE_MARGIN];




json_struct = [];
q.type = 'minX';
q.args = [-1 0] / inchesPerMeter;
json_struct.('no_thru_arm') = q;
q.type = 'maxX';
q.args = [frame_width_inches/2 + 48.0 0] / inchesPerMeter;
json_struct.('max_extension') = q;
q.type = 'minY';
q.args = [0 0] / inchesPerMeter;
json_struct.('floor') = q;
q.type = 'maxY';
q.args = [72 0] / inchesPerMeter;
json_struct.('max_height') = q;
for k = enabledObstacles
    q.type = 'rectangle';
    q.args = [min(obstacles{k}.x) min(obstacles{k}.y) max(obstacles{k}.x) max(obstacles{k}.y)] / inchesPerMeter;
    json_struct.(names{k}) = q;
end

json_txt = jsonencode(json_struct, PrettyPrint=true);

fid = fopen('..\..\src\main\deploy\constraints.json','w');
fprintf(fid, "%s", json_txt);
fclose(fid);