inchesPerMeter = 100/2.54;

% plot flags
inchesFlag = true;
degreesFlag = true;

xyScale = 1;
if inchesFlag
    xyScale = inchesPerMeter;
end

angleScale = 1;
if degreesFlag
    angleScale = 180/pi;
end

% Read arm config
s = jsondecode(fileread('..\..\src\main\deploy\arm_config.json'));  
shoulder = joint_config(s.shoulder.mass, s.shoulder.length, s.shoulder.moi, s.shoulder.cgRadius, falcon500(s.shoulder.motor.count, s.shoulder.motor.reduction));
elbow = joint_config(s.elbow.mass, s.elbow.length, s.elbow.moi, s.elbow.cgRadius, falcon500(s.elbow.motor.count, s.elbow.motor.reduction));
wrist = joint_config(s.wrist.mass, s.wrist.length, s.wrist.moi, s.wrist.cgRadius, falcon500(s.elbow.motor.count, s.elbow.motor.reduction));
origin.x = s.origin(1);
origin.y = s.origin(2);
arm_length = [s.shoulder.length (s.elbow.length + s.wrist.length)];

kinematics = arm_kinematics(origin.x, origin.y, arm_length);
dynamics = arm_dynamics(shoulder, elbow, wrist);

obstaclesJson = jsondecode(fileread('..\..\src\main\deploy\constraints.json'));

figure(1);
hXY = subplot(1,2,1);
plot_xy_space(hXY, obstaclesJson, xyScale, 'Cartesian');
hAngle = subplot(1,2,2);
plot_angle_space(hAngle, obstaclesJson, kinematics, angleScale, 'Angle Space');
for startIdx = 0:7
    for finalIdx = 0:7

        % Read trajectory
        filename = sprintf('arm_path_%d_%d.json', startIdx, finalIdx);
        s = jsondecode(fileread(['..\..\src\main\deploy\paths\' filename]));
        totalTime = max(s.totalTime, 0);
        theta1 = s.theta1;
        theta2 = s.theta2;
        points = [theta1.'; theta2.'];
    
        % interpolate trajectory to dt
        clockPeriod = 0.020;
        t = 0:clockPeriod:totalTime;
        [position, velocity, acceleration, jerk] = sample(t, totalTime, points);
        [torque, voltage, current] = dynamics.feedforward(position, velocity, acceleration);
    
        [elbow_loc.x, elbow_loc.y, tip_loc.x, tip_loc.y] = kinematics.forward_kinematics(position(1,:), position(2,:));

        plot_xy_path(hXY, origin, elbow_loc, tip_loc, xyScale);
        plot_angle_path(hAngle, position(1,:), position(2,:), angleScale);
    end
end

sgtitle('All Paths')