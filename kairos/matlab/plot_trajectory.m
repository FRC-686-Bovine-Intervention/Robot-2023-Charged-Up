function plot_trajectory(startIdx, finalIdx)

    startIdx = 0;
    finalIdx = 7;

    grannyFactor = 1;
    inchesPerMeter = 39.3701;

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

    % Convert everything to metric for calculations

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


    % Read trajectory
    filename = sprintf('arm_path_%d_%d.json', startIdx, finalIdx);
    s = jsondecode(fileread(['..\..\src\main\deploy\paths\' filename]));
    totalTime = max(s.totalTime, 0) * grannyFactor;
    theta1 = s.theta1;
    theta2 = s.theta2;
    points = [theta1.'; theta2.'];

    % interpolate trajectory to dt
    clockPeriod = 0.020;
    t = 0:clockPeriod:totalTime;
    [position, velocity, acceleration, jerk] = sample(t, totalTime, points);
    [torque, voltage, current] = dynamics.feedforward(position, velocity, acceleration);

    title_str = strrep(filename, '_', '\_');
    fig = figure(1);
    plot_state(fig, t, position, velocity, acceleration, jerk, degreesFlag, title_str);
    fig = figure(2);
    plot_dynamics(fig, t, torque, voltage, current, title_str);

    [elbow_loc.x, elbow_loc.y, tip_loc.x, tip_loc.y] = kinematics.forward_kinematics(position(1,:), position(2,:));
    obstaclesJson = jsondecode(fileread('..\..\src\main\deploy\constraints.json'));

    figure(3);
    plot_xy_space(gca, obstaclesJson, xyScale, title_str);
    plot_xy_path(gca, origin, elbow_loc, tip_loc, xyScale);
            
    figure(4);
    plot_angle_space(gca, obstaclesJson, kinematics, angleScale, title_str);
    plot_angle_path(gca, position(1,:), position(2,:), angleScale);


    function [position, velocity, acceleration, jerk] = sample(time, totalTime, points)
        dt = totalTime / length(points);

        prev1 = floor(time/dt) +1;
        next1 = ceil(time/dt) +1;
        next1(next1==prev1) = next1(next1==prev1)+1;
        prev2 = prev1-1 +1;
        next2 = next1+1 +1;

        prev1 = clamp(prev1, 0, length(points));
        next1 = clamp(next1, 0, length(points));
        prev2 = clamp(prev2, 0, length(points));
        next2 = clamp(next2, 0, length(points));
        
        position(1,:) = interp1(linspace(0,totalTime,length(points)), points(1,:), time, 'linear');
        position(2,:) = interp1(linspace(0,totalTime,length(points)), points(2,:), time, 'linear');

        prevVelocity = (points(:,prev1) - points(:,prev2)) / dt;
        velocity = (points(:,next1) - points(:,prev1)) / dt;
        nextVelocity = (points(:,next2) - points(:,next1)) / dt;

        prevAcceleration = (velocity - prevVelocity) / dt;
        acceleration = (nextVelocity - velocity) / dt;

        jerk = (acceleration - prevAcceleration) / (2*dt);
    end

    function out = clamp(in, low, high)
        out = min(max(in, low), high);
    end

%     function out = interpolate(startValue, endValue, t)
%         out = startValue + (endValue-startValue) * clamp(t, 0, 1);
%     end
end