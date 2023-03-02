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


% Read trajectory
startIdx = 1;
finalIdx = 4;
filename = sprintf('arm_path_%d_%d.json', startIdx, finalIdx);
s = jsondecode(fileread(['..\..\src\main\deploy\paths\' filename]));
orig_T = max(s.totalTime, 0) * grannyFactor;
orig_points = [s.theta1.'; s.theta2.'];
% remove extraneous loop
orig_points = orig_points(:,[1 6:end]);
orig_theta1 = orig_points(1,:);
orig_theta2 = orig_points(2,:);

theta1 = [orig_theta1(1)*180/pi  -118.2  orig_theta1(end)*180/pi] * pi/180;
theta2 = [orig_theta2(1)*180/pi     8.2  orig_theta2(end)*180/pi] * pi/180;

points = [theta1; theta2];

f = cscvn(points);
[breaks,l,d] = ppbrk(f,'b','l','d');
npoints = 101;
u = linspace(0,1,npoints);
x = linspace(breaks(1),breaks(l+1),npoints);
P = ppual(f,x);

figure(99);
cla;
plot(orig_theta1*180/pi,orig_theta2*180/pi,'b--');
hold on;
plot(P(1,:)*180/pi,P(2,:)*180/pi,'-r');
plot(theta1*180/pi,theta2*180/pi,'ro');
hold off;
grid on;
xlabel('\theta_1 (deg)');
ylabel('\theta_2 (deg)');
legend('Optimizer', 'Spline', 'Spline Ctrl Pts', 'location','best');

Vmax = 250*pi/180; % rad/sec
Amax = Vmax / 0.25;
Jmax = Amax / 0.25;


% figure(100);
% plot(P(1,:),P(2,:));

% calculate the arc-length along each segment
dL = abs(P(:,2:end) - P(:,1:end-1));
dL(:,end+1) = dL(:,end);    % extend to keep same length
L = sum(dL,2);

%==========================================================
% Find Velocity along Spline
% Numerical Integration Approach
%==========================================================

%==========================================================
% Find velocity constraints at planning points along path
%==========================================================

A_limit = Amax * ones(2,length(u));   
V_limit = Vmax * ones(2,length(u));   

% from beginning, calc maximum achieveable velocity
Vfwd = zeros(2,length(u));
Vrev = zeros(2,length(u));
Vfwd(:,1) = 0;
for uu=2:length(u)
    % Vf^2 = Vi^2 + 2*A*(Xf-Xi)
    Vfwd(:,uu) = sqrt( Vfwd(:,uu-1).^2 + 2*A_limit(:,uu-1).*dL(:,uu-1) );
    r = Vfwd(:,uu) ./ V_limit(:,uu);
    if max(r) >= 1
        Vfwd(:,uu) = Vfwd(:,uu)/max(r);    % decrease both velocities to meet both limits
    end
end

% repeat process in reverse from end, calc maximum achieveable velocity
for uu=length(u)-1:-1:1
    % Vf^2 = Vi^2 + 2*A*(Xf-Xi)
    Vrev(:,uu) = sqrt( Vrev(:,uu+1).^2 + 2*A_limit(:,uu).*dL(:,uu) );
    r = Vrev(:,uu) ./ V_limit(:,uu);
    if max(r) >= 1
        Vrev(:,uu) = Vrev(:,uu)/max(r);    % decrease both velocities to meet both limits
    end
end

V = min(Vfwd, Vrev);

figure(100);
subplot(211);
plot(u,Vfwd.',u,Vrev.');
subplot(212);
plot(u,V.');




% estimate time to traverse path
N = length(dL);
dt = 2*dL(:,1:N-1) ./ (V(:,1:N-1) + V(:,2:N));
dt = max(dt,[],1);
t = [0 cumsum(dt)];
T = t(end);

% uniform sample in time
ti = unique([0:0.020:T, T]);
Pi = [];
for k=1:2
    Pi(k,:) = interp1(t,P(k,:),ti);
end


% 
% Vi = interp1(t,V,ti);
% 
% figure(101);
% subplot(211);
% plot(ti, Pi);
% subplot(212);
% plot(ti, Vi);

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

points = Pi;

% interpolate trajectory to dt
clockPeriod = 0.020;
t = 0:clockPeriod:orig_T;
[position, velocity, acceleration, jerk] = sample(t, orig_T, orig_points);
[torque, voltage, current] = dynamics.feedforward(position, velocity, acceleration);

title_str = 'Original 1-4 Path';
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

    

clockPeriod = 0.020;
t = 0:clockPeriod:T;
[position, velocity, acceleration, jerk] = sample(t, T, points);
[torque, voltage, current] = dynamics.feedforward(position, velocity, acceleration);

title_str = 'test spline';
fig = figure(11);
plot_state(fig, t, position, velocity, acceleration, jerk, degreesFlag, title_str);
fig = figure(12);
plot_dynamics(fig, t, torque, voltage, current, title_str);

[elbow_loc.x, elbow_loc.y, tip_loc.x, tip_loc.y] = kinematics.forward_kinematics(position(1,:), position(2,:));
obstaclesJson = jsondecode(fileread('..\..\src\main\deploy\constraints.json'));

figure(13);
plot_xy_space(gca, obstaclesJson, xyScale, title_str);
plot_xy_path(gca, origin, elbow_loc, tip_loc, xyScale);
        
figure(14);
plot_angle_space(gca, obstaclesJson, kinematics, angleScale, title_str);
plot_angle_path(gca, position(1,:), position(2,:), angleScale);

    