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

theta1 = [-150.0  -58.8  -20.9];
theta2 = [ -36.4   46.9   35.5];

points = [theta1; theta2];

f = cscvn(points);

figure(99);
cla;
fnplt(f, 2);
hold on;
plot(theta1,theta2,'ro');
grid on;
xlabel('\theta_1 (deg)');
ylabel('\theta_2 (deg)');


Vmax = 360; % deg/sec
Amax = Vmax / 0.3;

[breaks,l,d] = ppbrk(f,'b','l','d');
npoints = 101;
u = linspace(0,1,npoints);
x = linspace(breaks(1),breaks(l+1),npoints);
P = ppual(f,x);

figure(100);
plot(P(1,:),P(2,:));

% calculate the arc-length along each segment
dL = sqrt((P(1,2:end) - P(1,1:end-1)).^2 + (P(2,2:end) - P(2,1:end-1)).^2);
dL(end+1) = dL(end);    % extend to keep same length
L = sum(dL);

%==========================================================
% Find Velocity along Spline
% Numerical Integration Approach
%==========================================================

%==========================================================
% Find velocity constraints at planning points along path
%==========================================================

V_limit = Vmax * ones(size(u));   

% from beginning, calc maximum achieveable velocity
V = zeros(size(u));
V(1) = 0;
for uu=2:length(u)
    % Vf^2 = Vi^2 + 2*A*(Xf-Xi)
    V(uu) = sqrt( V(uu-1)^2 + 2*Amax*dL(uu) );
    if V(uu) >= V_limit(uu)
        V(uu) = V_limit(uu);
    end
end

% repeat process in reverse from end, calc maximum achieveable velocity
V_limit = V;
V(end) = 0;
for uu=length(u)-1:-1:1
    % Vf^2 = Vi^2 + 2*A*(Xf-Xi)
    V(uu) = sqrt( V(uu+1)^2 + 2*Amax*dL(uu) );
    if V(uu) >= V_limit(uu)
        V(uu) = V_limit(uu);
    end
end

% estimate time to traverse path
N = length(dL);
dt = 2*dL(1:N-1) ./ (V(1:N-1) + V(2:N));
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

points = deg2rad(Pi);

% interpolate trajectory to dt
clockPeriod = 0.020;
t = 0:clockPeriod:T;
[position, velocity, acceleration, jerk] = sample(t, T, points);
[torque, voltage, current] = dynamics.feedforward(position, velocity, acceleration);

title_str = 'test spline';
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

    