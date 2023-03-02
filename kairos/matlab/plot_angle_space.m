function plot_angle_space(ax, obstaclesJson, arm_kinematics, angleScale, title_str)

cmap = colormap(ax, 'lines');
lt_gray = 0.85*[1 1 1];

obstacle_lines_res = 100;

if angleScale == 1
    units = 'rad';
else
    units = 'deg';
end

cla(ax);

f = fieldnames(obstaclesJson);
for o=1:numel(f)
    obstacle = obstaclesJson.(f{o});
    if strcmp(obstacle.type, 'rectangle')
        x = [linspace(obstacle.args(1), obstacle.args(1), obstacle_lines_res) ...
             linspace(obstacle.args(1), obstacle.args(3), obstacle_lines_res) ...
             linspace(obstacle.args(3), obstacle.args(3), obstacle_lines_res) ...
             linspace(obstacle.args(3), obstacle.args(1), obstacle_lines_res)];
        y = [linspace(obstacle.args(2), obstacle.args(4), obstacle_lines_res) ...
             linspace(obstacle.args(4), obstacle.args(4), obstacle_lines_res) ...
             linspace(obstacle.args(4), obstacle.args(2), obstacle_lines_res) ...
             linspace(obstacle.args(2), obstacle.args(2), obstacle_lines_res)];

        [theta1, theta2]   = arm_kinematics.inverse_kinematics(x,y);
        [theta1_alt, theta2_alt] = arm_kinematics.inverse_kinematics(x,y,1);
        
        theta1 = theta1(~isnan(theta1)) * angleScale;
        theta2 = theta2(~isnan(theta2)) * angleScale;
        theta1_alt = theta1_alt(~isnan(theta1_alt)) * angleScale;
        theta2_alt = theta2_alt(~isnan(theta2_alt)) * angleScale;

        p = patch(ax, theta1, theta2, cmap(o,:));
        q = patch(ax, theta1_alt, theta2_alt, cmap(o,:));
        if contains(f{o}, 'MARGIN')
            p.FaceAlpha = 0.25;
            q.FaceAlpha = 0.25;
        end
    end
end
ax.XLim = [-pi 0] * angleScale;
ax.YLim = [-pi pi] * angleScale;
grid(ax, 'on');
set(ax, 'PlotBoxAspectRatio',[1 1 1], 'DataAspectRatioMode','auto');
h = ax.XLabel;
h.String = ['\theta_1 (' units ')'];
h = ax.YLabel;
h.String = ['\theta_2 (' units ')'];
title(ax, title_str);