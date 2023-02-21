function plot_angle_space(ax, field, obstacles, obstacleEnum)

cmap = colormap(ax, 'lines');
lt_gray = 0.85*[1 1 1];

% h = plot(mygrid.theta1(~mygrid.blocked), mygrid.theta2(~mygrid.blocked), 'k.');
% h.Color = lt_gray;
for o=1:numel(obstacles)
    p = patch(ax, obstacles{o}.theta1, obstacles{o}.theta2, cmap(o,:));
    q = patch(ax, obstacles{o}.theta1_alt, obstacles{o}.theta2_alt, cmap(o,:));
    if o > obstacleEnum.HIGH_POLE
        p.FaceAlpha = 0.25;
        q.FaceAlpha = 0.25;
    end
end
ax.XLim = [-180 0];
ax.YLim = [-180 180];
% grid on;
set(ax, 'PlotBoxAspectRatio',[1 1 1], 'DataAspectRatioMode','auto');
h = ax.XLabel;
h.String = '\theta_1 (deg)';
h = ax.YLabel;
h.String = '\theta_2 (deg)';
