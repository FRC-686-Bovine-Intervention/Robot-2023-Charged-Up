function plot_xy_space(ax, field, obstacles, obstacleEnum)

cmap = colormap('lines');
lt_gray = 0.85*[1 1 1];

% h = plot(mygrid.x(~mygrid.blocked), mygrid.y(~mygrid.blocked), 'k.');
% h.Color = lt_gray;
for o=1:numel(obstacles)
    p = patch(ax, obstacles{o}.x, obstacles{o}.y, cmap(o,:));
    if o > obstacleEnum.HIGH_POLE
        p.FaceAlpha = 0.25;
    end
end
% axis equal
set(ax, 'PlotBoxAspectRatio',[1 1 1], 'DataAspectRatioMode','auto');
ax.XLim = [-20 field.back_wall.x];
ax.YLim = [0 72];
