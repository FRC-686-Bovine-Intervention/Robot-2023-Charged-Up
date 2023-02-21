function plot_xy_space(ax, obstaclesJson)

inchesPerMeter = 39.3701;
cmap = colormap(ax, 'lines');
lt_gray = 0.85*[1 1 1];

f = fieldnames(obstaclesJson);
for o=1:numel(f)
    obstacle = obstaclesJson.(f{o});
    if strcmp(obstacle.type, 'rectangle')
        x = [obstacle.args(1) obstacle.args(1) obstacle.args(3) obstacle.args(3)] * inchesPerMeter;
        y = [obstacle.args(2) obstacle.args(4) obstacle.args(4) obstacle.args(2)] * inchesPerMeter;
        p = patch(ax, x, y, cmap(o,:));
        if contains(f{o}, 'MARGIN')
            p.FaceAlpha = 0.25;
        end
    end
end
% axis equal
set(ax, 'PlotBoxAspectRatio',[1 1 1], 'DataAspectRatioMode','auto');
ax.XLim = [-20 68];
ax.YLim = [0 72];
