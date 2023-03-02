function h = plot_xy_path(ax, origin, elbow, tip, xyScale)

origin.x = origin.x * xyScale; 
origin.y = origin.y * xyScale; 
elbow.x = elbow.x * xyScale; 
elbow.y = elbow.y * xyScale; 
tip.x = tip.x * xyScale; 
tip.y = tip.y * xyScale; 

h(1) = line(ax, origin.x, origin.y, 'Color', 'g', 'Marker', 'x');
h(2) = line(ax, elbow.x, elbow.y, 'Color', 'b', 'Linestyle', '--');
h(3) = line(ax, elbow.x(1), elbow.y(1), 'Color', 'b', 'Marker', 'x');
h(4) = line(ax, elbow.x(end), elbow.y(end), 'Color', 'b', 'Marker', 'x');
h(5) = line(ax, tip.x, tip.y, 'Color', 'r', 'Linestyle', '--', 'Marker', '.');
h(6) = line(ax, tip.x(1), tip.y(1), 'Color', 'r', 'Marker', 'x');
h(7) = line(ax, tip.x(end), tip.y(end), 'Color', 'r', 'Marker', 'x');
