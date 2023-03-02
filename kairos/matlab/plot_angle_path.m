function h = plot_angle_path(ax, theta1, theta2, angleScale)

theta1 = theta1 * angleScale;
theta2 = theta2 * angleScale;

h(1) = line(ax, theta1, theta2, 'Color', 'r', 'Linestyle', '--');
h(2) = line(ax, theta1(1), theta2(1), 'Color', 'r', 'Marker', 'x');
h(3) = line(ax, theta1(end), theta2(end), 'Color', 'r', 'Marker', 'x');
