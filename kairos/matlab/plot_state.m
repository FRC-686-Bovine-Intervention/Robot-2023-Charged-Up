function plot_state(fig, t, position, velocity, acceleration, jerk, degreesFlag, title_str)

figure(fig);
ax(1)=subplot(411);
if degreesFlag
    plot(t, rad2deg(position));
    ylabel('\theta (deg)');
else
    plot(t, position);
    ylabel('\theta (rad)');
end
grid on;
legend('Shoulder','Elbow');
title(title_str);
ax(2)=subplot(412);
if degreesFlag
    plot(t, rad2deg(velocity));
    ylabel('\omega (deg/sec)');
else
    plot(t, velocity);
    ylabel('\omega (rad/sec)');
end
grid on;
ax(3)=subplot(413);
if degreesFlag
    plot(t, rad2deg(acceleration));
    ylabel('\alpha (deg/sec^2)');
else
    plot(t, acceleration);
    ylabel('\alpha (rad/sec^2)');
end
grid on;
ax(4)=subplot(414);
plot(t, jerk);
grid on;
ylabel('Jerk (rad/sec^3)');
linkaxes(ax,'x');