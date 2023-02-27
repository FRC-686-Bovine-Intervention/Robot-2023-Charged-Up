function plot_dynamics(fig, t, torque, voltage, current, title_str)

figure(fig);
ax(1)=subplot(311);
plot(t, torque);
grid on;
ylabel('Torque (Nm)');
legend('Shoulder','Elbow');
title(title_str);
ax(2)=subplot(312);
plot(t, voltage);
grid on;
ylabel('Voltage (V)');
ax(3)=subplot(313);
plot(t, current);
grid on;
ylabel('Current (A)')
linkaxes(ax,'x');
