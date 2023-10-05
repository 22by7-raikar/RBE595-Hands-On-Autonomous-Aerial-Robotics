clear all
close all

load("/MATLAB Drive/user_states.mat")

figure;

plot(time, x, 'b-', 'LineWidth', 2);
hold on;
plot(time, x_des, 'r--', 'LineWidth', 2);

xlabel('Time');
ylabel('Position');

legend('x', 'x_{des}', 'Location', 'Best');
title('Position x');
hold off;

figure;

plot(time, y, 'b-', 'LineWidth', 2);
hold on;
plot(time, y_des, 'r--', 'LineWidth', 2);

xlabel('Time');
ylabel('Position');

legend('y', 'y_{des}', 'Location', 'Best');
title('Position y');
hold off;


figure;

plot(time, z, 'b-', 'LineWidth', 2);
hold on;
plot(time, z_des, 'r--', 'LineWidth', 2);

xlabel('Time');
ylabel('Position');

legend('z', 'z_{des}', 'Location', 'Best');
title('Position z');
hold off;

figure;

plot(time, vx, 'b-', 'LineWidth', 2);
hold on;
plot(time, vx_des, 'r--', 'LineWidth', 2);

xlabel('Time');
ylabel('Velocity');

legend('vx', 'vx_{des}', 'Location', 'Best');
title('Velocity x');
hold off;

figure;

plot(time, vy, 'b-', 'LineWidth', 2);
hold on;
plot(time, vy_des, 'r--', 'LineWidth', 2);

xlabel('Time');
ylabel('Velocity');

legend('vy', 'vy_{des}', 'Location', 'Best');
title('Velocity y');
hold off;

figure;

plot(time, vz, 'b-', 'LineWidth', 2);
hold on;
plot(time, vz_des, 'r--', 'LineWidth', 2);

xlabel('Time');
ylabel('Velocity');

legend('vz', 'vz_{des}', 'Location', 'Best');
title('Velocity z');
hold off;

figure; 
plot3(x, y, z, 'b.'); 
hold on;
plot3(x_des, y_des, z_des, 'r.'); 
title('3D Trajectory Visualization'); 
xlabel('X-Axis'); 
ylabel('Y-Axis');
zlabel('Z-Axis'); 
grid on;

