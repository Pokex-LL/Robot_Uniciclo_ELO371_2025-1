clear;
clc;

dt = 0.01; 
T = 10;
t = 0:dt:10;
V  = 1;           % m/s sobre cada eje
x_d = V * t;
y_d = V * t;

[wL, wR] = diffDriveRefs(x_d, y_d, dt, 0.05, 0.30);

% Gráficas
figure;
subplot(2,1,1);
xlim([0 10]); ylim([27 29]);
plot(x_d, y_d, 'k', 'LineWidth',1.5);
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Trayectoria circular deseada');

subplot(2,1,2);
xlim([-3 3]); ylim([-1 4]);
plot(t, wL, 'b', t, wR, 'r--', 'LineWidth',1.5);
legend('\omega_L^{ref}','\omega_R^{ref}');
xlabel('Tiempo [s]'); ylabel('\omega [rad/s]');
title('Referencias de velocidad de rueda');
grid on;