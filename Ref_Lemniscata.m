clear; clc;

% Parámetros
dt = 0.05;
T  = 15;
t  = 0:dt:T;

r_wheel = 0.05;     % radio de rueda
L_axle  = 0.3;      % separación de ruedas

% Lemniscata basada en sin(t) y sin(t)*cos(t)
a = 0.8;
x_d = a * sin(t);
y_d = a * sin(t) .* cos(t);  % lemniscata de tipo seno doble

% Cálculo de referencias
[wL, wR, v_d, omega_d, theta_d] = diffDriveRefs(x_d, y_d, dt, r_wheel, L_axle);
writematrix(t, 't.csv');
writematrix(wL, 'wL.csv');
writematrix(wR, 'wR.csv');

% Gráficas
figure;
subplot(2,1,1);
plot(x_d, y_d, 'k', 'LineWidth', 1.5);
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Trayectoria Lemniscata (∞)');

subplot(2,1,2);
plot(t, wL, 'b', t, wR, 'r--', 'LineWidth', 1.5);
legend('\omega_L^{ref}','\omega_R^{ref}');
xlabel('Tiempo [s]'); ylabel('\omega [rad/s]');
title('Velocidades de ruedas');
grid on;
