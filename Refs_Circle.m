clear;
clc;
% Parámetros de simulación y robot
dt = 0.01;            % paso temporal [s]
T  = 10;              % duración de una vuelta [s]
t  = 0:dt:T;

r_wheel = 0.05;       % radio rueda [m]
L_axle  = 0.30;       % separación ruedas [m]

% ---------- Trayectoria CIRCULAR ----------------------------
R       = 2.0;                      % radio del círculo [m]
w_traj  = 2*pi / T;                 % rad/s para una vuelta en T

x_d = R * cos(w_traj * t);
y_d = R * sin(w_traj * t);

[wL, wR] = diffDriveRefs(x_d, y_d, dt, r_wheel, L_axle);

% Gráficas
figure;
subplot(2,1,1);
plot(x_d, y_d, 'k', 'LineWidth',1.5);
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Trayectoria circular deseada');

subplot(2,1,2);
plot(t, wL, 'b', t, wR, 'r--', 'LineWidth',1.5);
legend('\omega_L^{ref}','\omega_R^{ref}');
xlabel('Tiempo [s]'); ylabel('\omega [rad/s]');
title('Referencias de velocidad de rueda');
grid on;