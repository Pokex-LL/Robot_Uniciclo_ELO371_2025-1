clear;
clc;

% Parámetros de simulación y robot
dt = 0.1;            % paso temporal [s]
T  = 30;              % duración de una vuelta [s]
t  = 0:dt:T;

r_wheel = 0.05;       % radio rueda [m]
L_axle  = 0.30;       % separación ruedas [m]

% ---------- Trayectoria CIRCULAR ----------------------------
A       = 2.0;                      % Amplitud sinusoide 
w_traj  = 2*pi / T;                 % rad/s para una vuelta en T

y_d = A * sin(w_traj * t);

[wL, wR] = diffDriveRefs(t, y_d, dt, r_wheel, L_axle);
writematrix(t, 't.csv');
writematrix(wL, 'wL.csv');
writematrix(wR, 'wR.csv');

% Gráficas
figure;
subplot(2,1,1);
plot(t, y_d, 'k', 'LineWidth',1.5);
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Trayectoria Sinusoidal Deseada');

subplot(2,1,2);
plot(t, wL, 'b', t, wR, 'r--', 'LineWidth',1.5);
legend('\omega_L^{ref}','\omega_R^{ref}');
xlabel('Tiempo [s]'); ylabel('\omega [rad/s]');
title('Referencias de velocidad de rueda');
grid on;