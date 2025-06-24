% ------------------------------------------------------------ 
% Seguimiento de múltiples puntos con control PID + orientación
% ------------------------------------------------------------
clear; clc; close all;

%% Parámetros
dt = 0.05;     % paso temporal
T = 40;        % duración máxima
t = 0:dt:T;
N = length(t);
L = 0.3;  % distancia entre ruedas (en metros)

%% Inicialización adicional
vL_hist = [];
vR_hist = [];

%% Waypoints
A = 4;          % Amplitud de la onda
f = 2 * pi / 5;   % Frecuencia (1 ciclo cada 5 segundos aprox)
x_vals = 0:0.1:5; % Desde x = 0 hasta x = 5
y_vals = A * sin(f * x_vals);

waypoints = [x_vals', y_vals'];
n_points = size(waypoints, 1);

%% Ganancias PID
Kp_lin = 3.0;
Ki_lin = 0.08;
Kd_lin = 0.05;

Kp_ang = 8.0;
Ki_ang = 0.1;
Kd_ang = 0.2;

tolerance = 0.02;

%% Inicialización
x = 0; y = 0; th = 0;
x_hist = []; y_hist = []; th_hist = [];
v_hist = []; omega_hist = [];

int_e_lin = 0; prev_e_lin = 0;
int_e_ang = 0; prev_e_ang = 0;

current_wp = 1;

%% Simulación
for k = 1:N
    if current_wp > n_points
        break;
    end

    % Punto objetivo actual
    x_d = waypoints(current_wp, 1);
    y_d = waypoints(current_wp, 2);

    % Error
    dx = x_d - x;
    dy = y_d - y;
    dist = sqrt(dx^2 + dy^2);
    theta_des = atan2(dy, dx);
    e_theta = wrapToPi(theta_des - th);

    % PID angular
    int_e_ang = int_e_ang + e_theta * dt;
    der_e_ang = (e_theta - prev_e_ang) / dt;
    omega = Kp_ang * e_theta + Ki_ang * int_e_ang + Kd_ang * der_e_ang;
    prev_e_ang = e_theta;

    % PID lineal (solo si está bien orientado)
    if abs(e_theta) < pi/6
        int_e_lin = int_e_lin + dist * dt;
        der_e_lin = (dist - prev_e_lin) / dt;
        v = Kp_lin * dist + Ki_lin * int_e_lin + Kd_lin * der_e_lin;
        prev_e_lin = dist;
    else
        v = 0;
    end
        % Velocidades de ruedas
    v_R = v + (L/2) * omega;
    v_L = v - (L/2) * omega;
    
    vR_hist(end+1) = v_R;
    vL_hist(end+1) = v_L;


    % Integración
    x = x + v * cos(th) * dt;
    y = y + v * sin(th) * dt;
    th = th + omega * dt;

    % Guardar historial
    x_hist(end+1) = x;
    y_hist(end+1) = y;
    th_hist(end+1) = th;
    v_hist(end+1) = v;
    omega_hist(end+1) = omega;

    % Siguiente waypoint si se alcanzó
    if dist < tolerance
        current_wp = current_wp + 1;
    end
end

%% Animación
figure;
axis equal; grid on;
xlim([-1.5 6]); ylim([-5 5]);
xlabel('x [m]'); ylabel('y [m]');
title('Seguimiento de puntos con control PID');
hold on;

% Waypoints
plot(waypoints(:,1), waypoints(:,2), 'go--', 'MarkerFaceColor', 'g');

% Inicialización gráfica
traj_plot = plot(NaN, NaN, 'b', 'LineWidth', 1.5);
robot_circle = plot(NaN, NaN, 'r', 'LineWidth', 2);
robot_heading = plot(NaN, NaN, '-', 'Color', [1 0.5 0], 'LineWidth', 2);

theta_circle = linspace(0, 2*pi, 30);
radius = 0.1;
circle_x = radius * cos(theta_circle);
circle_y = radius * sin(theta_circle);
arrow_len = 0.08;
arrow_x = [0, arrow_len]; arrow_y = [0, 0];

for k = 1:10:length(x_hist)
    xk = x_hist(k);
    yk = y_hist(k);
    thk = th_hist(k);
    R_th = [cos(thk), -sin(thk); sin(thk), cos(thk)];

    circ = R_th * [circle_x; circle_y] + [xk; yk];
    set(robot_circle, 'XData', circ(1,:), 'YData', circ(2,:));

    arrow = R_th * [arrow_x; arrow_y] + [xk; yk];
    set(robot_heading, 'XData', arrow(1,:), 'YData', arrow(2,:));

    set(traj_plot, 'XData', x_hist(1:k), 'YData', y_hist(1:k));
    drawnow;
end

%% Gráficas de velocidades
figure;
subplot(2,1,1);
plot(1:length(v_hist), v_hist, 'b', 'LineWidth', 1.5);
ylabel('Velocidad lineal v [m/s]');
grid on;
title('Evolución de velocidades');

subplot(2,1,2);
plot(1:length(omega_hist), omega_hist, 'r', 'LineWidth', 1.5);
ylabel('Velocidad angular \omega [rad/s]');
xlabel('Iteración');
grid on;

%% Gráficas de velocidades de ruedas
figure;
plot(vR_hist, 'r', 'LineWidth', 1.5); hold on;
plot(vL_hist, 'b', 'LineWidth', 1.5);
legend('Rueda Derecha', 'Rueda Izquierda');
xlabel('Iteración'); ylabel('Velocidad [m/s]');
title('Velocidades de ruedas');
grid on;
