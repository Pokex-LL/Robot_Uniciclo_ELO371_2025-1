clear; clc;

% Parámetros físicos
r = 0.05;     % radio rueda [m]
L = 0.30;     % separación entre ruedas [m]
%dt = 0.01;    % paso [s]
%dt = 0.003699;
T  = 15;      % tiempo total [s]
%t  = 0:dt:T;
N  = length(t); 

%% Velocidades de las ruedas
% wL = 27.28 * ones(1, N);
% wR = 23.28 * ones(1, N);   % derecha más rápida → curva
tiempo = readtable('der_izq.csv');
t = table2array(tiempo(:, 1);
Izq = readtable('der_izq.csv');
wL = table2array(Izq(:, 3));
Der = readtable('der_izq.csv');
wR = table2array(Der(:, 2));
%% Cinemática del cuerpo
v     = r/2 * (wR + wL);        % velocidad lineal
omega = r/L * (wR - wL);        % velocidad angular

% Inicialización

% Vectores que guardan la pose en cada instante de tiempo
x = zeros(1, N); 
y = zeros(1, N); 
th = zeros(1, N);

% Pose inicial
x(1) = 0; 
y(1) = 0; 
th(1) = pi/4; %orientación medido desde eje x+

% Integración de Euler :( perdon
for k = 1:N-1
    x(k+1)  = x(k) + v(k)*cos(th(k))*dt;
    y(k+1)  = y(k) + v(k)*sin(th(k))*dt;
    th(k+1) = th(k) + omega(k)*dt;
end



%% Animación
figure;
axis equal; grid on;
xlim([-1 11]); ylim([-1 11]);
xlabel('x [m]'); ylabel('y [m]');
title('Animación del robot diferencial');
hold on;

% Inicializar gráficos
traj_plot = plot(x(1), y(1), 'r', 'LineWidth', 1.5);         % trayectoria
robot_circle_plot = plot(NaN, NaN, 'b', 'LineWidth', 2);     % cuerpo
robot_heading_plot = plot(NaN, NaN, '-', 'Color', [0.5 0 0.8], 'LineWidth', 2); % orientación naranja

% Forma del robot
theta_circle = linspace(0, 2*pi, 30);
radius = 0.1;
circle_x = radius * cos(theta_circle);
circle_y = radius * sin(theta_circle);
heading_len = 0.1;  % más corto
heading_x = [0, heading_len];
heading_y = [0, 0];

for k = 1:20:N
    % Pose actual
    xk = x(k);
    yk = y(k);
    thk = th(k);

    R_theta = [cos(thk), -sin(thk); sin(thk), cos(thk)];

    % Círculo
    circ = R_theta * [circle_x; circle_y] + [xk; yk];
    set(robot_circle_plot, 'XData', circ(1,:), 'YData', circ(2,:));

    % Flecha orientación
    heading = R_theta * [heading_x; heading_y] + [xk; yk];
    set(robot_heading_plot, 'XData', heading(1,:), 'YData', heading(2,:));

    % Trazo
    set(traj_plot, 'XData', x(1:k), 'YData', y(1:k));

    drawnow;
    pause(dt * 1);
end
