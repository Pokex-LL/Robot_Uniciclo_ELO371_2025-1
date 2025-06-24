clear; clc;

% Parámetros físicos
r = 0.05;     % radio rueda [m]
L = 0.30;     % separación entre ruedas [m]
dt = 0.01;    % paso [s]
T  = 10;      % tiempo total [s]
t  = 0:dt:T;
N  = length(t); 

%% Velocidades de las ruedas
wL = 27.28 * ones(1, N);
wR = 23.28 * ones(1, N);   % derecha más rápida → curva

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
%% Calcular RMSE

% Tiempos reales desde el CSV
t_real = datos{:,1};

% Rango válido (evita extrapolación fuera de la referencia)
valid_idx = t_real >= t(1) & t_real <= t(end);
t_real_valid = t_real(valid_idx);

% Interpolar trayectoria de referencia en esos tiempos válidos
x_d_interp = interp1(t, x_d, t_real_valid, 'linear');
y_d_interp = interp1(t, y_d, t_real_valid, 'linear');

% Extraer trayectoria real en los mismos puntos
x_real = x(valid_idx);
y_real = y(valid_idx);

% Asegurarse de que todos los vectores son columna
x_real     = x_real(:);
y_real     = y_real(:);
x_d_interp = x_d_interp(:);
y_d_interp = y_d_interp(:);

% Calcular error punto a punto
e = sqrt((x_real - x_d_interp).^2 + (y_real - y_d_interp).^2);

% Calcular RMSE (valor único)
rmse = sqrt(mean(e.^2));

% Mostrar en consola
fprintf('RMSE de la trayectoria: %.4f metros\n', rmse);

% Añadir como texto en el gráfico (opción más segura que annotation)
text(min(x), min(y) - 0.1, sprintf('RMSE = %.4f m', rmse), ...
     'FontSize', 11, 'BackgroundColor', 'w');
%% RMSE de forma 

N_uniform = 1000;

% --- Real: [x, y] ---
xy_real = [x(:), y(:)];
d_real = [0; cumsum(sqrt(sum(diff(xy_real).^2, 2)))];
d_real = d_real / d_real(end);  % normalizar a [0,1]
[~, idx_unique_real] = unique(d_real);
d_real = d_real(idx_unique_real);
xy_real = xy_real(idx_unique_real, :);
xy_real_uniform = interp1(d_real, xy_real, linspace(0,1,N_uniform), 'linear');

% --- Referencia: [x_d, y_d] ---
xy_ref = [x_d(:), y_d(:)];
d_ref = [0; cumsum(sqrt(sum(diff(xy_ref).^2, 2)))];
d_ref = d_ref / d_ref(end);
[~, idx_unique_ref] = unique(d_ref);
d_ref = d_ref(idx_unique_ref);
xy_ref = xy_ref(idx_unique_ref, :);
xy_ref_uniform = interp1(d_ref, xy_ref, linspace(0,1,N_uniform), 'linear');

% --- Centrar ambas trayectorias (quitar desplazamiento)
xy_real_centered = xy_real_uniform - mean(xy_real_uniform);
xy_ref_centered  = xy_ref_uniform  - mean(xy_ref_uniform);

% --- Alinear por rotación (mínimo error cuadrático, sin escala)
H = xy_real_centered' * xy_ref_centered;
[U, ~, V] = svd(H);
R = V * U';
xy_real_aligned = (R * xy_real_centered')';

% --- Calcular error de forma
e_forma = sqrt(sum((xy_real_aligned - xy_ref_centered).^2, 2));
rmse_forma = sqrt(mean(e_forma.^2));

% Mostrar
fprintf('RMSE de forma (sin tiempo, offset ni rotación): %.4f metros\n', rmse_forma);
