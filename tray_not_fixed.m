clear; clc;

%% Parámetros físicos
r = 0.025;     % radio rueda [m]
L = 0.07;     % separación entre ruedas [m]

%% Lectura de datos desde CSV
datos = readtable('data_LEM_ambas_ruedas.csv');
t  = table2array(datos(:, 1));   % Tiempo [s]
wR = table2array(datos(:, 2));   % Velocidad angular rueda derecha [rad/s]
wL = table2array(datos(:, 3));   % Velocidad angular rueda izquierda [rad/s]
N  = length(t);                  % Número de muestras

%% Cálculo del paso de tiempo variable
dt = diff(t);
dt(end+1) = dt(end);  % Repetimos el último valor para igualar longitud

%% Cinemática del cuerpo
v = r/2 * (wR + wL);        % Velocidad lineal
omega = r/L * (wR - wL);        % Velocidad angular

% Inicialización de vectores de estado
x  = zeros(1, N); 
y  = zeros(1, N); 
th = zeros(1, N);

% Pose inicial
x(1) = 0; 
y(1) = 0; 
th(1) = pi/4; % orientación inicial (rad)

%% Integración de Euler
for k = 1:N-1
    x(k+1)  = x(k) + v(k)*cos(th(k))*dt(k);
    y(k+1)  = y(k) + v(k)*sin(th(k))*dt(k);
    th(k+1) = th(k) + omega(k)*dt(k);
end


%EXTRA TRAYECTORIAS REF
%% Trayectoria de referencia (círculo)
dt = 0.01;

% T  = 20;      
% t  = 0:dt:T;
% R  = 2;
% w0 = 2*pi/T;
% xc = 0; yc = 2;
% x_d = xc + R*cos(w0*t);
% y_d = yc + R*sin(w0*t);

% Trayectoria parametrizada: SINUSOIDE
% T  = 15;              
% t  = 0:dt:T;
% A  = 1;                       
% w_traj  = 2*pi/(T/1.5);           
% y_d = A*sin(w_traj * t);
% x_d = t;

% Lemniscata basada en sin(t) y sin(t)*cos(t)
T  = 10;
t  = 0:dt:T;
a = 0.5;
x_d = a * sin(t);
y_d = a * sin(t) .* cos(t);

%recta
% T  = 20;
% t  = 0:dt:T;
% V  = 0.5;           
% x_d = V * t;         
% y_d = V * t;     


%% Animación
% --- dimensiones físicas (m) ---
body_L = 0.10;  % largo del robot (X)
body_W = 0.10;  % ancho del robot (Y)

sensor_W = 0.02;  % ancho del sensor verde
sensor_H = 0.08;  % alto del sensor
sensor_offset = 0.04;  % desplazamiento al frente

wheel_L = 0.04; % largo rueda (eje X)
wheel_W = 0.015; % ancho rueda (eje Y)
wheel_offset = body_W/2 + wheel_W/2;

% --- geometría local ---
body_xy = 0.5 * [-body_L -body_W;
                  body_L -body_W;
                  body_L  body_W;
                 -body_L  body_W]';

% sensor al frente del robot
sensor_xy = 0.5 * [-sensor_W -sensor_H;
                    sensor_W -sensor_H;
                    sensor_W  sensor_H;
                   -sensor_W  sensor_H]';
sensor_xy = sensor_xy + [sensor_offset; 0];

wheel_rect = 0.5 * [-wheel_L -wheel_W;
                    wheel_L -wheel_W;
                    wheel_L  wheel_W;
                   -wheel_L  wheel_W]';

L_pos = [0; -wheel_offset];  % izquierda
R_pos = [0;  wheel_offset];  % derecha

% --- inicializar figura ---
figure; axis equal; grid on; hold on;
xlabel('x [m]'); ylabel('y [m]');
title('Robot uniciclo – Trayectoria Recta');
h2 = plot(x_d, y_d, 'b--', 'LineWidth', 2);  % referencia (azul punteado)
traj_plot = plot(x(1), y(1), 'Color', [1 0.5 0], 'LineWidth', 1.2);  % naranja
h_body   = patch(NaN, NaN, 'w', 'EdgeColor', 'r', 'FaceAlpha', 0, 'LineWidth', 2);  % borde rojo
h_sensor = patch(NaN, NaN, 'w', 'EdgeColor', 'black', 'FaceAlpha', 0, 'LineWidth', 2);  % caja verde frontal
h_wL     = patch(NaN, NaN, 'k');  % rueda izquierda
h_wR     = patch(NaN, NaN, 'k');  % rueda derecha
legend([traj_plot, h2], {'Trayectoria generada', 'Trayectoria de referencia'});

% --- animación ---
for k = 1:20:N
    xk = x(k); yk = y(k); thk = th(k);
    R = [cos(thk) -sin(thk); sin(thk) cos(thk)];

    % cuerpo
    B = R * body_xy + [xk; yk];
    set(h_body, 'XData', B(1,:), 'YData', B(2,:));

    % sensor frontal
    S = R * sensor_xy + [xk; yk];
    set(h_sensor, 'XData', S(1,:), 'YData', S(2,:));

    % ruedas horizontales
    WL = R * (wheel_rect + L_pos) + [xk; yk];
    WR = R * (wheel_rect + R_pos) + [xk; yk];
    set(h_wL, 'XData', WL(1,:), 'YData', WL(2,:));
    set(h_wR, 'XData', WR(1,:), 'YData', WR(2,:));

    % trayectoria
    set(traj_plot, 'XData', x(1:k), 'YData', y(1:k));
    drawnow;
    pause(dt);
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
