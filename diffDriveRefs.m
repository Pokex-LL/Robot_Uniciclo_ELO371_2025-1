function [wL_ref, wR_ref, v_d, omega_d, theta_d] = ...
        diffDriveRefs(xd, yd, dt, r, L)
% xd, yd : puntos posición deseada
% dt     : paso temporal
% r, L   : radio y separación de ruedas
%
% Devuelve: wL_ref, wR_ref (rad/s)

    % 1) Velocidades en x e y deseadas para seguir la trayectoria
    dx  = gradient(xd, dt); 
    dy  = gradient(yd, dt);

    % 2) Orientación y velocidad angular 
    theta_d  = unwrap(atan2(dy, dx));
    omega_d  = gradient(theta_d, dt);

    % 3) Velocidad lineal deseada
    v_d = sqrt(dx.^2 + dy.^2);

    % 4) Velocidades de las ruedas a partir de la cinemática
    wR_ref = ( v_d + 0.5*L*omega_d ) / r;
    wL_ref = ( v_d - 0.5*L*omega_d ) / r;
end
