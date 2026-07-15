function R=Rz(theta)
% MATLAB Aplicado a Robotica y Mecatronica
% Editorial Alfaomega, Fernando Reyes Cortes.
% Capitulo 3 Cinematica - Archivo Rz.m
% Funcion: Matriz de rotacion alrededor del eje z
%   R = Rz(theta)
%   theta : angulo de rotacion [rad]

dato=whos('theta');
if strcmp(dato.class,'sym')          % variables simbolicas
    R = simplify([cos(theta), -sin(theta), 0;
                  sin(theta),  cos(theta), 0;
                  0,           0,          1]);
else                                 % calculos numericos
    digits(3);
    R = simplify([double(cos(theta)), double(-sin(theta)), 0;
                  double(sin(theta)), double( cos(theta)), 0;
                  0,                  0,                   1]);
end
end
