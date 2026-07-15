function R=Rx(theta)
% MATLAB Aplicado a Robotica y Mecatronica
% Editorial Alfaomega, Fernando Reyes Cortes.
% Capitulo 3 Cinematica - Archivo Rx.m
% Funcion: Matriz de rotacion alrededor del eje x
%   R = Rx(theta)

dato=whos('theta');
if strcmp(dato.class,'sym')
    R = simplify([1, 0,           0;
                  0, cos(theta), -sin(theta);
                  0, sin(theta),  cos(theta)]);
else
    digits(3);
    R = simplify([1, 0,                    0;
                  0, double(cos(theta)), double(-sin(theta));
                  0, double(sin(theta)), double( cos(theta))]);
end
end
