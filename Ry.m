function R=Ry(theta)
% MATLAB Aplicado a Robotica y Mecatronica
% Editorial Alfaomega, Fernando Reyes Cortes.
% Capitulo 3 Cinematica - Archivo Ry.m
% Funcion: Matriz de rotacion alrededor del eje y
%   R = Ry(theta)

dato=whos('theta');
if strcmp(dato.class,'sym')
    R = simplify([ cos(theta), 0, sin(theta);
                   0,          1, 0;
                  -sin(theta), 0, cos(theta)]);
else
    digits(3);
    R = simplify([double( cos(theta)), 0, double(sin(theta));
                  0,                  1, 0;
                  double(-sin(theta)),0, double(cos(theta))]);
end
end
