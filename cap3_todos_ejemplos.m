%% ================================================================
%  MATLAB Aplicado a Robotica y Mecatronica - Fernando Reyes Cortes
%  Editorial Alfaomega
%  CAPITULO 3: Preliminares Matematicos
%  Script principal: ejecuta Ejemplos 3.1 a 3.7
%  ================================================================
clc; clear all; close all;

%% ---------------------------------------------------------------
% EJEMPLO 3.1 - Producto Interno (Escalar)
% Codigo Fuente 3.1
% ---------------------------------------------------------------
disp('=== EJEMPLO 3.1: Producto Interno o Escalar ===')
x = [2; 4];            % vector columna x in IR^2
y = [8; -3];           % vector columna y in IR^2

xdoty        = dot(x,y)                         % usando funcion dot
xdoty_m      = x'*y                             % forma matricial
theta        = acos(x'*y / (norm(x,2)*norm(y,2))) % angulo entre x e y
xdoty_geo    = norm(x,2)*norm(y,2)*cos(theta)   % forma geometrica

figure(1);
quiver(0,0,x(1),x(2),0,'b','LineWidth',2,'MaxHeadSize',0.5); hold on;
quiver(0,0,y(1),y(2),0,'r','LineWidth',2,'MaxHeadSize',0.5);
proj = (xdoty/norm(x)^2)*x;
plot([y(1) proj(1)],[y(2) proj(2)],'g--','LineWidth',1.5);
plot(proj(1),proj(2),'go','MarkerSize',8,'MarkerFaceColor','g');
text(x(1)+0.3,x(2)+0.2,'x=[2,4]^T','Color','b','FontSize',12,'FontWeight','bold');
text(y(1)+0.3,y(2)-0.5,'y=[8,-3]^T','Color','r','FontSize',12,'FontWeight','bold');
text(proj(1)+0.2,proj(2)-0.5,'||y||cos(\theta)','Color',[0 0.6 0],'FontSize',11);
grid on; axis equal;
xlim([-1 11]); ylim([-5 6]);
title(sprintf('Ejemplo 3.1 — Producto Interno: x·y=%g  |  \\theta=%.4f rad (%.2f°)',...
      xdoty, theta, rad2deg(theta)),'FontSize',12);
xlabel('x'); ylabel('y');
legend('x','y','Proyeccion','Location','NorthWest');

%% ---------------------------------------------------------------
% EJEMPLO 3.2 - Propiedades de la Matriz de Rotacion Rz(theta)
% Codigo Fuente 3.5
% ---------------------------------------------------------------
disp('=== EJEMPLO 3.2: Propiedades de Rz(theta) ===')
syms a b real

% Resultados simbolicos
disp('Rz(a)*Rz(b) ='); simplify(Rz(a)*Rz(b))
disp('Rz(b)*Rz(a) ='); simplify(Rz(b)*Rz(a))
disp('inv(Rz(a)) =');  simplify(inv(Rz(a)))
disp('Rz(-a) =');      simplify(Rz(-a))
disp('Rz(a)^T =');     simplify(Rz(a)')
disp('Rz(a)^T * Rz(a) ='); simplify(Rz(a)'*Rz(a))
disp('Rz(a) * Rz(a)^T ='); simplify(Rz(a)*Rz(a)')
disp('det(Rz(a)) =');  simplify(det(Rz(a)))

% Calculo numerico theta = 90 grados
theta = 90*pi/180;
disp('Rz(90°) numerico:'); Rz(theta)

%% ---------------------------------------------------------------
% EJEMPLO 3.3 - Rotacion 90° alrededor de z0 de una imagen (flecha)
% Codigo Fuente 3.4
% ---------------------------------------------------------------
disp('=== EJEMPLO 3.3: Rotacion 90° alrededor del eje z0 (flecha) ===')
clc_local = false;

pxo=[0,0,0,0,0,6,7,7,8,8,9,9,10,10,10,10,10,10,10,10,10,10,11,12,13,14,14,...
     14,14,14,14,14,14,14,14,15,15,16,16,17,17,18,19,20,21];
pyo=[0,0,0,0,0,9,9,10,9,11,9,12,1,2,3,4,5,6,7,8,9,12,12,12,12,1,2,3,4,5,...
     6,7,8,9,12,9,12,9,11,9,10,9,0,0,0];
pzo=zeros(1,45);

theta = 90*3.1416/180.0;
R_ztheta = [ cos(theta), -sin(theta), 0;
             sin(theta),  cos(theta), 0;
             0,           0,          1];

% Conversion de coordenadas: p1 = Rz(theta)*p0
Sigma1 = R_ztheta * [pxo; pyo; pzo];
px1 = Sigma1(1,:);
py1 = Sigma1(2,:);
pz1 = Sigma1(3,:);

figure(2);
plot3(pxo,pyo,pzo,'.',px1,py1,pz1,'x');
legend('\Sigma_0 original','\Sigma_1 rotado 90°');
title('Ejemplo 3.3 — Rotacion 90° alrededor del eje z_0','FontSize',12);
xlabel('x'); ylabel('y'); zlabel('z');
grid on; axis equal;

%% ---------------------------------------------------------------
% EJEMPLO 3.4 - Proyeccion de p1=[0.8,0.5,1] en sistema fijo Sigma0
% ---------------------------------------------------------------
disp('=== EJEMPLO 3.4: Proyeccion de p1 en sistema fijo Sigma0 ===')
theta = pi/2;
p1 = [0.8; 0.5; 1.0];

% p0 = Rz(pi/2) * p1
p0 = Rz(theta) * p1
disp('Verificacion inversa p1 = Rz(pi/2)^T * p0:')
p1_rec = Rz(theta)' * p0

figure(3);
quiver3(0,0,0,1.5,0,0,'r','LineWidth',2,'MaxHeadSize',0.3); hold on;
quiver3(0,0,0,0,1.5,0,'g','LineWidth',2,'MaxHeadSize',0.3);
quiver3(0,0,0,0,0,1.5,'b','LineWidth',2,'MaxHeadSize',0.3);
% Ejes rotados Sigma1
R = Rz(theta);
e1=R*[1;0;0]*1.2; e2=R*[0;1;0]*1.2; e3=R*[0;0;1]*1.2;
quiver3(0,0,0,e1(1),e1(2),e1(3),'r--','LineWidth',1.5,'MaxHeadSize',0.3);
quiver3(0,0,0,e2(1),e2(2),e2(3),'g--','LineWidth',1.5,'MaxHeadSize',0.3);
quiver3(0,0,0,e3(1),e3(2),e3(3),'b--','LineWidth',1.5,'MaxHeadSize',0.3);
scatter3(p1(1),p1(2),p1(3),150,'r','filled');
scatter3(p0(1),p0(2),p0(3),150,'m','filled');
plot3([0 p1(1)],[0 p1(2)],[0 p1(3)],'r--','LineWidth',1.5);
plot3([0 p0(1)],[0 p0(2)],[0 p0(3)],'m-','LineWidth',1.5);
text(p1(1)+0.05,p1(2)+0.05,p1(3)+0.05,'p_1=[0.8,0.5,1]^T','FontSize',10,'Color','r');
text(p0(1)+0.05,p0(2)+0.05,p0(3)+0.05,...
    sprintf('p_0=[%.1f,%.1f,%.1f]^T',p0(1),p0(2),p0(3)),'FontSize',10,'Color','m');
text(1.55,0,0,'\Sigma_0: x_0','Color','r','FontSize',10);
text(0,1.55,0,'\Sigma_0: y_0','Color','g','FontSize',10);
text(0,0,1.55,'\Sigma_0: z_0','Color','b','FontSize',10);
title('Ejemplo 3.4 — Proyeccion de p_1 en \Sigma_0','FontSize',12);
xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal;

%% ---------------------------------------------------------------
% EJEMPLO 3.5 - Paralelepipedo 180° alrededor de z0
% ---------------------------------------------------------------
disp('=== EJEMPLO 3.5: Paralelipedo rotado 180° alrededor de z0 ===')

% Vertices del paralelepipedo
[bx,by,bz] = meshgrid([1 4],[1 3],[0 2]);
pts0 = [bx(:)'; by(:)'; bz(:)'];

R = Rz(pi);
pts1 = R * pts0;

figure(4);
scatter3(pts0(1,:),pts0(2,:),pts0(3,:),80,'b','filled'); hold on;
scatter3(pts1(1,:),pts1(2,:),pts1(3,:),80,'r','filled');
% Dibujar caras originales y rotadas
X0=[1 4 4 1 1]; Y0=[1 1 3 3 1];
for z0=[0 2]
    plot3(X0,Y0,z0*ones(1,5),'b-','LineWidth',2);
    plot3(-X0,-Y0,z0*ones(1,5),'r-','LineWidth',2);
end
for i=1:4
    plot3(X0([i i+1]),Y0([i i+1]),[0 2],'b-','LineWidth',1.5);
    plot3(-X0([i i+1]),-Y0([i i+1]),[0 2],'r-','LineWidth',1.5);
end
legend('p_0 (original)','p_1 = R_z(\pi)p_0 (rotado 180°)');
title('Ejemplo 3.5 — Paralelepipedo rotado 180° alrededor de z_0','FontSize',12);
xlabel('x'); ylabel('y'); zlabel('z'); grid on;
quiver3(0,0,0,6,0,0,'k','LineWidth',2,'MaxHeadSize',0.15);
quiver3(0,0,0,0,6,0,'k','LineWidth',2,'MaxHeadSize',0.15);
quiver3(0,0,0,0,0,3,'k','LineWidth',2,'MaxHeadSize',0.15);
text(6.2,0,0,'x_0','FontSize',11); text(0,6.2,0,'y_0','FontSize',11);

%% ---------------------------------------------------------------
% EJEMPLO 3.6 - Composicion de Rotaciones
% ---------------------------------------------------------------
disp('=== EJEMPLO 3.6: Composicion de Rotaciones ===')
syms phi theta real

R_y0_phi = Ry(phi);
R_z1_theta = Rz(theta);

% Orden 1: R2_0 = Ry(phi)*Rz(theta)
R20_orden1 = simplify(R_y0_phi * R_z1_theta)

% Orden 2 (inverso): R2_0 = Rz(theta)*Ry(phi)
R20_orden2 = simplify(R_z1_theta * R_y0_phi)

disp('Las matrices SON distintas => rotaciones NO conmutativas')
disp('simplify(R20_orden1 - R20_orden2) =')
simplify(R20_orden1 - R20_orden2)

% Numericamente: phi=45, theta=30
phi_n = 45*pi/180; theta_n = 30*pi/180;
R20_n1 = Ry(phi_n)*Rz(theta_n);
R20_n2 = Rz(theta_n)*Ry(phi_n);
disp('R20 = Ry(45°)*Rz(30°):'); disp(R20_n1)
disp('R20 = Rz(30°)*Ry(45°):'); disp(R20_n2)
fprintf('max|diff| = %.6f\n', max(max(abs(R20_n1 - R20_n2))));

%% ---------------------------------------------------------------
% EJEMPLO 3.7 - Rotaciones sucesivas: z0, x1, y2
% Codigo Fuente 3.5
% ---------------------------------------------------------------
disp('=== EJEMPLO 3.7: Rotaciones sucesivas alrededor de z0, x1, y2 ===')

clc; clear all; close all;
pzo=zeros(1,45);
pyo=[0,0,0,0,0,9,9,10,9,11,9,12,1,2,3,4,5,6,7,8,9,12,12,12,12,1,2,3,4,5,...
     6,7,8,9,12,9,12,9,11,9,10,9,0,0,0];
pxo=[0,0,0,0,0,6,7,7,8,8,9,9,10,10,10,10,10,10,10,10,10,10,11,12,13,14,14,...
     14,14,14,14,14,14,14,14,15,15,16,16,17,17,18,19,20,21];

theta = 90*3.1416/180.0;
R_ztheta = [ cos(theta),-sin(theta),0; sin(theta), cos(theta),0; 0,0,1];
R_xtheta = [ 1,0,0; 0,cos(theta),-sin(theta); 0,sin(theta),cos(theta)];
R_ytheta = [ cos(theta),0,sin(theta); 0,1,0; -sin(theta),0,cos(theta)];

% Sigma1 = Rz(theta) * p0
Sigma1 = R_ztheta * [pxo; pyo; pzo];
px1=Sigma1(1,:); py1=Sigma1(2,:); pz1=Sigma1(3,:);

% Sigma2 = Rx(theta) * p1
Sigma2 = R_xtheta * [px1; py1; pz1];
px2=Sigma2(1,:); py2=Sigma2(2,:); pz2=Sigma2(3,:);

% Sigma3 = Ry(theta) * p2
Sigma3 = R_ytheta * [px2; py2; pz2];
px3=Sigma3(1,:); py3=Sigma3(2,:); pz3=Sigma3(3,:);

figure(5);
plot3(pxo,pyo,pzo,'.',px1,py1,pz1,'x',px2,py2,pz2,'o',px3,py3,pz3,'^');
legend('\Sigma_0 original',...
       '\Sigma_1: R_z(90°)p_0',...
       '\Sigma_2: R_x(90°)p_1',...
       '\Sigma_3: R_y(90°)p_2',...
       'Location','Best');
title({'Ejemplo 3.7 — Rotaciones Sucesivas alrededor de z_0, x_1, y_2',...
       'p_3 = R_y(90°) R_x(90°) R_z(90°) p_0'},'FontSize',12);
xlabel('x'); ylabel('y'); zlabel('z');
grid on;

disp('=== Todos los ejemplos completados ===')
