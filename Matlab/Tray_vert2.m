%trayectoria 

%Punto inicial
x_in = 0.30;
y_in = 0.20;
z_in = 0.25;

%Punto final
x_fin = 0.25;
y_fin = -0.20;
z_fin = 0.15;

%Definición de los parámetros de la trayectoria
t_total = 10;%s
t_in = 0.1; %S
t_sim = 0:t_in:t_total;

%Puntos de la trayectoria
xp = x_in+((10/t_total^3)*t_sim.^3-(15/t_total^4)*t_sim.^4+(6/t_total^5)*t_sim.^5)*(x_fin-x_in);
yp = y_in+((10/t_total^3)*t_sim.^3-(15/t_total^4)*t_sim.^4+(6/t_total^5)*t_sim.^5)*(y_fin-y_in);
zp = z_in+((10/t_total^3)*t_sim.^3-(15/t_total^4)*t_sim.^4+(6/t_total^5)*t_sim.^5)*(z_fin-z_in);

plot3(xp,yp,zp)
grid on
title('Trayectoria')
xlabel('x')
ylabel('y')
zlabel('z')


