%%%Programa de solución analítica de un robot 3R en el espacio
 
%Definición de la trayectoria%%%%%%%%
 
%Punto inicial%%%%%%%%%%%%
x_in = 0.25;
y_in = 0.20;
z_in = 0.15;
 
%Punto final%%%%%%%%%%%%%%
x_fin = 0.25;
y_fin = -0.20;
z_fin = 0.15;
 
%Definición de los parámetros de la trayectoria%%%%%%%%%%%%
t_total = 10;
t_in = 0.1;
t_sim = 0:t_in:t_total;
 
%Puntos de la trayectoria%%%%%%%%%%%%%%
xp = x_in+((10/t_total^3)*t_sim.^3-(15/t_total^4)*t_sim.^4+(6/t_total^5)*t_sim.^5)*(x_fin-x_in);
yp = y_in+((10/t_total^3)*t_sim.^3-(15/t_total^4)*t_sim.^4+(6/t_total^5)*t_sim.^5)*(y_fin-y_in);
zp = z_in+((10/t_total^3)*t_sim.^3-(15/t_total^4)*t_sim.^4+(6/t_total^5)*t_sim.^5)*(z_fin-z_in);

%%%%%%%Cálculo de las velocidades

%Velocidades de la trayectoria%%%%%%%%%%%%%%
xpv = ((30/t_total^3)*t_sim.^2-(60/t_total^4)*t_sim.^3+(30/t_total^5)*t_sim.^4)*(x_fin-x_in);
ypv = ((30/t_total^3)*t_sim.^2-(60/t_total^4)*t_sim.^3+(30/t_total^5)*t_sim.^4)*(y_fin-y_in);
zpv = ((30/t_total^3)*t_sim.^2-(60/t_total^4)*t_sim.^3+(30/t_total^5)*t_sim.^4)*(z_fin-z_in);


 
plot3(xp,yp,zp)
grid on
title('Trayectoria')
xlabel('x')
ylabel('y')
zlabel('z')
 
%%%%%%%parametros del robot
b1 = 0.085;%0z1
L1 = 0.25; %1z2
L2 = 0.2875;%2z3
 
%%Cálculo de la solución analítica
 
 
z1 = [0;0;1];     %Vector de dirección z1
z1_n = 1;       %Norma del vector z1
p0_1 = [0;0;b1];    %Posición del sistema 1 con respecto al sistema 0
 
for i=1:length(t_sim)
    
    p0_P = [xp(i);yp(i);zp(i)]; %Vector de posición del sistema P con respecto a 0
    p1_P =  p0_P-p0_1;  
%Vector de posición del sistema P con respecto a 1
    
    nor_p1_P = norm(p1_P);
%Cálculo de la norma del vector p1_P
    produc_p = dot(z1,p1_P);
    
    ang_phi = acos(dot(z1,p1_P)/(norm(p1_P)*z1_n));
    ang_alfa = acos((L1^2+nor_p1_P^2-L2^2)/(2*L1*nor_p1_P));
    
    theta1_analitic(i) = atan2(yp(i),xp(i));
    theta2_analitic(i) = ang_phi-ang_alfa;
    theta3_analitic(i) = pi-asin(nor_p1_P*sin(ang_alfa)/L2);
end
 
%%Grafica de la postura de las juntas
 
figure
plot(t_sim,theta1_analitic,t_sim,theta2_analitic,t_sim,theta3_analitic)
grid on
title('Posiciones angulares numerico')
xlabel('t')
ylabel('rad/s')
legend({'theta1','theta2','theta3'},'Location','southwest')
 
%%Generación de las señales de salida
 
t = transpose(t_sim);
 
theta1_geo = transpose(theta1_analitic);
theta2_geo = transpose(theta2_analitic);
theta3_geo = transpose(theta3_analitic);
 
signal_theta1geo = [t theta1_geo];
signal_theta2geo = [t theta2_geo];
signal_theta3geo = [t theta3_geo];
 
%Cálculo de la inversa cinemática 
 
for i=1:length(t_sim)
    
   theta1_sol = theta1_analitic(i);
   theta2_sol = theta2_analitic(i);
   theta3_sol = theta3_analitic(i);
 
  
J11 = -sin(theta1_sol)*(L2*sin(theta2_sol+theta3_sol)+L1*sin(theta2_sol));
J12 = cos(theta1_sol)*(L2*cos(theta2_sol+theta3_sol)+L1*cos(theta2_sol));
J13 = L2*cos(theta2_sol+theta3_sol)*cos(theta1_sol);
 
J21 = cos(theta1_sol)*(L2*sin(theta2_sol+theta3_sol)+L1*sin(theta2_sol));
J22 = sin(theta1_sol)*(L2*cos(theta2_sol+theta3_sol)+L1*cos(theta2_sol));
J23 = L2*cos(theta2_sol+theta3_sol).*sin(theta1_sol); 
 
J31 = 0;
J32 = -L2*sin(theta2_sol+theta3_sol)-L1*sin(theta2_sol);
J33 = -L2*sin(theta2_sol+theta3_sol);
 
J = [J11,J12,J13;J21,J22,J23;J31,J32,J33];
 
pvx = xpv(i);
pvy = ypv(i);
pvz = zpv(i);
 
pv = [pvx;pvy;pvz];
 
qv = inv(J)*pv;
 
theta1v_n(i)=qv(1);
theta2v_n(i)=qv(2);
theta3v_n(i)=qv(3);
 
w(i) = det(J)/(L1*L2);
 
end
 
figure
plot(t_sim,w)
grid on
title('Manipulabilidad')
xlabel('t')
ylabel('w')
 
figure
plot(t_sim,theta1v_n,t_sim,theta2v_n,t_sim,theta3v_n)
grid on
title('Velocidades angulares numerico')
xlabel('t')
ylabel('rad/s')
legend({'theta1','theta2','theta3'},'Location','southwest')

