%%%Programa de solución analítica de un robot 3R en el espacio
 
%Definición de la trayectoria%%%%%%%%
 
%Punto inicial%%%%%%%%%%%%
x_in = 0.30;
y_in = 0.15;
z_in = 0.15;
beta_in = 3*pi/4;

%Punto final%%%%%%%%%%%%%%
x_fin = 0.30;
y_fin = -0.15;
z_fin = 0.15;
beta_fin = 3*pi/4;
 
%Definición de los parámetros de la trayectoria%%%%%%%%%%%%
t_total = 10;
t_in = 0.1;
t_sim = 0:t_in:t_total;
 
%Puntos de la trayectoria%%%%%%%%%%%%%%
xp = x_in+((10/t_total^3)*t_sim.^3-(15/t_total^4)*t_sim.^4+(6/t_total^5)*t_sim.^5)*(x_fin-x_in);
yp = y_in+((10/t_total^3)*t_sim.^3-(15/t_total^4)*t_sim.^4+(6/t_total^5)*t_sim.^5)*(y_fin-y_in);
zp = z_in+((10/t_total^3)*t_sim.^3-(15/t_total^4)*t_sim.^4+(6/t_total^5)*t_sim.^5)*(z_fin-z_in);

%%Grafica de la trayectoria

plot3(xp,yp,zp)
grid on
title('Trayectoria')
xlabel('x')
ylabel('y')
zlabel('z')

%%Solucion del robot 

%Parametros del robot
%Longitudes de los eslabones
L_1 = 0.08412; %m
L_2 = 0.08412; %m
L_3 = 0.16332; %m

%Componentes del vector p_m_1
%Coordenadas del origen del sistema 1 con respecto al sistema m.
x_m_1 = 0.072;
y_m_1 = 0.0;
z_m_1 = 0.105;

%Definicion de los vectores
p_m_1 = [x_m_1;y_m_1;z_m_1]; %Vector de posicion del sistema 1 con respecto al sistema m

z1 = [0;0;1];     %Vector de dirección z1
z1_n = 1;       %Norma del vector z1
p0_1 = [x_m_1;y_m_1;z_m_1];    %Posición del sistema 1 con respecto al sistema 0
 
%%%%%%%
beta = 3*pi/4;

for i=1:length(t_sim)

    p_m_P = [xp(i);yp(i);zp(i)]; % vector de posicion del sistema P con respecto al sistema m
    p_1_P = p_m_P-p_m_1;

    theta_1_P = atan2(yp(i),(xp(i)-x_m_1));
    p_P_4(i) = p_m_P-[L_3*cos(theta_1_P)*cos(beta);L_3*sin(theta_1_P)*cos(beta);L_3*sin(beta)];

    p_1_4 = p_1_P+p_P_4(i);

%Calculo de los angulos complementarios

    z_1 = [0;0;1];

    Epsilon(i) = acos((dot(z_1,p_1_4))/(norm(z_1)*norm(p_1_4)));
    alpha(i) = acos((L_1^2+norm(p_1_4)^2-L_2^2)/(2*L_1*norm(p_1_4)));

    beta_r(i) = asin((norm(p_1_4)*sin(alpha(i)))/L_2);

    theta_m_1(i) = atan2(yp(i),(xp(i)-x_m_1));
    theta_1_2(i) =Epsilon(i)-alpha(i);
    theta_2_3(i) = pi-beta_r(i);
    theta_3_4(i) = beta_in;
    theta_4_5(i) = 0;
    
end

