
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





%Solución del robot


waprox = [pi/3 pi/4 pi/3];

for i=1:length(t_sim)
    
  q_cal(i,:) = fsolve(@(q) postura3R(q,xp(i),yp(i),zp(i)),waprox); 
    
   theta1_cal(i) =  q_cal(i,1); 
   theta2_cal(i) =  q_cal(i,2); 
   theta3_cal(i) =  q_cal(i,3); 
   
   waprox = [theta1_cal(i) theta2_cal(i) theta3_cal(i)];
    
end


%Parámetros del tiempo

t_total = 10; %s
t_muestreo = 0.1; %s
t_sim = 0:t_muestreo:t_total; %arreglo del tiempo

t = transpose(t_sim);

theta1_t = transpose(theta1_cal);
theta2_t = transpose(theta2_cal);
theta3_t = transpose(theta3_cal);

signal_theta1 = [t theta1_t];
signal_theta2 = [t theta2_t];
signal_theta3 = [t theta3_t];

%Fin del programa