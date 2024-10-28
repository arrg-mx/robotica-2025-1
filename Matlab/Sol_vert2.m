%Solución del robot


waprox = [pi/3 pi/4 pi/3];

for i=1:length(t_sim)
    
  q_cal(i,:) = fsolve(@(q) postura3R(q,xp(i),yp(i),zp(i)),waprox); 
    
   theta1_cal(i) =  q_cal(i,1); 
   theta2_cal(i) =  q_cal(i,2); 
   theta3_cal(i) =  q_cal(i,3); 
   
   waprox = [theta1_cal(i) theta2_cal(i) theta2_cal(i)];
    
end


%Parámetros del tiempo

t_total = 10; %s
t_muestreo = 0.1; %s
t_sim = 0:t_muestreo:t_total; %arreglo del tiempo

t = transpose(t_sim);

theta1_t = transpose(-theta1_cal);
theta2_t = transpose(-theta2_cal);
theta3_t = transpose(theta3_cal);

signal_theta1 = [t theta1_t];
signal_theta2 = [t theta2_t];
signal_theta3 = [t theta3_t];

%Fin del programa

