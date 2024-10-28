%Solución del robot


waprox = [pi/3 pi/4 pi/3];

for i=1:length(t_sim)
    
  q_cal(i,:) = fsolve(@(q) postura3R(q,x_arr_1_4(i),y_arr_1_4(i),z_arr_1_4(i)),waprox); 
    
   theta1_cal(i) =  q_cal(i,1); 
   theta2_cal(i) =  q_cal(i,2); 
   theta3_cal(i) =  q_cal(i,3); 
   
   waprox = [theta1_cal(i) theta2_cal(i) theta2_cal(i)];
    
end


%%

%Arreglo en el tiempo
%%%%%%%%%%%%%%%%
t_total_1_a = 4;%s
t_in = 0.1; %S
tlinea_1_tray = 0:t_in:t_total_1_a;

%%%%%%%%%%%%%%%%%
t_total_2_a = 4+t_total_1_a;%s
tlinea_2_tray = t_total_1_a:t_in:t_total_2_a;

%%%%%%%%%%%%%%%%%
t_total_3_a = 4+t_total_2_a;%s
tlinea_3_tray = t_total_2_a:t_in:t_total_3_a;

%%%%%%%%%%%%%%%%%
t_total_4_a = 4+t_total_3_a;%s
tlinea_4_tray = t_total_3_a:t_in:t_total_4_a;

%%%%%%%%%%%%%%%%%%
t_trayectoria = [tlinea_1_tray tlinea_2_tray tlinea_3_tray tlinea_4_tray];


%%

theta1_t = transpose(-theta1_cal);
theta2_t = transpose(-theta2_cal);
theta3_t = transpose(theta3_cal);

t_tray = transpose(t_trayectoria);


signal_theta1p = [t_tray theta1_t];
signal_theta2p = [t_tray theta2_t];
signal_theta3p = [t_tray theta3_t];

%Fin del programa

