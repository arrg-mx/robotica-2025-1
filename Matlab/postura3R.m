%Definición de la función
function Frvert2 = postura3R(q,xp,yp,zp)

%variables 
theta1 = q(1);
theta2 = q(2);
theta3 = q(3);

%parametros del robot
b1 = 0.085;
L1 = 0.25;
L2 = 0.2875;

%Defunciones
Frvert2(1) = cos(theta1)*(L2*sin(theta2+theta3)+L1*sin(theta2))-xp;
Frvert2(2) = sin(theta1)*(L2*sin(theta2+theta3)+L1*sin(theta2))-yp;
Frvert2(3) = b1+L2*cos(theta2+theta3)+L1*cos(theta2)-zp;
end

