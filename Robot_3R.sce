
//Longitudes de los links y la pala
L1 = 0.62
L2 = 0.57
g1 = 0.10
g2 = 0.20
g3 = 0.30

//Posición inicial de la pala
x  =  0.90
y  = -0.20
gamma = 0

//Velocidades para la dinámica del robot
Vx =  0
Vy = -0.10

//Número de iteraciones que se quieran realizar en el bucle
dt = 0.1

function[theta1,theta2,theta3] = EndEffectorPosition2JointPose(x, y, gamma)

//Offset desde la pala hasta el tercer joint
Gamma_matrix = [cos(gamma),-sin(gamma);
                sin(gamma), cos(gamma)]
x_y_prima = [x;y]-Gamma_matrix*[0;-g2]-Gamma_matrix*[g3;0]

//Theta 2
d = 2 * L1 * L2
f = (x_y_prima(1)-g1*cos(gamma))^2+(x_y_prima(2)-g1*sin(gamma))^2-L1^2-L2^2

theta2 = acos(f/d)

if theta2 > %pi then
   theta2 = 2*%pi - theta2
end

//Theta 1
A = L1 + L2 * cos(theta2)
B = L2 * sin(theta2)
E = x_y_prima(1) - g1 * cos(gamma)
F = x_y_prima(2) - g1 * sin(gamma)

theta1_matrix = inv([A, -B; B, A])*[E;F]

theta1 = atan(theta1_matrix(2), theta1_matrix(1))

if theta1 > %pi then
   theta1 = 2*%pi - theta1
end

//Theta 3
theta3 = gamma - theta1 - theta2

endfunction

function [Pos1, Pos2, Pos3] = POSE(theta1, theta2, theta3)

//Posición de cada joint
x1 = 0
y1 = 0
Pos1 = [x1,y1]

x2 = L1 * cos(theta1)
y2 = L1 * sin(theta1)
Pos2 = [x2,y2]

x3 = L2 * cos(theta1 + theta2) + x2
y3 = L2 * sin(theta1 + theta2) + y2
Pos3 = [x3,y3]

endfunction

function [w1, w2, w3] = Jacobian(Pos1, Pos2, Pos3)

//Dinamica del robot
J = [Pos1(2),  Pos2(2),  Pos3(2);
    -Pos1(1), -Pos2(1), -Pos3(1);
     1,        1,        1]

Twist =[Vx;
        Vy;
        gamma]

Gamma = inv(J)*Twist

w1 = Gamma(1)
w2 = Gamma(2)
w3 = Gamma(3)

endfunction


function [] = Robot_Plot(Pos1, Pos2, Pos3)

//Posición de cada "link" de la pala
x_g1 = g1 * cos(theta1 + theta2 + theta3) + Pos3(1)
y_g1 = g1 * sin(theta1 + theta2 + theta3) + Pos3(2)
Pos_g1 = [x_g1,y_g1]

x_g2 = x_g1
y_g2 = y_g1 - g2
Pos_g2 = [x_g2,y_g2]

x_g3 = x_g2 + g3
y_g3 = y_g2
Pos_g3 = [x_g3,y_g3]

subplot(211)

//Título y nombres de los ejes
title("Proyecto Cinemática - Brazo Robótico 3R")
xlabel('X [m]'); ylabel('Y [m]')
xgrid(7)

//Incluir ejes en el gráfico y limitarlos [Min_x,Min_y,Max_x,Max_y]
a=get("current_axes")
a.data_bounds = [-0.2,-1;1,0]

//Borrar anteriores ejes para crear el movimiento del robot
a = gca();
delete(a.children);

//Dibujar cada uno de los links del robot y la pala en el gráfico
xsegs([Pos1(1),Pos2(1)],[Pos1(2),Pos2(2)],1:1)
xsegs([Pos2(1),Pos3(1)],[Pos2(2),Pos3(2)],1:1)
xsegs([Pos3(1),Pos_g1(1)],[Pos3(2),Pos_g1(2)],1:1)
xsegs([Pos_g1(1),Pos_g2(1)],[Pos_g1(2),Pos_g2(2)],1:1)
xsegs([Pos_g2(1),Pos_g3(1)],[Pos_g2(2),Pos_g3(2)],1:1)
plot(Pos_g3(1),Pos_g3(2),'o')

endfunction

function [] = Omega_Plot(w1, w2, w3, tiempo)

subplot(212)

//Título y nombres de los ejes
title("Gráfico Omegas - Tiempo")
xlabel('Tiempo [s]'); ylabel('Omegas [rad/s]')
xgrid(7)
//Incluir ejes en el gráfico y limitarlos [Min_x,Min_y,Max_x,Max_y]
a = gca()
a.data_bounds = [0,-0.3;5,0.5]

plot2d(tiempo, w1, 0)
e = gce();
point = e.children();    //point sirve para añadir propiedades a ese plot
point.mark_mode="on";    //se muestran puntos a lo largo de la gráfica
point.mark_size =1;      //tamaño de los puntos
point.mark_foreground=5; //color de los puntos

plot2d(tiempo, w2, 0);
e = gce();
point = e.children();
point.mark_mode="on";
point.mark_size =1;
point.mark_foreground=6;

plot2d(tiempo, w3, 0);
e = gce();
point = e.children();
point.mark_mode="on";
point.mark_size =1;
point.mark_foreground=4;

hl=legend(['w1(t)';'w2(t)';'w3(t)'])

endfunction

//------Iteración para movimiento del robot-------

//Crea una figura donde se dibujarán los gráficos
figure()

//Limpia la figura. Por si no se cerró y hay gráficos dibujados
clf()

//Bucle de 0 a 5 segundos divido en "n" iteraciones
for tiempo=0:(dt):5

//Llamamos las funciones que hemos programado
//El primer parentesis son las variables que obtenemos de computar la función
//Después del igual hay el nombre de la función y
//entre parentesis hay las variables que solicita la función para computar

[theta1,theta2,theta3] = EndEffectorPosition2JointPose(x, y, gamma)

[Pos1, Pos2, Pos3] = POSE(theta1, theta2, theta3)

Robot_Plot(Pos1, Pos2, Pos3)

[w1, w2, w3] = Jacobian(Pos1, Pos2, Pos3)

Omega_Plot(w1, w2, w3, tiempo)

//En cada iteración desplazamos unos milímetros el robot hacia abajo
y= y-0.01

end
