L = 0.075; %distancia entre as rodas = 7.5cm
x_traj = out.ref_trajectory.x_ref.data; 
y_traj = out.ref_trajectory.y_ref.data;
x_robot = out.robot_pose.x.data;
y_robot = out.robot_pose.y.data;
v = out.v.data;
w = out.w.data;
t = out.tout;
errors = out.errors.data;

wR = (2*v + L*w)/2;
wL = (2*v - L*w)/2;
control_signal = [v,w];



figure(1)
plot(x_traj, y_traj)
hold on
plot(x_robot, y_robot, "o",'MarkerSize', 4)
hold off
legend("Trajetoria referencia","Trajetoria robo")
xlabel("x (m)");
ylabel("y (m)");
title("Trajetoria de Referencia");
grid on
saveas(gcf,'imagens/robo_traj.png')

figure(2)
stairs(t, wR)
hold on
stairs(t, wL)
hold off
title("Sinais de Velocidade das Rodas do Robô")
legend("Roda direita", "Roda esquerda");
ylabel("Velocidade angular (rad/s)")
xlabel("Tempo (s)")
grid on
saveas(gcf,'imagens/rodas.png')

figure(3)
stairs(t, errors)
legend("x_{e}", "x'_{e}", "y_{e}", "y'_{e}")
xlabel("Tempo (s)")
ylabel("Erro de estado")
title("Erro das Variaveis do Espaço de Estados")
grid on
saveas(gcf,'imagens/erros.png')

figure(4)
subplot(2,1,1)
stairs(t, v)
xlabel("Tempo (s)")
ylabel("v (m/s)")
title("Sinal de Controle da Velocidade Linear")
grid on

subplot(2,1,2)
stairs(t, w)
legend("w (rad/s)")
xlabel("Tempo (s)")
ylabel("Sinal de Controle")
title("Sinal de Controle da Velocidade Angular")
grid on
saveas(gcf,'imagens/controle.png')
    
