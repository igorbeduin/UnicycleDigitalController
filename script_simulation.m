clear

% Tempo de amostragem
Ts = 0.033;

% Tempo de simulacao
t = 0:Ts:30;

% Trajetoria
ganho = 1;
freq = 2*pi/30;
traj = [1 + ganho*sin(freq*t); 1 + ganho*sin(2*freq*t)];
dtraj = [freq*ganho*cos(freq*t);2*freq*ganho*cos(2*freq*t)];
ddtraj = [-freq^2*ganho*sin(freq*t); -4*freq^2*ganho*sin(2*freq*t)];

% Definicao do sistema
G = [0 1 0 0;
     0 0 0 0;
     0 0 0 1;
     0 0 0 0];

H = [0 0;
     1 0;
     0 0;
     0 1];
 
C = [1;
     1;
     0];
 

% Calculo da matriz de ganho K
poles = [-1, -2, -3, -5];
K = place(G, H, poles);

% Condicao inicial
i_pose = [0, 0, 0];
i_vel = [(traj(1,2) - traj(1,1))/Ts, (traj(2,2) - traj(2,1))/Ts];

% Set das variaveis do primeiro loop
x_pos = i_pose(1);
y_pos = i_pose(2);
theta = i_pose(3);
x_vel = i_vel(1);
y_vel = i_vel(2);

robot_x_pos = [];
robot_y_pos = [];

% Loop de simulacao
for k = 1:length(t)
    robot_x_pos = [robot_x_pos; x_pos];
    robot_y_pos = [robot_y_pos; y_pos];
    
    % Calculo das referencias por loop
    x_pos_ref = traj(1,k);
    y_pos_ref = traj(2,k);
    
    x_vel_ref = dtraj(1,k);
    y_vel_ref = dtraj(2,k);
    
%     x_vel_ref  = (x_pos_ref - x_pos)/Ts;
%     y_vel_ref  = (y_pos_ref - y_pos)/Ts;
    
    % Calculo dos estados (erros)
    z_e = [x_pos_ref - x_pos;
           x_vel_ref - x_vel;
           y_pos_ref - y_pos;
           y_vel_ref - y_vel];
       
    % Calculo das aceleraçoes de referencia
    x_a_ref = (x_vel_ref - x_vel)/Ts;
    y_a_ref = (y_vel_ref - y_vel)/Ts;
    
    % Lei de controle
    mid_u = K*z_e;
    % mid_u = K*z_e + [x_a_ref; y_a_ref];
    % mid_u = K*z_e%[ddtraj(1,k); ddtraj(2,k)];
    
    % Calculo da matriz F
    v = sqrt((x_vel)^2 + (y_vel)^2);
 
    F = [cos(theta) -v*sin(theta);
         sin(theta) v*cos(theta)];
     
    final_u = F^(-1)*mid_u;
    
    x_vel = v*cos(theta);
    y_vel = v*sin(theta);
    
    % Entrada no robo
    v = final_u(1)*Ts + v; % integra dv
    w = final_u(2);
    robot_input = [v; w];
    robot_cinematic = [cos(theta) 0;
                       sin(theta) 0;
                                0 1];
    dout_robot = robot_cinematic * robot_input;
    
    % "Leitura" dos estados do robô (integrando a saída)
    out_robot = [x_pos + Ts*dout_robot(1);
                 y_pos + Ts*dout_robot(2);
                 theta + Ts*dout_robot(3)];
    
    x_vel = dout_robot(1);
    y_vel = dout_robot(2);
    
    x_pos = out_robot(1);
    y_pos = out_robot(2);
    theta = out_robot(3);
end

plot(traj(1,:), traj(2,:));
hold on
plot(robot_x_pos, robot_y_pos);
hold off
legend("Trajetoria de referencia", "Trajetoria do robo");


