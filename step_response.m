clear

% Tempo de amostragem
Ts = 0.0033;

% Tempo de simulacao
t = 0:Ts:30;

% Trajetoria
ganho = 1;
freq = 2*pi/30;
% traj = [ganho*sin(freq*t); ganho*sin(2*freq*t)];
traj = [ganho*ones(length(t)); ganho*ones(length(t))];

% Definicao do sistema
G = [0 1 0 0;
     0 0 0 0;
     0 0 0 1;
     0 0 0 0];

H = [0 0;
     1 0;
     0 0;
     0 1];
 
C = [1 1 0 0];
 
discSys = c2d(ss(G,H,C,0), Ts);

real = 10;
im = 0;

% Calculo da matriz de ganho K
cont_poles = [-real + im*1i, -real - im*1i, -2*real + im*1i, -2*real - im*1i];
disc_poles = [pol2cart(im*Ts, abs(exp(-real*Ts)));
              pol2cart(-im*Ts, abs(exp(-real*Ts)));
              pol2cart(im*Ts, abs(exp(-2*real*Ts)));
              pol2cart(-im*Ts, abs(exp(-2*real*Ts)))]

Kc = place(G, H, cont_poles);
Kd = place(discSys.A, discSys.B, disc_poles);

% Condicao inicial
i_pose = [0, 0, 0];
i_vel = [0.1, 0.1];

% Set das variaveis do primeiro loop
x_pos = i_pose(1);
y_pos = i_pose(2);
theta = i_pose(3);
x_vel = i_vel(1);
y_vel = i_vel(2);

x_vel_ref_prev = x_vel;
y_vel_ref_prev = y_vel;

robot_x_pos = [];
robot_y_pos = [];

% Loop de simulacao
for k = 2:length(t)
    robot_x_pos = [robot_x_pos; x_pos];
    robot_y_pos = [robot_y_pos; y_pos];
    
    % Calculo das referencias por loop
    x_pos_ref = traj(1,k);
    y_pos_ref = traj(2,k);
    x_vel_ref  = (x_pos_ref - traj(1,k-1))/Ts;
    y_vel_ref  = (y_pos_ref - traj(2,k-1))/Ts;
    
    % Calculo dos estados (erros)
    z_e = [x_pos_ref - x_pos;
           x_vel_ref - x_vel;
           y_pos_ref - y_pos;
           y_vel_ref - y_vel];
      
    % Calculo das aceleraçoes de referencia
    x_a_ref = (x_vel_ref - x_vel_ref_prev)/Ts;
    y_a_ref = (y_vel_ref - y_vel_ref_prev)/Ts;
    
    
    % Lei de controle
    mid_u = Kc*z_e + [x_a_ref; y_a_ref];
    
    % Calculo da matriz F
    v = sqrt((x_vel)^2 + (y_vel)^2);
    F = [cos(theta) -v*sin(theta);
         sin(theta) v*cos(theta)]; 
    final_u = F^(-1)*mid_u;
    
    % Entrada no robo
    v = final_u(1)*Ts + v; % integra dv
    w = final_u(2);
    robot_input = [v; w];
    robot_cinematic = [cos(theta) 0;
                       sin(theta) 0;
                                0 1];
    dout_robot = robot_cinematic * robot_input;
    
    % "Leitura" dos estados do robô (integrando a saída)
    noise = 0.00; % Set to experiment with noise (e.g. 0.001)
    out_robot = [x_pos + Ts*dout_robot(1);
                 y_pos + Ts*dout_robot(2);
                 theta + Ts*dout_robot(3)];
    
    x_vel = dout_robot(1);
    y_vel = dout_robot(2);
     
    x_pos = out_robot(1);
    y_pos = out_robot(2);
    theta = out_robot(3);
    
    x_vel_ref_prev = x_vel_ref;
    y_vel_ref_prev = y_vel_ref;
end

plot(t(1:length(t)),traj(2,:));
hold on
plot(t(1:length(t)-1), robot_y_pos);
hold off
% legend("Trajetoria de referencia", "Trajetoria do robo");


