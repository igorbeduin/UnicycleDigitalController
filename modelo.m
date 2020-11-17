clear

 Ts = 0.033; % Sampling time
 t = 0:Ts:30; % Simulation time

 % Reference
 ganho = 1;
 freq = 2*pi/30;
 xRef = 1 + ganho*sin(freq*t); yRef = 1 + ganho*sin(2*freq*t);
 dxRef = freq*ganho*cos(freq*t); dyRef = 2*freq*ganho*cos(2*freq*t);
 ddxRef =-freq^2*ganho*sin(freq*t); ddyRef =-4*freq^2*ganho*sin(2*freq*t);
 
 qRef = [xRef; yRef; atan2(dyRef, dxRef)];
 uRef = [ddxRef; ddyRef];

 q = [0; 0; 0]; % Initial robot pose
 z1 = [q(1); dxRef(1)]; % Initial state [x, x’]
 z2 = [q(2); dyRef(1)]; % Initial state [y, y’]
 v = sqrt(z1(2)^2+z2(2)^2); % Initial state of velocity integrator

 % Matrices of linearized system
 A=[0,1;0,0];B=[0;1];C=[1,0];
 % State feedback controller
 mod = 2;
 cmp = 5;
 %desPoles = [-mod - cmp*1i, -mod + cmp*1i]; % Desired poles (of the controller)
 desPoles = [-1, -2]
 K = place(A, B, desPoles); % Control gains obtained by pole placement
 
 posX = [];
 posY = [];
 e_posX = [];
 e_posY = [];
 
 velX = [];
 velY = [];
 e_velX = [];
 e_velY = [];
 for k = 1:length(t)
    % Reference states
    zRef1 = [xRef(k); dxRef(k)]; zRef2 = [yRef(k); dyRef(k)];
    % Error and control
    ez1 = zRef1 - z1;
    ez2 = zRef2 - z2;
    uu = [K*ez1; K*ez2]+ [ddxRef(k); ddyRef(k)];
    % Compute inputs to the robot
    F = [cos(q(3)), -v*sin(q(3)); ...
         sin(q(3)), v*cos(q(3))];
    vv = F^(-1)*uu; % Translational acceleration and angular velocity 
    v = v + Ts*vv(1) % Integrate translational acceleration
    u = [v; vv(2)]; % Robot input
    
    % Robot motion simulation
    dq = [u(1)*cos(q(3)); u(1)*sin(q(3)); u(2)];
    noise = 0.005; % Set to experiment with noise (e.g. 0.001)
    q = q + Ts*dq + randn(3,1)*noise; % Euler integration
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]
    
    % Take known (measured) orientation and velocity to compute states 
    z1 = [q(1); u(1)*cos(q(3))];
    z2 = [q(2); u(1)*sin(q(3))];
    
    posX = [posX;z1(1)];
    posY = [posY;z2(1)];
    e_posX = [e_posX; ez1(1)];
    e_posY = [e_posY; ez2(1)];
    
    velX = [posX;z1(2)];
    velY = [posY;z2(2)];
    e_velX = [e_velX; ez1(2)];
    e_velY = [e_velY; ez2(2)];
 end

plot(xRef, yRef)
hold on
plot(posX, posY)
hold off
legend("Trajetoria de referencia", "Trajetoria do robo")




