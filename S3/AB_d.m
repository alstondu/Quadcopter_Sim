% Compute discret Ad and Bd 
function [Ad, Bd] = AB_d(obj)
    % Symbolic variables used to compute jacobian
    syms x [12,1]
    syms x_dot [12,1]
    syms u [4,1]
    
    %% Derive physics
    
    G=[0; 0; -obj.g];

    % Rotation Matrix
    phi = x(7);
    theta = x(8);
    psi = x(9);
    
    Rx = [1 0 0;
        0 cos(phi) -sin(phi); ... 
        0 sin(phi) cos(phi)];
    Ry = [ cos(theta) 0 sin(theta);
        0 1 0; ...
        -sin(theta) 0 cos(theta)];
    Rz = [cos(psi) -sin(psi) 0; ... 
        sin(psi) cos(psi) 0; ... 
        0 0 1];
    R = Rz*Ry*Rx;

    % Drag force 
    Fd = -obj.kd * [x(4); x(5); x(6)] ;
    
    % Torques
    tau = [
        obj.L * obj.k * (u(1) - u(3))
        obj.L * obj.k * (u(2) - u(4))
        obj.b * (u(1) - u(2) + u(3) - u(4))];

    % Thrust
    T = [0; 0; obj.k * sum(u)];

    % Acceleration
    a = G +(1/obj.m) * R *(T) + (1/obj.m) *Fd;

    % Derivative of orientation
    rot_dot = [1, 0, -sin(x(8));
               0, cos(x(7)), cos(x(8)) * sin(x(7));
               0, -sin(x(7)), cos(x(8)) * cos(x(7))] \ x(10:12);
    
    % Angular acceleration
    omega_dot = (obj.I) \ (tau - cross(x(10:12), obj.I * x(10:12)));
    
    %% state space
    x_dot(1:3) = x(4:6);
    x_dot(4:6) = a;
    x_dot(7:9) = rot_dot;
    x_dot(10:12) = omega_dot;

    % Compute continuous A and B (symbolic) 
    Aj = jacobian(x_dot,x);
    Bj = jacobian(x_dot,u);
    
    % Compute continuous A and B (numerical)
    A =  subs(Aj,...
        [x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12],...
        obj.x.');
    A = double (subs(A, u, obj.gamma_equ));
    B = double (subs(Bj,...
        [x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12],...
        obj.x.'));
    C = eye (size(A));
    D = zeros(size(B));

    cont_sys = ss(A,B,C,D);

    % Discretize the system
    disc_sys = c2d(cont_sys,obj.dt,'zoh');
    Ad = disc_sys.A;
    Bd = disc_sys.B;
    
end
