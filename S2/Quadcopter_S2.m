classdef Quadcopter_S2 < handle
    
    % Define robot fixed parameters
    properties (Access=private, Constant)
        
        %width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0];

        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];        

    end
    
    % Define robot variable parameters (incl. state, output, input, etc)
    properties    
        % plotting
        ax (1,1) matlab.graphics.axis.Axes;
        
        % Robot parameters needed for the simulation
        pos (3,1) double;       % 3D position: x-y-z 
        rot (3,1) double;       % 3D orientation: yaw-pitch-roll 
        pos_dot (3,1) double;   % 3D velocity
        rot_dot (3,1) double;   % Derivative of rot    
        omega (3,1) double;     % Angular velocity vector
        R                       % Rotation matrix
        time                    % Simulation time
        dt                      % The time interval
        m                       %mass
        g                       %gravity
        kd                      %frictional coefficient
        L                       % Length from each propeler to the centre
        I                       % Quadcopter rotational inertia matrix
        gamma                   % The square of the angular velocities, omega_dot 
        gamma_equ               % The inputs when the quadcopter is in equilibrium
        delta_u                 % Input variance
        x                       % State variables
        x_equ                   % equilibrium state
        delta_x                 % State variable variance
        % Propeller constants
        k
        b
        % Discret LTI parameters
        Ad
        Bd
        % Record the position, orientation and time for plots
        POS
        ROT
    end    

    methods
        % Class constructor
        function obj = Quadcopter_S2(ax, dt)
            % Initialize parameters
            obj.ax = ax;
            obj.dt = dt;
            obj.time = 0;
            obj.m = 0.3;
            obj.g = 9.8;
            obj.kd = 0.2;
            obj.k = 1;
            obj.L = 0.25;
            obj.b = 0.2;
            obj.I = [1, 0, 0;
                     0, 1, 0;
                     0, 0, 0.4];

            % Initialize variables
            obj.pos = [2.5; 2.5; 10];
            obj.pos_dot = zeros(3,1);
            obj.rot =  zeros(3,1);
            obj.rot_dot =  zeros(3,1);
            obj.R = rotation(obj);
            obj.omega = rotdot2omega(obj);

            obj.gamma = a2gamma(zeros(3, 1), obj);
            obj.gamma_equ = a2gamma(zeros(3, 1), obj);
            obj.delta_u = zeros(4,1);
            
            % Initialize state space parameters
            obj.x = [obj.pos; obj.pos_dot; obj.rot; obj.omega];
            obj.x_equ = obj.x;
            obj.delta_x = zeros(12,1);
            [obj.Ad, obj.Bd] = AB_d(obj);
            obj.POS = [];
            obj.ROT = [];
        end        
        
        %% Plot Function
        function plot(obj)
            %create middle sphere
            [X Y Z] = sphere(8);
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.ax,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            rot_mat           = eul2rotm(obj.rot.');
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = rot_mat*rotorPosBody;
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.ax,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
        end
        % you can modify this to model quadcopter physics
        %% Gamma
        % Compute the inputs gamma of 4 rotors required for hovering,
        % when a_z equals 0
        function gamma = a2gamma(a, obj)
            obj.R = rotation(obj);
            G=[0; 0; -obj.g];
            Fd = -obj.kd * obj.pos_dot;
            T = Fd + obj.m * (a-G) / (cos(obj.rot(2))*cos(obj.rot(1)));
            
            gamma = zeros(4, 1);
            gamma(:) = T(3)/4;
        end
        %% Acceleration
        % Calculate the accelertation of the quadcopter
        function a = acceleration(obj)
            obj.R = rotation(obj);
            G=[0; 0; -obj.g];
            Fd = -obj.kd * obj.pos_dot;
            T = obj.R * thrust(obj.gamma, obj);
            a = G + 1 / obj.m * T + Fd;
        end
        %% Rotation
        % Derive the Rotation Matrix
        function R = rotation(obj)
            phi = obj.rot(1);
            theta = obj.rot(2);
            psi = obj.rot(3);
            
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
        end
        %% Thrust
        % Calculate the trust based on the inputs
        function T = thrust(gamma, obj)
            T= [0; 0; obj.k * sum(gamma)];
        end
        %% Omega
        function omega = rotdot2omega(obj)
            % Calculate the angular velocity vector based on 
            % rot and rot_dot
            phi = obj.rot(1);
            theta = obj.rot(2);
            omega = [1, 0, -sin(theta);
                0, cos(phi), cos(theta) * sin(phi);
                0, -sin(phi), cos(theta) * cos(phi)] * obj.rot_dot;
        end
        %% Angular Velocity
        % Calculate the angular velocity
        function omega_dot = angular_acceleration(obj)
            tau = torques(obj.gamma, obj);
            omega_dot = (obj.I) \ (tau - cross(obj.omega, obj.I * obj.omega));
        end

        %% Rot_dot
        % Calculate the first derivative of rot
        function rot_dot = omega2thetadot(obj)
            phi = obj.rot(1);
            theta = obj.rot(2);
            rot_dot = [1, 0, -sin(theta);
                0, cos(phi), cos(theta) * sin(phi);
                0, -sin(phi), cos(theta) * cos(phi)] \ obj.omega;
        end
    
        %% Torques
        % Calculate the torques from inputs
        function tau = torques(gamma, obj)
            tau = [
                obj.L * obj.k * (gamma(1) - gamma(3))
                obj.L * obj.k * (gamma(2) - gamma(4))
                obj.b * (gamma(1) - gamma(2) + gamma(3) - gamma(4))];
        end
        %% Update Function
        function update(obj, t)
            t = t + obj.dt;

            % update inputs gamma in question 2 
            if t < 4 % Rotation at fixed altitude
                obj.gamma(1) = obj.gamma_equ(1) + 0.2;
                obj.gamma(2) = obj.gamma_equ(2) - 0.2;
                obj.gamma(3) = obj.gamma_equ(3) + 0.2;
                obj.gamma(4) = obj.gamma_equ(4) - 0.2;
            elseif t >= 4 % Fall in one direction
                obj.gamma(1) = obj.gamma_equ(1) + 0.2;
                obj.gamma(2) = obj.gamma_equ(2) - 0.2;
                obj.gamma(3) = 0;
                obj.gamma(4) = 0;
            end

            % Update states
            % x[k + 1] = x0 + Ad δ(x[k]-x0) + Bd (u[k]-u0)          
            obj.delta_x = obj.x - obj.x_equ;
            obj.delta_u = obj.gamma - obj.gamma_equ;
            obj.x = obj.x_equ + obj.Ad * obj.delta_x + obj.Bd * obj.delta_u;
            
            % update variables
            obj.pos = obj.x (1:3);
            obj.pos_dot = obj.x (4:6);
            obj.rot = obj.x (7:9);
            obj.omega = obj.x (10:12);
            obj.R = rotation(obj);

            % Record previous position, orientation and time
            obj.POS = [obj.POS,obj.pos];
            obj.ROT = [obj.ROT, obj.rot];

        end
    end
end