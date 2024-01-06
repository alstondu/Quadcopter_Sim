classdef Quadcopter_S3 < handle
    
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
        dt                      % The time interval
        m                       %mass
        g                       %gravity
        kd                      %frictional coefficient
        L                       % Length from each propeler to the centre
        I                       % Quadcopter rotational inertia matrix
        gamma                   % The square of the angular velocities, omega_dot 
        gamma_equ               % The inputs when the quadcopter is in equilibrium
        gamma_pre               % The inputs in the last iteration
        x                       % State variables
        delta_x                 % State variable variance
        % Discret LTI parameters
        Ad
        Bd
        % Propeller constants
        k
        b
        %Full state feedback variables
        eigenvalues
        K
        M_ref                   % reference values for drone to follow
        circle_points           % The number of points on the circle path
        flag                    % bool indicating the reachment of targets
        t                       % Simulation time
        time_stamp              % time stamp for the hovering
        complete                % bool used to stop simulation
        % Record the position, orientation and time for plots
        POS
        V
        Time
    end    

    methods
        % Class constructor
        function obj = Quadcopter_S3(ax, dt)
            % Initialize parameters
            obj.ax = ax;
            obj.dt = dt;
            obj.t = 0;
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
            obj.pos = [0; 0; 0];
            obj.pos_dot = zeros(3,1);
            obj.R = rotation(obj);
            obj.rot =  zeros(3,1);
            obj.rot_dot =  zeros(3,1);
            obj.omega = rotdot2omega(obj);

            obj.gamma = a2gamma(zeros(3, 1), obj);
            obj.gamma_equ = a2gamma(zeros(3, 1), obj);
            obj.gamma_pre = obj.gamma_equ;

            % Initialize state space
            obj.x = [obj.pos; obj.pos_dot; obj.rot; obj.omega];
            obj.delta_x = zeros(12,1);
            [obj.Ad, obj.Bd] = AB_d(obj);

            % Initialize controller variables
            obj.eigenvalues = [0.967, 0.951, 0.92, 0.983, 0.933, 0.984, 0.991, 0.993, 0.974, 0.947, 0.931, 0.925];
            obj.K = place(obj.Ad,obj.Bd,obj.eigenvalues);
            obj.circle_points = 100;
            obj.M_ref = get_M_ref(obj.circle_points);

            % Simulation variables
            obj.flag = zeros(5,1);
            obj.time_stamp = obj.t;
            obj.complete = false;

            % Record simualtion data
            obj.POS = [];
            obj.V = [];
            obj.Time = [];
        end        
        
        % you can modify this to model quadcopter physics
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
        %% Full State Feedback Controller 
        % Obtain inputs from the controller based on reference
        function gamma_fsf(ref, obj)
            e = obj.x-ref;  % error
            % Adjust input relative to the equilibrium inputs
            obj.gamma = obj.gamma_equ - obj.K*e;
            obj.gamma = obj.gamma(:);
            % Confine the inputs within the range [-1.5 1.5]
            obj.gamma = max(min(obj.gamma, 1.5), -1.5);
        end
        %% Reachability check
        % Function to determine reachability
        function check_reachability(Ad, Bd)
            Wr = ctrb(Ad, Bd);  % Rechability matrix
            if rank(Wr) == size(Ad, 1)
                disp('The system is controllable.');
            else
                disp('The system is not controllable.');
            end
        end

        %% Dynamics
        function sim_dynamics(obj)
            obj.omega = rotdot2omega(obj);
            obj.omega = obj.omega + obj.dt * angular_acceleration(obj);
            obj.rot_dot = omega2thetadot(obj);
            obj.rot = obj.rot + obj.dt * obj.rot_dot;
            obj.pos_dot = obj.pos_dot + obj.dt * acceleration(obj);
            obj.pos = obj.pos + obj.dt * obj.pos_dot;
        end
        %% Update Function
        function update(obj)
            obj.t = obj.t + obj.dt;
            tol = 0.05; %tolerance for controller
            if obj.flag(1) == 0 %while the target isn't reached
                ref = obj.M_ref(:,1); %target = point 5,5,5
                disp ('Approaching (0,0,5)')
                gamma_fsf(ref, obj);
                sim_dynamics(obj);
                if sqrt(sum((ref(1:3)-obj.pos).^2)) < tol
                    obj.flag(1) = 1;
                    obj.time_stamp = obj.t;
                end

            elseif obj.flag(1) == 1 ...
                    && obj.flag(2) == 0 %when previous target is reached
                if obj.t < obj.time_stamp+5
                    disp ('Staying at (0,0,5)');
                    ref = obj.M_ref(:,1); %target(0,0,5)
                    gamma_fsf(ref, obj);
                    sim_dynamics(obj);
                else    
                    ref = obj.M_ref(:,2); %target = point (0,2.5,5)
                    disp ('Approaching (0,2.5,5)')
                    gamma_fsf(ref, obj);
                    sim_dynamics(obj);
                    if sqrt(sum((ref(1:3)-obj.pos).^2)) < tol
                        obj.flag(2) = 1;
                    end
                end
            % Circle trajectory
            elseif obj.flag(2) == 1 ...
                && obj.flag(3) == 0 %when previous target is reached
                % Approaching points on the circle iteratively
                for i = 1:obj.circle_points 
                    ref = obj.M_ref(:,2+i);
                    disp(['Approaching the ', num2str(i), 'th point on the circle'])
                    while sqrt(sum((ref(1:3)-obj.pos).^2)) > tol*0.5
                        obj.t = obj.t + obj.dt;
                        gamma_fsf(ref, obj);
                        sim_dynamics(obj);
                        % update state
                        obj.x = [obj.pos;obj.pos_dot;obj.rot;obj.omega];
                        % draw drone to figure (slow down simulation)
                        % plot(obj, obj.t)
                        % drawnow nocallbacks limitrate
                        % Record data
                        obj.POS = [obj.POS,obj.pos];
                        obj.V = [obj.V, obj.pos_dot];
                        obj.Time = [obj.Time, obj.t];
                    end
                    if i == obj.circle_points
                        obj.flag(3) = 1;
                        disp('Drone has returned to (0,2.5,5).');
                        break
                    end
                end
            
            elseif obj.flag(3) == 1 ...
                    && obj.flag(4) == 0 %when previous target is reached
                ref = obj.M_ref(:,obj.circle_points+3); %target (2.5,2.5,2.5)
                disp ('aiming for [2.5,2.5,2.5]')
                gamma_fsf(ref, obj);
                sim_dynamics(obj);
                if sqrt(sum((ref(1:3)-obj.pos).^2)) < tol
                    obj.flag(4) = 1;
                end

            elseif obj.flag(4) == 1 ...
                    && obj.flag(5) == 0 %when previous target is reached
                ref = obj.M_ref(:,obj.circle_points+4); %target(2.5,2.5,0) 
                disp ('Landing')
                gamma_fsf(ref, obj);
                sim_dynamics(obj);
                disp(obj.pos_dot(3))
                if sqrt(sum((ref(1:3)-obj.pos).^2)) < tol
                    obj.flag(5) = 1;
                    obj.gamma = zeros(4,1);
                    disp ('Complete');
                    obj.time_stamp = obj.t;
                end
               
            elseif obj.flag(5) == 1 && obj.t > obj.time_stamp + 2
                sim_dynamics(obj);
                obj.complete = true;   
            end

            % altitude limit
            if obj.pos(3)<0 
                obj.pos(3) = 0;
            end

            disp(obj.pos)

            % update state
            obj.x = [obj.pos;obj.pos_dot;obj.rot;obj.omega];               

            % Record previous position, velocity and time
            obj.POS = [obj.POS,obj.pos];
            obj.V = [obj.V, obj.pos_dot];
            obj.Time = [obj.Time, obj.t];

        end
    end
end