% This is the script used to get the simulation data of the none-linear model in S1 
% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 7;
dt          = 0.05;
TIME_SCALE  = 0.1; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


% Initialise plot
figure;
ax1 = axes;
hold(ax1,'on');
view(ax1, 3);
axis('equal')
axis([-10 10 -10 10 -15 15])
axis('manual')
xlabel('x');
ylabel('y');
ylabel('z');
axis vis3d
grid ON
grid MINOR
ax1.Toolbar.Visible = 'off';
ax1.Interactions = [];

% Initialise Simulation
drone1 = Quadcopter_NL(ax1,dt);
drone1.gamma_equ = a2gamma(zeros(3,1), drone1);

% Run Simulation
for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS ______ %
    drone1.update(t);
    drone1.plot(t);
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS ______ %
    
    drawnow nocallbacks limitrate
    pause(TIME_SCALE*dt-toc); 
end

figure(2)
plot3(drone1.POS(1,:),drone1.POS(2,:),drone1.POS(3,:))
title('Quadcopter Path')
xlabel('x')
ylabel('y')
zlabel('z')
grid on;

figure(3);
% Plot position over time
subplot(3,2,1);
plot(drone1.Time,drone1.POS(1,:));
title('x Coordinate Over Time');
xlabel('Time(s)')
ylabel('x(m)')
grid on;

subplot(3,2,3);
plot(drone1.Time,drone1.POS(2,:));
title('y Coordinate Over Time');
xlabel('Time(s)')
ylabel('y(m)')
grid on;

subplot(3,2,5);
plot(drone1.Time,drone1.POS(3,:));
title('z Coordinate Over Time');
xlabel('Time(s)')
ylabel('z(m)')
grid on;

% Plot orientation over time
subplot(3,2,2);
plot(drone1.Time,drone1.ROT(1,:));
title('Roll Over Time');
xlabel('Time(s)')
ylabel('Roll(°)')
grid on;

subplot(3,2,4);
plot(drone1.Time,drone1.ROT(2,:));
title('Pitch Over Time');
xlabel('Time(s)')
ylabel('Pitch(°)')
grid on;

subplot(3,2,6);
plot(drone1.Time,drone1.ROT(3,:));
title('Yaw Over Time');
xlabel('Time(s)')
ylabel('Yaw(°)')
grid on;

% Record the simulation data for Question 2
writematrix(drone1.POS, 'P0SNL.txt');
writematrix(drone1.ROT,'ROTNL.txt');
writematrix(drone1.Time, 'Time.txt');
