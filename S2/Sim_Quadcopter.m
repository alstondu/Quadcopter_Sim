% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 7;
dt          = 0.05;
TIME_SCALE  = 0.5; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


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
drone1 = Quadcopter_S2(ax1,dt);

% Run Simulation
for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS ______ %
    drone1.update(t);
    drone1.plot;
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS ______ %
    
    drawnow nocallbacks limitrate
    pause(TIME_SCALE*dt-toc); 
end

% Load data from q1 for plotting
P0SQ1 = readmatrix('P0SNL.txt');
ROTQ1 = readmatrix('ROTNL.txt');
Time = readmatrix('Time.txt');

% Plot quadcopter path for both models
figure(2)
plot3(drone1.POS(1,:),drone1.POS(2,:),drone1.POS(3,:))
hold on
plot3(P0SQ1(1,:),P0SQ1(2,:),P0SQ1(3,:))
legend('Non-Linear','LTI')
title('Quadcopter Path Q2')
xlabel('x')
ylabel('y')
zlabel('z')
grid on;

% Plot position and orientation over time
figure(3);
% Plot position over time
subplot(3,2,1);
plot(Time,P0SQ1(1,:), 'b');
hold on
plot(Time,drone1.POS(1,:), 'r--');
legend('Non-Linear','LTI')
title('x Coordinate Over Time');
xlabel('Time(s)')
ylabel('x(m)')
grid on;

subplot(3,2,3);
plot(Time,P0SQ1(2,:), 'b');
hold on
plot(Time,drone1.POS(2,:), 'r--');
legend('Non-Linear','LTI')
title('y Coordinate Over Time');
xlabel('Time(s)')
ylabel('y(m)')
grid on;

subplot(3,2,5);
plot(Time,P0SQ1(3,:), 'b');
hold on
plot(Time,drone1.POS(3,:), 'r--');
legend('Non-Linear','LTI')
title('z Coordinate Over Time');
xlabel('Time(s)')
ylabel('z(m)')
grid on;

% Plot orientation over time
subplot(3,2,2);
plot(Time,ROTQ1(1,:), 'b');
hold on
plot(Time,drone1.ROT(1,:), 'r--');
legend('Non-Linear','LTI')
title('Roll Over Time');
xlabel('Time(s)')
ylabel('Roll(°)')
grid on;

subplot(3,2,4);
plot(Time,ROTQ1(2,:), 'b');
hold on
plot(Time,drone1.ROT(2,:), 'r--');
legend('Non-Linear','LTI')
title('Pitch Over Time');
xlabel('Time(s)')
ylabel('Pitch(°)')
grid on;

subplot(3,2,6);
plot(Time,ROTQ1(3,:), 'b');
hold on
plot(Time,drone1.ROT(3,:), 'r--');
legend('Non-Linear','LTI')
title('Yaw Over Time');
xlabel('Time(s)')
ylabel('Yaw(°)')
grid on;

