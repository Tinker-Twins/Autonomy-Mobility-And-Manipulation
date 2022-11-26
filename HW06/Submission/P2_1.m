%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 6: Jacobians (Related Design and Control Issues) %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti, Olamide Akinyele

close all;
clear;
clc;

fprintf('\n=========================================================================\n');
fprintf('JACOBIANS (RELATED DESIGN AND CONTROL ISSUES)\n');
fprintf('=========================================================================\n');

%% Problem 2 (i) %%

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 2 (i)\n');
fprintf('----------------------------------------------------------------\n\n');

% RR Serial Link and Circle Parameters
L1=4; % Link length 1
L2=4; % Link length 2
alpha = linspace(0, 3*360, 3*360)';
sinalpha = sind(alpha);
cosalpha = cosd(alpha);
t = 3*10; % Elapsed time for full cycle
Radius = 1.5; % Circle radius
X0 = 1; % Circle center in X coordinate
Y0 =  1; % Circle center in Y coordinate

% Circle trajectory
X_circ= X0 + Radius*cosd(alpha);
Y_circ= Y0 + Radius*sind(alpha);

% Compute task space position velocity Xdot, Ydot for circle
[X_circ_dot, Y_circ_dot] = ComputeXYdot(X_circ,Y_circ,t);

% Compute desired joint position using RR-Serial inverse kinematics function
[thetad] = InverseKinematics(X_circ,Y_circ,L1,L2);

%% Tajectory Results for Joint-Space Closed Loop Control

% Run Euler Forward for Joint Space Closed loop Control
t = [0 30];
theta0 = [-48.5304 140.6636]'; % initial value of theta
[theta,thetadotJoint] = EulerJointCircle(t,X_circ_dot,Y_circ_dot,theta0,thetad);

% Plot forward kinematics trajectory
X_actual = L1.*cosd(theta(1,:)) + L2.*cosd(theta(1,:)+theta(2,:));
Y_actual = L1.*sind(theta(1,:)) + L2.*sind(theta(1,:)+theta(2,:));

figure(1)
plot(X_circ,Y_circ,'--','DisplayName','Desired Trajectory')
hold on 
plot(X_actual,Y_actual,'DisplayName','Actual Trajectory')
%comet(X_actual,Y_actual)
xlabel("X (m)")
ylabel("Y (m)")
title('Joint-Space Closed-Loop Control')
legend
grid on
grid minor
xlim([-1,3])
ylim([-1,3])
pbaspect([1 1 1])

%% Tajectory Results for Task-Space Closed Loop Control

% Run Euler Forward for Task Space Closed loop Control
t = [0 30];
theta0 = [-48.5304 140.6636]';
[theta,thetadottask] = EulerTaskCircle(t,X_circ_dot,Y_circ_dot,X_circ,Y_circ,theta0,thetad);

% Plot forward kinematics trajectory
X_actual = L1.*cosd(theta(1,:)) + L2.*cosd(theta(1,:)+theta(2,:));
Y_actual = L1.*sind(theta(1,:)) + L2.*sind(theta(1,:)+theta(2,:));

figure(2)
plot(X_circ,Y_circ,'--','DisplayName','Desired Trajectory')
hold on 
plot(X_actual,Y_actual,'DisplayName','Actual Trajectory')
%comet(X_actual,Y_actual)
xlabel("X (m)")
ylabel("Y (m)")
title('Task-Space Closed-Loop Control')
legend
grid on
grid minor
xlim([-1,3])
ylim([-1,3])
pbaspect([1 1 1])

%% Euler Forward Functions for Joint-Space and Task-Space Closed Loop Control

% Euler forward function to compute Joint-Space Closed Loop for Circle
function [theta,thetadot] = EulerJointCircle(t,X_circ_dot,Y_circ_dot,theta0,thetad)
    L1 = 4; % Link 1 length
    L2 = 4; % Link 2 length
    tau = 3; % Error dynamics time constant
    K1 = 1/tau; % Controller gain 1
    K2 = 1/tau; % Controller gain 2

    % Loop through the velocity and desired joint space
    a = t(1); b = t(end);
    n = linspace(a,b,3*360); % Number of iteration
    dt = 0.001;  % Time step
    theta(:,1) = theta0; % Initial theta value
    Jacobian = [-L1*sind(thetad(1,1)) -L2*sind(thetad(2,1)); L1*cosd(thetad(1,1)) L2*cosd(thetad(2,1))];
    thetadot(:,1) = pinv(Jacobian)*[X_circ_dot(1);Y_circ_dot(1)]  + diag([K1 K2])*[thetad(1,1)-theta(1,1);thetad(2,1)-theta(2,1)];
    for k = 1:length(n)-1
        theta(:,k+1) = thetad(:,k) + thetadot(:,k)*dt;
        Jacobian = [-L1*sind(thetad(1,k+1)) -L2*sind(thetad(2,k+1)); L1*cosd(thetad(1,k+1)) L2*cosd(thetad(2,k+1))];
        thetadot(:,k+1) = pinv(Jacobian)*[X_circ_dot(k+1);Y_circ_dot(k+1)]  + diag([K1 K2])*[thetad(1,k+1)-theta(1,k+1);thetad(2,k+1)-theta(2,k+1)];
    end
end

% Euler forward function to compute Task-Space Closed Loop for Circle
function [theta,thetadot] = EulerTaskCircle(t,X_circ_dot,Y_circ_dot,X_circ,Y_circ,theta0,thetad)
    L1 = 4; % Link 1 length
    L2 = 4; % Link 2 length
    K1 = -5; % Controller gain 1
    K2 = -10; % Controller gain 2

    % Loop through the velocity and desired task space
    a = t(1); b = t(end);
    n = linspace(a,b,3*360); % Number of iteration
    dt = 0.001; % Time step
    theta(:,1) = theta0; % Initial theta value
    X(1) = L1.*cosd(theta(1,1)) + L2.*cosd(theta(1,1)+theta(2,1));
    Y(1) = L1.*sind(theta(1,1)) + L2.*sind(theta(1,1)+theta(2,1));
    Jacobian = [-L1*sind(thetad(1,1)) -L2*sind(thetad(2,1)); L1*cosd(thetad(1,1)) L2*cosd(thetad(2,1))];
    thetadot(:,1) = pinv(Jacobian)*([X_circ_dot(1);Y_circ_dot(1)] + diag([K1 K2])*[X_circ(1)-X(1);Y_circ(1)-Y(1)]);
    for k = 1:length(n)-1
        theta(:,k+1) = thetad(:,k) + thetadot(:,k)*dt;
        X(k+1) = L1.*cosd(theta(1,k+1)) + L2.*cosd(theta(1,k+1)+theta(2,k+1));
        Y(k+1) = L1.*sind(theta(1,k+1)) + L2.*sind(theta(1,k+1)+theta(2,k+1));
        Jacobian = [-L1*sind(thetad(1,k+1)) -L2*sind(thetad(2,k+1)); L1*cosd(thetad(1,k+1)) L2*cosd(thetad(2,k+1))];
        thetadot(:,k+1) = pinv(Jacobian)*([X_circ_dot(k+1);Y_circ_dot(k+1)] + diag([K1 K2])*[X_circ(k+1)-X(k+1);Y_circ(k+1)-Y(k+1)]);    
    end
end

%% Inverse Kinematics Function

% This function computes the inverse kinematics of RR manipulator
function [thetad] = InverseKinematics(X,Y,L1,L2)
theta2 = acosd((X.^2 + Y.^2 - L1^2 - L2^2)./(2*L1*L2));
theta1 = atan2d(Y,X) - atan2d(L2.*sind(theta2),(L1 + L2.*cosd(theta2)));
thetad = [theta1';theta2'];
end

%% Joint Velocity Function

function [X_dot,Y_dot] = ComputeXYdot(X,Y,t)
% This function computes the velocity of circle and elipse geometry of
% interest within a range of specified time
% (X(k+1) - X(k))/dt: change in 1 degree X and Y axes == (10/360) seconds
dt = (t/360); % Change in time per degree
X_dot = zeros(size(X,1),1); % Buffer to store X dot values
Y_dot = zeros(size(Y,1),1); % Buffer to store Y dot values
X_dot(1) = (X(2) - X(1))/dt; 
Y_dot(1) = (Y(2) - Y(1))/dt;
    for i = 2:length(X)-1
        X_dot(i) = (X(i+1) - X(i))/dt;
        Y_dot(i) = (Y(i+1) - Y(i))/dt;
    end
end