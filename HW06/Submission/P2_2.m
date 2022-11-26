%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 6: Jacobians (Related Design and Control Issues) %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti, Olamide Akinyele

close all;
clear;
clc;

fprintf('\n=========================================================================\n');
fprintf('JACOBIANS (RELATED DESIGN AND CONTROL ISSUES)\n');
fprintf('=========================================================================\n');

%% Problem 2 (ii) %%

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 2 (ii)\n');
fprintf('----------------------------------------------------------------\n\n');

% RR Serial Link and Ellipse Parameters
L1=4; % Link length 1
L2=4; % Link length 2

% First Ellipse Parameters
angle1=(30); % Elipse rotation 
t1 = 3*10; % Elapse time for one full cycle.
X10=0.5; % Ellipse center in X coordinate
Y10=0.5; % Circle center in Y coordinate
a1=1.5; % Semimajor axis
b1=1; % Semiminor axis
beta1 = angle1;
sinbeta1 = sind(beta1);
cosbeta1 = cosd(beta1);
alpha1 = linspace(0, 3*360, 3*360)';
sinalpha1 = sind(alpha1);
cosalpha1 = cosd(alpha1);

% First Ellipse Trajectory
X1_elips = X10 + (a1 * cosalpha1 * cosbeta1 - b1 * sinalpha1 * sinbeta1);
Y1_elips = Y10 + (a1 * cosalpha1 * sinbeta1 + b1 * sinalpha1 * cosbeta1);

% Second Ellipse Parameters
% angle2=(0); % Elipse rotation 
% t2 = 3*10; % Elapse time for one full cycle.
% X20=0.5;  % Ellipse center in X coordinate
% Y20=0.5;  % Circle center in Y coordinate
% a2=0.5; % Semimajor axis
% b2=1.5; % Semiminor axis
% beta2 = angle2;
% sinbeta2 = sind(beta2);
% cosbeta2 = cosd(beta2);
% alpha2 = linspace(0, 3*360, 3*360)';
% sinalpha2 = sind(alpha2);
% cosalpha2 = cosd(alpha2);

% Second Ellipse Trajectory
% X2_elips = X20 + (a2 * cosalpha2 * cosbeta2 - b2 * sinalpha2 * sinbeta2);
% Y2_elips = Y20 + (a2 * cosalpha2 * sinbeta2 + b2 * sinalpha2 * cosbeta2);

% Third Ellipse Parameters
% angle3=(60); % Elipse rotation 
% t3 = 3*10; % Elapse time for one full cycle.
% X30=0.5;  % Ellipse center in X coordinate
% Y30=0.5;  % Circle center in Y coordinate
% a3=1.0; % Semimajor axis
% b3=1.5; % Semiminor axis
% beta3 = angle3;
% sinbeta3 = sind(beta3);
% cosbeta3 = cosd(beta3);
% alpha3 = linspace(0, 3*360, 3*360)';
% sinalpha3 = sind(alpha3);
% cosalpha3 = cosd(alpha3);

% Third Ellipse Trajectory
% X3_elips = X30 + (a3 * cosalpha3 * cosbeta3 - b3 * sinalpha3 * sinbeta3);
% Y3_elips = Y30 + (a3 * cosalpha3 * sinbeta3 + b3 * sinalpha3 * cosbeta3);

% Compute task space position velocity Xdot, Ydot for Ellipses
[X1_elips_dot, Y1_elips_dot] = ComputeXYdot(X1_elips,Y1_elips,t1);
% [X2_elips_dot, Y2_elips_dot] = ComputeXYdot(X2_elips,Y2_elips,t2);
% [X3_elips_dot, Y3_elips_dot] = ComputeXYdot(X3_elips,Y3_elips,t3);

% Compute desired joint position using RR-Serial inverse kinematics function
[thetad1] = InverseKinematics(X1_elips,Y1_elips,L1,L2);
% [thetad2] = InverseKinematics(X2_elips,Y2_elips,L1,L2);
% [thetad3] = InverseKinematics(X3_elips,Y3_elips,L1,L2);

%% Tajectory Results for Joint-Space Closed Loop Control

% Run Euler Forward for Joint Space Closed loop Control for first Ellipse
t1 = [0 30];
theta10 = [-39.5153 148.2149]';
[theta1,thetadotjointelips1] = EulerJointEllipse(t1,X1_elips_dot,Y1_elips_dot,theta10,thetad1);

% Forward Kinematics for first ellipse 
X1_actual = L1.*cosd(theta1(1,:)) + L2.*cosd(theta1(1,:)+theta1(2,:));
Y1_actual = L2.*sind(theta1(1,:)) + L2.*sind(theta1(1,:)+theta1(2,:));

% Run Euler Forward for Joint Space Closed loop Control for second Ellipse
% t2 = [0 30];
% theta20 = [-55.4013 163.9327]';
% [theta2,thetadotjointelips2] = EulerJointEllipse(t2,X2_elips_dot,Y2_elips_dot,theta20,thetad2);

% Forward Kinematics for second ellipse 
% X2_actual = L1.*cosd(theta2(1,:)) + L2.*cosd(theta2(1,:)+theta2(2,:));
% Y2_actual = L2.*sind(theta2(1,:)) + L2.*sind(theta2(1,:)+theta2(2,:));

% Run Euler Forward for Joint Space Closed loop Control for third Ellipse
% t3 = [0 30];
% theta30 = [-23.9889 155.5658]';
% [theta3,thetadotjointelips3] = EulerJointEllipse(t3,X3_elips_dot,Y3_elips_dot,theta30,thetad3);

% Forward Kinematics for third ellipse 
% X3_actual = L1.*cosd(theta3(1,:)) + L2.*cosd(theta3(1,:)+theta3(2,:));
% Y3_actual = L2.*sind(theta3(1,:)) + L2.*sind(theta3(1,:)+theta3(2,:));

figure(1)
plot(X1_elips,Y1_elips,'--','DisplayName','Desired Trajectory')
hold on 
plot(X1_actual,Y1_actual,'DisplayName','Actual Trajectory')
% comet(X1_actual,Y1_actual)
% hold on
% plot(X2_elips,Y2_elips,'DisplayName','Alternate Desired Trajectory 1')
% hold on 
% plot(X2_actual,Y2_actual,'--','DisplayName','Alternate Actual Trajectory 1')
% comet(X2_actual,Y2_actual)
% hold on
% plot(X3_elips,Y3_elips,'DisplayName','Alternate Desired Trajectory 2')
% hold on 
% plot(X3_actual,Y3_actual,'--','DisplayName','Alternate Actual Trajectory 2')
% comet(X3_actual,Y3_actual)
xlabel("X (m)")
ylabel("Y (m)")
title('Joint-Space Closed-Loop Control')
legend
grid on
grid minor
xlim([-1,2])
ylim([-1,2])
pbaspect([1 1 1])

%% Tajectory Results for Task-Space Closed Loop Control

% Run Euler Forward for Task Space Closed loop Control for first Ellipse
t1 = [0 30];
theta10 = [-39.5153 148.2149]';
[theta11,thetadottaskelips1] = EulerTaskEllipse(t1,X1_elips_dot,Y1_elips_dot,X1_elips,Y1_elips,theta10,thetad1);

% Forward Kinematics for first ellipse 
X11_actual = L1.*cosd(theta11(1,:)) + L2.*cosd(theta11(1,:)+theta11(2,:));
Y11_actual = L2.*sind(theta11(1,:)) + L2.*sind(theta11(1,:)+theta11(2,:));

% Run Euler Forward for Task Space Closed loop Control for second Ellipse
% t2 = [0 30];
% theta20 = [-55.4013 163.9327]';
% [theta22,thetadottaskelips2] = EulerTaskEllipse(t2,X2_elips_dot,Y2_elips_dot,X2_elips,Y2_elips,theta20,thetad2);

% Forward Kinematics for second ellipse 
% X22_actual = L1.*cosd(theta22(1,:)) + L2.*cosd(theta22(1,:)+theta22(2,:));
% Y22_actual = L2.*sind(theta22(1,:)) + L2.*sind(theta22(1,:)+theta22(2,:));

% Run Euler Forward for Task Space Closed loop Control for third Ellipse
% t3 = [0 30];
% theta30 = [-23.9889 155.5658]';
% [theta33,thetadottaskelips3] = EulerTaskEllipse(t3,X3_elips_dot,Y3_elips_dot,X3_elips,Y3_elips,theta30,thetad3);

% Forward Kinematics for third ellipse 
% X33_actual = L1.*cosd(theta33(1,:)) + L2.*cosd(theta33(1,:)+theta33(2,:));
% Y33_actual = L2.*sind(theta33(1,:)) + L2.*sind(theta33(1,:)+theta33(2,:));

figure(2)
plot(X1_elips,Y1_elips,'--','DisplayName','Desired Trajectory')
hold on 
plot(X11_actual,Y11_actual,'DisplayName','Actual Trajectory')
%comet(X11_actual,Y11_actual)
% hold on
% plot(X2_elips,Y2_elips,'DisplayName','Alternate Desired Trajectory 1')
% hold on 
% plot(X22_actual,Y22_actual,'--','DisplayName','Alternate Actual Trajectory 2')
% comet(X22_actual,Y22_actual)
% hold on
% plot(X3_elips,Y3_elips,'DisplayName','Alternate Desired Trajectory 2')
% hold on 
% plot(X33_actual,Y33_actual,'--','DisplayName','Alternate Actual Trajectory 2')
% comet(X33_actual,Y33_actual)
xlabel("X (m)")
ylabel("Y (m)")
title('Task-Space Closed-Loop Control')
legend
grid on
grid minor
xlim([-1,2])
ylim([-1,2])
pbaspect([1 1 1])

%% Euler Forward Functions for Joint-Space and Task-Space Closed Loop Control

% Euler forward function to compute Joint-Space Closed Loop for Ellipse
function [theta,thetadot] = EulerJointEllipse(t,X_elips_dot,Y_elips_dot,theta0,thetad)
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
    thetadot(:,1) = pinv(Jacobian)*[X_elips_dot(1);Y_elips_dot(1)]  + diag([K1 K2])*[thetad(1,1)-theta(1,1);thetad(2,1)-theta(2,1)];
    for k = 1:length(n)-1
        theta(:,k+1) = thetad(:,k) + thetadot(:,k)*dt;
        Jacobian = [-L1*sind(thetad(1,k+1)) -L2*sind(thetad(2,k+1)); L1*cosd(thetad(1,k+1)) L2*cosd(thetad(2,k+1))];
        thetadot(:,k+1) = pinv(Jacobian)*[X_elips_dot(k+1);Y_elips_dot(k+1)]  + diag([K1 K2])*[thetad(1,k+1)-theta(1,k+1);thetad(2,k+1)-theta(2,k+1)];
    end
end

% Euler forward function to compute Task-Space Closed Loop for circle
function [theta,thetadot] = EulerTaskEllipse(t,X_elips_dot,Y_elips_dot,X_elips,Y_elips,theta0,thetad)
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
    thetadot(:,1) = pinv(Jacobian)*([X_elips_dot(1);Y_elips_dot(1)] + diag([K1 K2])*[X_elips(1)-X(1);Y_elips(1)-Y(1)]);
    for k = 1:length(n)-1
        theta(:,k+1) = thetad(:,k) + thetadot(:,k)*dt;
        X(k+1) = L1.*cosd(theta(1,k+1)) + L2.*cosd(theta(1,k+1)+theta(2,k+1));
        Y(k+1) = L1.*sind(theta(1,k+1)) + L2.*sind(theta(1,k+1)+theta(2,k+1));
        Jacobian = [-L1*sind(thetad(1,k+1)) -L2*sind(thetad(2,k+1)); L1*cosd(thetad(1,k+1)) L2*cosd(thetad(2,k+1))];
        thetadot(:,k+1) = pinv(Jacobian)*([X_elips_dot(k+1);Y_elips_dot(k+1)] + diag([K1 K2])*[X_elips(k+1)-X(k+1);Y_elips(k+1)-Y(k+1)]);    
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