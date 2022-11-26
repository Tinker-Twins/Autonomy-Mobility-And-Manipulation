%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 6: Jacobians (Related Design and Control Issues) %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti, Olamide Akinyele

close all;
clear;
clc;

fprintf('\n=========================================================================\n');
fprintf('JACOBIANS (RELATED DESIGN AND CONTROL ISSUES)\n');
fprintf('=========================================================================\n');

%% Problem 3 (i) %%

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 3 (i)\n');
fprintf('----------------------------------------------------------------\n\n');

% Elliptical trajectory
x0 = 0.5; % Center x-offset
y0 = 0.5; % Center y-offset
a = 1.5; % Semi-major axis
b = 1; % Semi-minor axis
beta = deg2rad(30); % Orientation of major axis
alpha = 3*deg2rad(0:359);
x_ellipse = x0 + (a*cos(alpha)*cos(beta) - b*sin(alpha)*sin(beta));
y_ellipse = y0 + (a*cos(alpha)*sin(beta) + b*sin(alpha)*cos(beta));

% Circular trajectory
alpha = 3*deg2rad(0:359);
x0 = 1; % Center x-offset
y0 = 1; % Center y-offset
r = 1.5; % Radius
x_circle = x0 + r*cos(alpha);
y_circle = y0 + r*sin(alpha);

% Verify task-space trajectories
% figure()
% plot(0,0,'*')
% hold on
% plot(x_ellipse,y_ellipse,'.')
% plot(x_circle,y_circle,'.')
% xlabel("X (m)")
% ylabel("Y (m)")
% title('Desired Trajectories')
% legend('Workspace Origin','Circular Trajectory','Elliptical Trajectory','Location','SW')
% xlim([-3,3])
% ylim([-3,3])
% pbaspect([1,1,1])
% grid on
% hold off

% Robot definition using PCRTB (DH parametrization)
L1 = 2;
L2 = 3;
L3 = 1.5;
L(1) = Link([0 0 L1 0 0 3*pi/2],'R'); % First '0' doesn't mean 0. MATLAB takes it in as a variable 'theta' because we specify 'R', which defines a revolute joint
L(2) = Link([0 0 L2 0 0],'R'); % First '0' doesn't mean 0. MATLAB takes it in as a variable 'theta' because we specify 'R', which defines a revolute joint
L(3) = Link([0 0 L3 0 0],'R'); % First '0' doesn't mean 0. MATLAB takes it in as a variable 'theta' because we specify 'R', which defines a revolute joint
Robot = SerialLink(L, 'name', '3-R Planar Robot') % Concatenate the links as a serial robot

% Verify home-position of the robot
% figure()
% Robot.plot([0,0,0],'view','top')

% Calculate task-space velocity vectors
% Elliptical trajectory
a = 1.5; % Semi-major axis
b = 1; % Semi-minor axis
beta = deg2rad(30); % Orientation of major axis
q_dot_circle = deg2rad(360/10); % rad/s
i = 1;
for theta=deg2rad(0:3*360-1)
    X_dot_ellipse(:,i) = [(-a*cos(beta)*sin(theta)*q_dot_circle)-(b*sin(beta)*cos(theta)*q_dot_circle);
                          (-a*sin(beta)*sin(theta)*q_dot_circle)+(b*cos(beta)*cos(theta)*q_dot_circle)];
    i = i+1;
end
% Circular trajectory
r = 1.5; % Radius
q_dot_circle = deg2rad(360/10); % rad/s
i = 1;
for theta=deg2rad(0:3*360-1)
    X_dot_circle(:,i) = [-r*sin(theta)*q_dot_circle;
                         r*cos(theta)*q_dot_circle];
    i = i+1;
end

% Verify velocity vectors
% figure()
% hold on
% plot(X_dot_circle(1,:))
% plot(X_dot_circle(2,:))
% plot(X_dot_ellipse(1,:))
% plot(X_dot_ellipse(2,:))
% xlabel("Circumferential Progress along Trajectory (deg)")
% ylabel("Velocity (m/s)")
% title('Desired Velocities')
% legend('v_x for Circular Trajectory','v_y for Circular Trajectory','v_x for Elliptical Trajectory','v_y for Elliptical Trajectory','Location','SE')
% grid on
% hold off
% figure()
% quiver(x_circle,y_circle,X_dot_circle(1,:),X_dot_circle(2,:))
% hold on
% quiver(x_ellipse,y_ellipse,X_dot_ellipse(1,:),X_dot_ellipse(2,:))
% xlabel("X (m)")
% ylabel("Y (m)")
% title('Desired Velocity Vectors')
% legend('Circular Trajectory','Elliptical Trajectory','Location','SW')
% xlim([-3,3])
% ylim([-3,3])
% pbaspect([1,1,1])
% grid on
% hold off

% Follow the elliptical trajectory
J = [];
J_inv = [];
q_dot_ellipse = [];
theta_ellipse = [];
% Initial Joint Angles
IK_ellipse_init = Robot.ikine(transl(x_ellipse(1),y_ellipse(1), 0),'q0',[0 0 0],'mask',[1 1 1 0 0 0])'; % Mask vector (1x6 - xyzrpy) specifies the Cartesian DOF that will be ignored in reaching IK solution
theta1_rel = IK_ellipse_init(1);
theta2_rel = IK_ellipse_init(2);
theta3_rel = IK_ellipse_init(3);
theta_ellipse(:,1)=[IK_ellipse_init(1);IK_ellipse_init(1)+IK_ellipse_init(2);IK_ellipse_init(1)+IK_ellipse_init(2)+IK_ellipse_init(3)];
for i=1:3*360
    % Compute Jacobian
    J(:,:,i) = [L1*cos(theta_ellipse(1,i)), L2*cos(theta_ellipse(2,i)), L3*cos(theta_ellipse(3,i));
                L1*sin(theta_ellipse(1,i)), L2*sin(theta_ellipse(2,i)), L3*sin(theta_ellipse(3,i))];
    % Compute Jacobian Pseudoinverse
    J_inv(:,:,i) = pinv(J(:,:,i));
    % Compute Joint Velocities
    q_dot_ellipse(:,i) = J_inv(:,:,i) * X_dot_ellipse(:,i);
    % Compute Joint Angles using Euler Forward
    theta_ellipse(:,i+1) = theta_ellipse(:,i)+(q_dot_ellipse(:,i)*(10/360));
    % Plot
    %Robot.plot(IK_ellipse(:,i)','view','top')
    %Robot.plot([theta_ellipse(1,i), theta_ellipse(2,i)-theta_ellipse(1,i), theta_ellipse(3,i)-theta_ellipse(2,i)],'view','top')
    %plot(x_ellipse,y_ellipse,'--')
end

% Follow the circular trajectory
J = [];
J_inv = [];
q_dot_circle = [];
theta_circle = [];
% Initial Joint Angles
IK_circle_init = Robot.ikine(transl(x_circle(1),y_circle(1), 0),'q0',[0 0 0],'mask',[1 1 1 0 0 0])'; % Mask vector (1x6 - xyzrpy) specifies the Cartesian DOF that will be ignored in reaching IK solution
theta1_rel = IK_circle_init(1);
theta2_rel = IK_circle_init(2);
theta3_rel = IK_circle_init(3);
theta_circle(:,1)=[IK_circle_init(1);IK_circle_init(1)+IK_circle_init(2);IK_circle_init(1)+IK_circle_init(2)+IK_circle_init(3)];
for i=1:3*360
    % Compute Jacobian
    J(:,:,i) = [L1*cos(theta_circle(1,i)), L2*cos(theta_circle(2,i)), L3*cos(theta_circle(3,i));
                L1*sin(theta_circle(1,i)), L2*sin(theta_circle(2,i)), L3*sin(theta_circle(3,i))];
    % Compute Jacobian Pseudoinverse
    J_inv(:,:,i) = pinv(J(:,:,i));
    % Compute Joint Velocities
    q_dot_circle(:,i) = J_inv(:,:,i) * X_dot_circle(:,i);
    % Compute Joint Angles using Euler Forward
    theta_circle(:,i+1) = theta_circle(:,i)+(q_dot_circle(:,i)*(10/360));
    % Plot
    %Robot.plot(IK_circle(:,i)','view','top')
    %Robot.plot([theta_circle(1,i), theta_circle(2,i)-theta_circle(1,i), theta_circle(3,i)-theta_circle(2,i)],'view','top')
    %plot(x_circle,y_circle,'--')
end

%% Plot Results

% Elliptical Trajectory
EE_ellipse = [];
for i = 1:length(theta_ellipse)
    FK_ellipse = Robot.fkine([theta_ellipse(1,i), theta_ellipse(2,i)-theta_ellipse(1,i), theta_ellipse(3,i)-theta_ellipse(2,i)]);
    t_ellipse = FK_ellipse.t;
    EE_ellipse(:,i) = t_ellipse(1:2);
end
figure()
plot(x_ellipse,y_ellipse,'--','DisplayName','Desired Trajectory')
hold on 
plot(EE_ellipse(1,:),EE_ellipse(2,:),'DisplayName','Actual Trajectory')
xlabel("X (m)")
ylabel("Y (m)")
title('Pseudoinverse Method')
legend
grid on
grid minor
xlim([-1,2])
ylim([-1,2])
pbaspect([1 1 1])

% Circular Trajectory
EE_circle = [];
for i = 1:length(theta_ellipse)
    FK_circle = Robot.fkine([theta_circle(1,i), theta_circle(2,i)-theta_circle(1,i), theta_circle(3,i)-theta_circle(2,i)]);
    t_circle = FK_circle.t;
    EE_circle(:,i) = t_circle(1:2);
end
figure()
plot(x_circle,y_circle,'--','DisplayName','Desired Trajectory')
hold on 
plot(EE_circle(1,:),EE_circle(2,:),'DisplayName','Actual Trajectory')
xlabel("X (m)")
ylabel("Y (m)")
title('Pseudoinverse Method')
legend
grid on
grid minor
xlim([-1,3])
ylim([-1,3])
pbaspect([1 1 1])