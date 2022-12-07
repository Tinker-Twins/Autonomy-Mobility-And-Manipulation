%% AuE 8220 - Autonomy: Mobility and Manipulation
% Project: Mobile Manipulator %
% Authors: Tanmay Samak, Chinmay Samak

close all;
clear;
clc;

%% Robot Parameters

L = 1; % Lenght of mobile base
W = 1; % Width of mobile base
r = 0.25; % Wheel radius of mobile base
w = 0.2; % Wheel width of mobile base
L1 = 2.5; % Link 1 length of manipulator
L2 = 1; % Link 2 length of manipulator
robot_params = [L,W,r,w,L1,L2];

%% Desired Task-Space Paths

% Elliptical path
x0_ellipse = 0.5; % Center x-offset
y0_ellipse = 0.5; % Center y-offset
a_ellipse = 2.5; % Semi-major axis
b_ellipse = 0.5; % Semi-minor axis
beta_ellipse = deg2rad(30); % Orientation of major axis
traversals_ellipse = 3; % Number of traversals
alpha_ellipse = deg2rad(linspace(0, traversals_ellipse*359, traversals_ellipse*360));
x_elips = a_ellipse*cos(alpha_ellipse); % X coordinates for ellipse
y_elips = b_ellipse*sin(alpha_ellipse); % Y coordinates for ellipse
x_ellipse = x0_ellipse + x_elips*cos(beta_ellipse) - y_elips*sin(beta_ellipse); % X coordinates for translated & rotated ellipse
y_ellipse = y0_ellipse + x_elips*sin(beta_ellipse) + y_elips*cos(beta_ellipse); % Y coordinates for translated & rotated ellipse

% Rectangular path
x0_rectangle = 1.0; % Center x-offset
y0_rectangle = 1.0; % Center y-offset
a_rectangle = 3.5/2; % Semi-major side length
b_rectangle = 3.5/2; % Semi-minor side length
beta_rectangle = deg2rad(0); % Orientation of major side
traversals_rectangle = 3; % Number of traversals
alpha_rectangle = deg2rad(linspace(0, traversals_rectangle*359, traversals_rectangle*360));
x_rect = a_rectangle.*(abs(cos(alpha_rectangle)).*cos(alpha_rectangle) - abs(sin(alpha_rectangle)).*sin(alpha_rectangle)); % X coordinates for rectangle
y_rect = b_rectangle.*(abs(cos(alpha_rectangle)).*cos(alpha_rectangle) + abs(sin(alpha_rectangle)).*sin(alpha_rectangle)); % Y coordinates for rectangle
x_rectangle = x0_rectangle + x_rect*cos(beta_rectangle) - y_rect*sin(beta_rectangle); % X coordinates for translated & rotated rectangle
y_rectangle = y0_rectangle + x_rect*sin(beta_rectangle) + y_rect*cos(beta_rectangle); % Y coordinates for translated & rotated rectangle

% Verify task-space paths
% figure(1)
% plot(x_ellipse,y_ellipse)
% %comet(x_ellipse,y_ellipse)
% hold on
% plot(x_rectangle,y_rectangle)
% %comet(x_rectangle,y_rectangle)
% plot(0,0,'+k')
% xlabel("X (m)")
% ylabel("Y (m)")
% title('Desired Paths')
% legend('Elliptical Path','Rectangular Path','Workspace Origin','Location','SW')
% xlim([-5,5])
% ylim([-5,5])
% pbaspect([1,1,1])
% hold off

%% Desired Task-Space Trajectories

% Elliptical trajectory
cycle_time_ellipse = 20; % Cycle time
alpha_dot_ellipse = deg2rad(360/cycle_time_ellipse); % Phaser velocity
i = 1;
for alpha_ellipse=deg2rad(0:traversals_ellipse*360-1)
    x_dot_elips = -a_ellipse.*sin(alpha_ellipse).*alpha_dot_ellipse; % X component of velocity vector
    y_dot_elips = b_ellipse.*cos(alpha_ellipse).*alpha_dot_ellipse; % Y component of velocity vector
    x_dot_ellipse = x_dot_elips*cos(beta_ellipse) - y_dot_elips*sin(beta_ellipse); % X component of rotated velocity vector
    y_dot_ellipse = x_dot_elips*sin(beta_ellipse) + y_dot_elips*cos(beta_ellipse); % Y component of rotated velocity vector
    X_dot_ellipse(:,i) = [x_dot_ellipse; y_dot_ellipse];
    i = i+1;
end

% Rectangular trajectory
cycle_time_rectangle = 20; % Cycle time
alpha_dot_rectangle = deg2rad(360/cycle_time_rectangle); % Phaser velocity
i = 1;
for alpha_rectangle=deg2rad(0:traversals_rectangle*360-1)
    x_dot_rect = a_rectangle*((-sin(alpha_rectangle).*cos(alpha_rectangle)./abs(cos(alpha_rectangle))).*cos(alpha_rectangle)*alpha_dot_rectangle + abs(cos(alpha_rectangle)).*(-sin(alpha_rectangle)*alpha_dot_rectangle) - ...
                 (sin(alpha_rectangle).*cos(alpha_rectangle)./abs(sin(alpha_rectangle))).*sin(alpha_rectangle)*alpha_dot_rectangle - abs(sin(alpha_rectangle)).*(cos(alpha_rectangle)*alpha_dot_rectangle)); % X component of velocity vector
    y_dot_rect = a_rectangle*((-sin(alpha_rectangle).*cos(alpha_rectangle)./abs(cos(alpha_rectangle))).*cos(alpha_rectangle)*alpha_dot_rectangle + abs(cos(alpha_rectangle)).*(-sin(alpha_rectangle)*alpha_dot_rectangle) + ...
                 (sin(alpha_rectangle).*cos(alpha_rectangle)./abs(sin(alpha_rectangle))).*sin(alpha_rectangle)*alpha_dot_rectangle + abs(sin(alpha_rectangle)).*(cos(alpha_rectangle)*alpha_dot_rectangle)); % Y component of velocity vector
    x_dot_rectangle = x_dot_rect*cos(beta_rectangle) - y_dot_rect*sin(beta_rectangle); % X component of rotated velocity vector
    y_dot_rectangle = x_dot_rect*sin(beta_rectangle) + y_dot_rect*cos(beta_rectangle); % Y component of rotated velocity vector
    X_dot_rectangle(:,i) = [x_dot_rectangle; y_dot_rectangle];
    i = i+1;
end
X_dot_rectangle(isnan(X_dot_rectangle)) = 0; % Replace NaN elements with zero

% Verify velocity vectors
% figure(2)
% hold on
% plot(X_dot_ellipse(1,:))
% plot(X_dot_ellipse(2,:))
% plot(X_dot_rectangle(1,:))
% plot(X_dot_rectangle(2,:))
% xlabel("Circumferential Progress Along Trajectory (deg)")
% ylabel("Velocity (m/s)")
% title('Desired Velocities')
% legend('v_x for Elliptical Trajectory','v_y for Elliptical Trajectory','v_x for Rectangular Trajectory','v_y for Rectangular Trajectory','Location','SE')
% hold off

% Verify task-space trajectories
% figure(3)
% quiver(x_ellipse,y_ellipse,X_dot_ellipse(1,:),X_dot_ellipse(2,:))
% hold on
% quiver(x_rectangle,y_rectangle,X_dot_rectangle(1,:),X_dot_rectangle(2,:))
% xlabel("X (m)")
% ylabel("Y (m)")
% title('Desired Trajectories')
% legend('Elliptical Trajectory','Rectangular Trajectory','Location','SW')
% xlim([-5,5])
% ylim([-5,5])
% pbaspect([1,1,1])
% hold off

% %% Redundancy Resolution Using Pseudoinverse Method - Ellipse
% 
% % CoppeliaSim Remote API Connection
% vrep = remApi('remoteApi');
% vrep.simxFinish(-1);
% clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
% if clientID > -1
%     disp('Connected to CoppeliaSim!');
%     % Get Object Handles from Simulator
%     [~, WMR] = vrep.simxGetObjectHandle(clientID, 'Mobile_Base', vrep.simx_opmode_blocking);
%     [~, W_L] = vrep.simxGetObjectHandle(clientID, 'Left_Wheel', vrep.simx_opmode_blocking);
%     [~, W_R] = vrep.simxGetObjectHandle(clientID, 'Right_Wheel', vrep.simx_opmode_blocking);
%     [~, J_1] = vrep.simxGetObjectHandle(clientID, 'Proximal_Link_Joint', vrep.simx_opmode_blocking);
%     [~, J_2] = vrep.simxGetObjectHandle(clientID, 'Distal_Link_Joint', vrep.simx_opmode_blocking);
%     while true
%         % Follow the elliptical trajectory
%         X_dot = [X_dot_ellipse; ones(1,length(X_dot_ellipse))*alpha_dot_ellipse; zeros(1,length(X_dot_ellipse))];
%         J = [];
%         J_inv = [];
%         q_dot_ellipse = [];
%         q_ellipse = [];
%         X_E_ellipse = [];
%         Y_E_ellipse = [];
%         phi_l = 0; phi_r = 0;
%         % Initial configuration
%         X = x_ellipse(1)-robot_params(5);
%         Y = y_ellipse(1)-robot_params(6);
%         phi = deg2rad(0);
%         theta2 = real(acos(((x_ellipse(1)-X)^2 + (y_ellipse(1)-Y)^2 - robot_params(5)^2 - robot_params(6)^2)./(2*robot_params(5)*robot_params(6))));
%         theta1 = atan2(y_ellipse(1)-Y,x_ellipse(1)-X) - atan2(robot_params(6).*sin(theta2),(robot_params(5) + robot_params(6).*cos(theta2))) - phi;
%         q_ellipse(:,1)=[X;Y;phi;theta1;theta2];
%         for i=1:traversals_ellipse*360
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i))-robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J14 = -robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i))-robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J15 = -robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i))+robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J24 = robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i))+robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J25 = robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_ellipse(3,i));
%             J42 = cos(q_ellipse(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45];
%             % Compute Jacobian Pseudoinverse
%             J_inv(:,:,i) = pinv(J(:,:,i));
%             % Compute Joint Velocities
%             q_dot_ellipse(:,i) = J_inv(:,:,i) * X_dot(:,i);
%             % Compute Joint Angles using Euler Forward
%             q_ellipse(:,i+1) = q_ellipse(:,i)+(q_dot_ellipse(:,i).*(cycle_time_ellipse/360));
%             % Compute Wheel Velocities
%             phi_l_dot = (2*(q_dot_ellipse(1,i)*cos(q_ellipse(3,i)) + q_dot_ellipse(2,i)*sin(q_ellipse(3,i))) - robot_params(2)*q_dot_ellipse(3,i))/(2*robot_params(3));
%             phi_r_dot = (2*(q_dot_ellipse(1,i)*cos(q_ellipse(3,i)) + q_dot_ellipse(2,i)*sin(q_ellipse(3,i))) + robot_params(2)*q_dot_ellipse(3,i))/(2*robot_params(3));
%             % Compute Wheel Angles using Euler Forward
%             phi_l = phi_l + phi_l_dot*(cycle_time_ellipse/360);
%             phi_r = phi_r + phi_r_dot*(cycle_time_ellipse/360);
%             % Get Pose of Mobile Base
%             robot_x = q_ellipse(1,i); robot_y = q_ellipse(2,i); robot_z = 0.2850;
%             [robot_qx,robot_qy,robot_qz,robot_qw] = EA2Q(0,-pi/2,q_ellipse(3,i)-pi/2);
%             % Command Robot Joints
%             [~] = vrep.simxSetObjectPosition(clientID, WMR, -1, [robot_x,robot_y,robot_z], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetObjectQuaternion(clientID, WMR, -1, [robot_qx,robot_qy,robot_qz,robot_qw], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_L, phi_l, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_R, -phi_r, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_1, q_ellipse(4,i), vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_2, q_ellipse(5,i), vrep.simx_opmode_streaming);
%             %figure(4)
%             %PlotRobot(q_ellipse(:,i),robot_params)
%             pause(cycle_time_ellipse/360);
%         end
%         % Stop Simulation and MATLAB Script
%         disp("Simulation Completed!")
%         [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
%         break
%     end
% else
%     disp('Error Connecting to CoppeliaSim!');
% end

% %% Redundancy Resolution Using Pseudoinverse Method - Square
% 
% % CoppeliaSim Remote API Connection
% vrep = remApi('remoteApi');
% vrep.simxFinish(-1);
% clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
% if clientID > -1
%     disp('Connected to CoppeliaSim!');
%     % Get Object Handles from Simulator
%     [~, WMR] = vrep.simxGetObjectHandle(clientID, 'Mobile_Base', vrep.simx_opmode_blocking);
%     [~, W_L] = vrep.simxGetObjectHandle(clientID, 'Left_Wheel', vrep.simx_opmode_blocking);
%     [~, W_R] = vrep.simxGetObjectHandle(clientID, 'Right_Wheel', vrep.simx_opmode_blocking);
%     [~, J_1] = vrep.simxGetObjectHandle(clientID, 'Proximal_Link_Joint', vrep.simx_opmode_blocking);
%     [~, J_2] = vrep.simxGetObjectHandle(clientID, 'Distal_Link_Joint', vrep.simx_opmode_blocking);
%     while true
%         % Follow the rectangular trajectory
%         X_dot = [X_dot_rectangle; ones(1,length(X_dot_rectangle))*alpha_dot_rectangle; zeros(1,length(X_dot_rectangle))];
%         J = [];
%         J_inv = [];
%         q_dot_rectangle = [];
%         q_rectangle = [];
%         X_E_rectangle = [];
%         Y_E_rectangle = [];
%         phi_l = 0; phi_r = 0;
%         % Initial configuration
%         X = x_rectangle(1)-robot_params(5);
%         Y = y_rectangle(1)-robot_params(6);
%         phi = deg2rad(0);
%         theta2 = real(acos(((x_rectangle(1)-X)^2 + (y_rectangle(1)-Y)^2 - robot_params(5)^2 - robot_params(6)^2)./(2*robot_params(5)*robot_params(6))));
%         theta1 = atan2(y_rectangle(1)-Y,x_rectangle(1)-X) - atan2(robot_params(6).*sin(theta2),(robot_params(5) + robot_params(6).*cos(theta2))) - phi;
%         q_rectangle(:,1)=[X;Y;phi;theta1;theta2];
%         for i=1:traversals_rectangle*360
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i))-robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J14 = -robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i))-robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J15 = -robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i))+robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J24 = robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i))+robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J25 = robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_rectangle(3,i));
%             J42 = cos(q_rectangle(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45];
%             % Compute Jacobian Pseudoinverse
%             J_inv(:,:,i) = pinv(J(:,:,i));
%             % Compute Joint Velocities
%             q_dot_rectangle(:,i) = J_inv(:,:,i) * X_dot(:,i);
%             % Compute Joint Angles using Euler Forward
%             q_rectangle(:,i+1) = q_rectangle(:,i)+(q_dot_rectangle(:,i).*(cycle_time_rectangle/360));
%             % Compute Wheel Velocities
%             phi_l_dot = (2*(q_dot_rectangle(1,i)*cos(q_rectangle(3,i)) + q_dot_rectangle(2,i)*sin(q_rectangle(3,i))) - robot_params(2)*q_dot_rectangle(3,i))/(2*robot_params(3));
%             phi_r_dot = (2*(q_dot_rectangle(1,i)*cos(q_rectangle(3,i)) + q_dot_rectangle(2,i)*sin(q_rectangle(3,i))) + robot_params(2)*q_dot_rectangle(3,i))/(2*robot_params(3));
%             % Compute Wheel Angles using Euler Forward
%             phi_l = phi_l + phi_l_dot*(cycle_time_ellipse/360);
%             phi_r = phi_r + phi_r_dot*(cycle_time_ellipse/360);
%             % Get Pose of Mobile Base
%             robot_x = q_rectangle(1,i); robot_y = q_rectangle(2,i); robot_z = 0.2850;
%             [robot_qx,robot_qy,robot_qz,robot_qw] = EA2Q(0,-pi/2,q_rectangle(3,i)-pi/2);
%             % Command Robot Joints
%             [~] = vrep.simxSetObjectPosition(clientID, WMR, -1, [robot_x,robot_y,robot_z], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetObjectQuaternion(clientID, WMR, -1, [robot_qx,robot_qy,robot_qz,robot_qw], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_L, phi_l, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_R, -phi_r, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_1, q_rectangle(4,i), vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_2, q_rectangle(5,i), vrep.simx_opmode_streaming);
%             %figure(4)
%             %PlotRobot(q_rectangle(:,i),robot_params)
%             pause(cycle_time_rectangle/360);
%         end
%         % Stop Simulation and MATLAB Script
%         disp("Simulation Completed!")
%         [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
%         break
%     end
% else
%     disp('Error Connecting to CoppeliaSim!');
% end

% %% Redundancy Resolution Using Augmented Task-Space Method (Theta_1_Dot + Theta_2_Dot = 0)  - Ellipse
% 
% % CoppeliaSim Remote API Connection
% vrep = remApi('remoteApi');
% vrep.simxFinish(-1);
% clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
% if clientID > -1
%     disp('Connected to CoppeliaSim!');
%     % Get Object Handles from Simulator
%     [~, WMR] = vrep.simxGetObjectHandle(clientID, 'Mobile_Base', vrep.simx_opmode_blocking);
%     [~, W_L] = vrep.simxGetObjectHandle(clientID, 'Left_Wheel', vrep.simx_opmode_blocking);
%     [~, W_R] = vrep.simxGetObjectHandle(clientID, 'Right_Wheel', vrep.simx_opmode_blocking);
%     [~, J_1] = vrep.simxGetObjectHandle(clientID, 'Proximal_Link_Joint', vrep.simx_opmode_blocking);
%     [~, J_2] = vrep.simxGetObjectHandle(clientID, 'Distal_Link_Joint', vrep.simx_opmode_blocking);
%     while true
%         % Follow the elliptical trajectory
%         X_dot = [X_dot_ellipse; ones(1,length(X_dot_ellipse))*alpha_dot_ellipse; zeros(1,length(X_dot_ellipse)); zeros(1,length(X_dot_ellipse))];
%         J = [];
%         J_inv = [];
%         q_dot_ellipse = [];
%         q_ellipse = [];
%         X_E_ellipse = [];
%         Y_E_ellipse = [];
%         phi_l = 0; phi_r = 0;
%         % Initial configuration
%         X = x_ellipse(1)-robot_params(5);
%         Y = y_ellipse(1)-robot_params(6);
%         phi = deg2rad(0);
%         theta2 = real(acos(((x_ellipse(1)-X)^2 + (y_ellipse(1)-Y)^2 - robot_params(5)^2 - robot_params(6)^2)./(2*robot_params(5)*robot_params(6))));
%         theta1 = atan2(y_ellipse(1)-Y,x_ellipse(1)-X) - atan2(robot_params(6).*sin(theta2),(robot_params(5) + robot_params(6).*cos(theta2))) - phi;
%         q_ellipse(:,1)=[X;Y;phi;theta1;theta2];
%         for i=1:traversals_ellipse*360
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i))-robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J14 = -robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i))-robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J15 = -robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i))+robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J24 = robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i))+robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J25 = robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_ellipse(3,i));
%             J42 = cos(q_ellipse(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J51 = 0;
%             J52 = 0;
%             J53 = 0;
%             J54 = 1;
%             J55 = 1;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45;
%                         J51 J52 J53 J54 J55];
%             % Compute Jacobian Pseudoinverse
%             J_inv(:,:,i) = inv(J(:,:,i));
%             % Compute Joint Velocities
%             q_dot_ellipse(:,i) = J_inv(:,:,i) * X_dot(:,i);
%             % Compute Joint Angles using Euler Forward
%             q_ellipse(:,i+1) = q_ellipse(:,i)+(q_dot_ellipse(:,i).*(cycle_time_ellipse/360));
%             % Compute Wheel Velocities
%             phi_l_dot = (2*(q_dot_ellipse(1,i)*cos(q_ellipse(3,i)) + q_dot_ellipse(2,i)*sin(q_ellipse(3,i))) - robot_params(2)*q_dot_ellipse(3,i))/(2*robot_params(3));
%             phi_r_dot = (2*(q_dot_ellipse(1,i)*cos(q_ellipse(3,i)) + q_dot_ellipse(2,i)*sin(q_ellipse(3,i))) + robot_params(2)*q_dot_ellipse(3,i))/(2*robot_params(3));
%             % Compute Wheel Angles using Euler Forward
%             phi_l = phi_l + phi_l_dot*(cycle_time_ellipse/360);
%             phi_r = phi_r + phi_r_dot*(cycle_time_ellipse/360);
%             % Get Pose of Mobile Base
%             robot_x = q_ellipse(1,i); robot_y = q_ellipse(2,i); robot_z = 0.2850;
%             [robot_qx,robot_qy,robot_qz,robot_qw] = EA2Q(0,-pi/2,q_ellipse(3,i)-pi/2);
%             % Command Robot Joints
%             [~] = vrep.simxSetObjectPosition(clientID, WMR, -1, [robot_x,robot_y,robot_z], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetObjectQuaternion(clientID, WMR, -1, [robot_qx,robot_qy,robot_qz,robot_qw], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_L, phi_l, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_R, -phi_r, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_1, q_ellipse(4,i), vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_2, q_ellipse(5,i), vrep.simx_opmode_streaming);
%             %figure(4)
%             %PlotRobot(q_ellipse(:,i),robot_params)
%             pause(cycle_time_ellipse/360);
%         end
%         % Stop Simulation and MATLAB Script
%         disp("Simulation Completed!")
%         [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
%         break
%     end
% else
%     disp('Error Connecting to CoppeliaSim!');
% end

% %% Redundancy Resolution Using Augmented Task-Space Method (Theta_1_Dot + Theta_2_Dot = 0)  - Square
% 
% % CoppeliaSim Remote API Connection
% vrep = remApi('remoteApi');
% vrep.simxFinish(-1);
% clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
% if clientID > -1
%     disp('Connected to CoppeliaSim!');
%     % Get Object Handles from Simulator
%     [~, WMR] = vrep.simxGetObjectHandle(clientID, 'Mobile_Base', vrep.simx_opmode_blocking);
%     [~, W_L] = vrep.simxGetObjectHandle(clientID, 'Left_Wheel', vrep.simx_opmode_blocking);
%     [~, W_R] = vrep.simxGetObjectHandle(clientID, 'Right_Wheel', vrep.simx_opmode_blocking);
%     [~, J_1] = vrep.simxGetObjectHandle(clientID, 'Proximal_Link_Joint', vrep.simx_opmode_blocking);
%     [~, J_2] = vrep.simxGetObjectHandle(clientID, 'Distal_Link_Joint', vrep.simx_opmode_blocking);
%     while true
%         % Follow the rectangular trajectory
%         X_dot = [X_dot_rectangle; ones(1,length(X_dot_rectangle))*alpha_dot_rectangle; zeros(1,length(X_dot_rectangle)); zeros(1,length(X_dot_rectangle))];
%         J = [];
%         J_inv = [];
%         q_dot_rectangle = [];
%         q_rectangle = [];
%         X_E_rectangle = [];
%         Y_E_rectangle = [];
%         phi_l = 0; phi_r = 0;
%         % Initial configuration
%         X = x_rectangle(1)-robot_params(5);
%         Y = y_rectangle(1)-robot_params(6);
%         phi = deg2rad(0);
%         theta2 = real(acos(((x_rectangle(1)-X)^2 + (y_rectangle(1)-Y)^2 - robot_params(5)^2 - robot_params(6)^2)./(2*robot_params(5)*robot_params(6))));
%         theta1 = atan2(y_rectangle(1)-Y,x_rectangle(1)-X) - atan2(robot_params(6).*sin(theta2),(robot_params(5) + robot_params(6).*cos(theta2))) - phi;
%         q_rectangle(:,1)=[X;Y;phi;theta1;theta2];
%         for i=1:traversals_rectangle*360
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i))-robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J14 = -robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i))-robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J15 = -robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i))+robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J24 = robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i))+robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J25 = robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_rectangle(3,i));
%             J42 = cos(q_rectangle(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J51 = 0;
%             J52 = 0;
%             J53 = 0;
%             J54 = 1;
%             J55 = 1;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45;
%                         J51 J52 J53 J54 J55];
%             % Compute Jacobian Pseudoinverse
%             J_inv(:,:,i) = inv(J(:,:,i));
%             % Compute Joint Velocities
%             q_dot_rectangle(:,i) = J_inv(:,:,i) * X_dot(:,i);
%             % Compute Joint Angles using Euler Forward
%             q_rectangle(:,i+1) = q_rectangle(:,i)+(q_dot_rectangle(:,i).*(cycle_time_rectangle/360));
%             % Compute Wheel Velocities
%             phi_l_dot = (2*(q_dot_rectangle(1,i)*cos(q_rectangle(3,i)) + q_dot_rectangle(2,i)*sin(q_rectangle(3,i))) - robot_params(2)*q_dot_rectangle(3,i))/(2*robot_params(3));
%             phi_r_dot = (2*(q_dot_rectangle(1,i)*cos(q_rectangle(3,i)) + q_dot_rectangle(2,i)*sin(q_rectangle(3,i))) + robot_params(2)*q_dot_rectangle(3,i))/(2*robot_params(3));
%             % Compute Wheel Angles using Euler Forward
%             phi_l = phi_l + phi_l_dot*(cycle_time_ellipse/360);
%             phi_r = phi_r + phi_r_dot*(cycle_time_ellipse/360);
%             % Get Pose of Mobile Base
%             robot_x = q_rectangle(1,i); robot_y = q_rectangle(2,i); robot_z = 0.2850;
%             [robot_qx,robot_qy,robot_qz,robot_qw] = EA2Q(0,-pi/2,q_rectangle(3,i)-pi/2);
%             % Command Robot Joints
%             [~] = vrep.simxSetObjectPosition(clientID, WMR, -1, [robot_x,robot_y,robot_z], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetObjectQuaternion(clientID, WMR, -1, [robot_qx,robot_qy,robot_qz,robot_qw], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_L, phi_l, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_R, -phi_r, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_1, q_rectangle(4,i), vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_2, q_rectangle(5,i), vrep.simx_opmode_streaming);
%             %figure(4)
%             %PlotRobot(q_rectangle(:,i),robot_params)
%             pause(cycle_time_rectangle/360);
%         end
%         % Stop Simulation and MATLAB Script
%         disp("Simulation Completed!")
%         [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
%         break
%     end
% else
%     disp('Error Connecting to CoppeliaSim!');
% end

% %% Redundancy Resolution Using Augmented Task-Space Method (Theta_1_Dot = 0) - Ellipse
% 
% % CoppeliaSim Remote API Connection
% vrep = remApi('remoteApi');
% vrep.simxFinish(-1);
% clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
% if clientID > -1
%     disp('Connected to CoppeliaSim!');
%     % Get Object Handles from Simulator
%     [~, WMR] = vrep.simxGetObjectHandle(clientID, 'Mobile_Base', vrep.simx_opmode_blocking);
%     [~, W_L] = vrep.simxGetObjectHandle(clientID, 'Left_Wheel', vrep.simx_opmode_blocking);
%     [~, W_R] = vrep.simxGetObjectHandle(clientID, 'Right_Wheel', vrep.simx_opmode_blocking);
%     [~, J_1] = vrep.simxGetObjectHandle(clientID, 'Proximal_Link_Joint', vrep.simx_opmode_blocking);
%     [~, J_2] = vrep.simxGetObjectHandle(clientID, 'Distal_Link_Joint', vrep.simx_opmode_blocking);
%     while true
%         % Follow the elliptical trajectory
%         X_dot = [X_dot_ellipse; ones(1,length(X_dot_ellipse))*alpha_dot_ellipse; zeros(1,length(X_dot_ellipse)); zeros(1,length(X_dot_ellipse))];
%         J = [];
%         J_inv = [];
%         q_dot_ellipse = [];
%         q_ellipse = [];
%         X_E_ellipse = [];
%         Y_E_ellipse = [];
%         phi_l = 0; phi_r = 0;
%         % Initial configuration
%         X = x_ellipse(1)-robot_params(5);
%         Y = y_ellipse(1)-robot_params(6);
%         phi = deg2rad(0);
%         theta2 = real(acos(((x_ellipse(1)-X)^2 + (y_ellipse(1)-Y)^2 - robot_params(5)^2 - robot_params(6)^2)./(2*robot_params(5)*robot_params(6))));
%         theta1 = atan2(y_ellipse(1)-Y,x_ellipse(1)-X) - atan2(robot_params(6).*sin(theta2),(robot_params(5) + robot_params(6).*cos(theta2))) - phi;
%         q_ellipse(:,1)=[X;Y;phi;theta1;theta2];
%         for i=1:traversals_ellipse*360
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i))-robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J14 = -robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i))-robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J15 = -robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i))+robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J24 = robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i))+robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J25 = robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_ellipse(3,i));
%             J42 = cos(q_ellipse(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J51 = 0;
%             J52 = 0;
%             J53 = 0;
%             J54 = 1;
%             J55 = 0;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45;
%                         J51 J52 J53 J54 J55];
%             % Compute Jacobian Pseudoinverse
%             J_inv(:,:,i) = inv(J(:,:,i));
%             % Compute Joint Velocities
%             q_dot_ellipse(:,i) = J_inv(:,:,i) * X_dot(:,i);
%             % Compute Joint Angles using Euler Forward
%             q_ellipse(:,i+1) = q_ellipse(:,i)+(q_dot_ellipse(:,i).*(cycle_time_ellipse/360));
%             % Compute Wheel Velocities
%             phi_l_dot = (2*(q_dot_ellipse(1,i)*cos(q_ellipse(3,i)) + q_dot_ellipse(2,i)*sin(q_ellipse(3,i))) - robot_params(2)*q_dot_ellipse(3,i))/(2*robot_params(3));
%             phi_r_dot = (2*(q_dot_ellipse(1,i)*cos(q_ellipse(3,i)) + q_dot_ellipse(2,i)*sin(q_ellipse(3,i))) + robot_params(2)*q_dot_ellipse(3,i))/(2*robot_params(3));
%             % Compute Wheel Angles using Euler Forward
%             phi_l = phi_l + phi_l_dot*(cycle_time_ellipse/360);
%             phi_r = phi_r + phi_r_dot*(cycle_time_ellipse/360);
%             % Get Pose of Mobile Base
%             robot_x = q_ellipse(1,i); robot_y = q_ellipse(2,i); robot_z = 0.2850;
%             [robot_qx,robot_qy,robot_qz,robot_qw] = EA2Q(0,-pi/2,q_ellipse(3,i)-pi/2);
%             % Command Robot Joints
%             [~] = vrep.simxSetObjectPosition(clientID, WMR, -1, [robot_x,robot_y,robot_z], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetObjectQuaternion(clientID, WMR, -1, [robot_qx,robot_qy,robot_qz,robot_qw], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_L, phi_l, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_R, -phi_r, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_1, q_ellipse(4,i), vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_2, q_ellipse(5,i), vrep.simx_opmode_streaming);
%             %figure(4)
%             %PlotRobot(q_ellipse(:,i),robot_params)
%             pause(cycle_time_ellipse/360);
%         end
%         % Stop Simulation and MATLAB Script
%         disp("Simulation Completed!")
%         [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
%         break
%     end
% else
%     disp('Error Connecting to CoppeliaSim!');
% end

% %% Redundancy Resolution Using Augmented Task-Space Method (Theta_1_Dot = 0) - Square
% 
% % CoppeliaSim Remote API Connection
% vrep = remApi('remoteApi');
% vrep.simxFinish(-1);
% clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
% if clientID > -1
%     disp('Connected to CoppeliaSim!');
%     % Get Object Handles from Simulator
%     [~, WMR] = vrep.simxGetObjectHandle(clientID, 'Mobile_Base', vrep.simx_opmode_blocking);
%     [~, W_L] = vrep.simxGetObjectHandle(clientID, 'Left_Wheel', vrep.simx_opmode_blocking);
%     [~, W_R] = vrep.simxGetObjectHandle(clientID, 'Right_Wheel', vrep.simx_opmode_blocking);
%     [~, J_1] = vrep.simxGetObjectHandle(clientID, 'Proximal_Link_Joint', vrep.simx_opmode_blocking);
%     [~, J_2] = vrep.simxGetObjectHandle(clientID, 'Distal_Link_Joint', vrep.simx_opmode_blocking);
%     while true
%         % Follow the rectangular trajectory
%         X_dot = [X_dot_rectangle; ones(1,length(X_dot_rectangle))*alpha_dot_rectangle; zeros(1,length(X_dot_rectangle)); zeros(1,length(X_dot_rectangle))];
%         J = [];
%         J_inv = [];
%         q_dot_rectangle = [];
%         q_rectangle = [];
%         X_E_rectangle = [];
%         Y_E_rectangle = [];
%         phi_l = 0; phi_r = 0;
%         % Initial configuration
%         X = x_rectangle(1)-robot_params(5);
%         Y = y_rectangle(1)-robot_params(6);
%         phi = deg2rad(0);
%         theta2 = real(acos(((x_rectangle(1)-X)^2 + (y_rectangle(1)-Y)^2 - robot_params(5)^2 - robot_params(6)^2)./(2*robot_params(5)*robot_params(6))));
%         theta1 = atan2(y_rectangle(1)-Y,x_rectangle(1)-X) - atan2(robot_params(6).*sin(theta2),(robot_params(5) + robot_params(6).*cos(theta2))) - phi;
%         q_rectangle(:,1)=[X;Y;phi;theta1;theta2];
%         for i=1:traversals_rectangle*360
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i))-robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J14 = -robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i))-robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J15 = -robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i))+robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J24 = robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i))+robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J25 = robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_rectangle(3,i));
%             J42 = cos(q_rectangle(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J51 = 0;
%             J52 = 0;
%             J53 = 0;
%             J54 = 1;
%             J55 = 0;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45;
%                         J51 J52 J53 J54 J55];
%             % Compute Jacobian Pseudoinverse
%             J_inv(:,:,i) = inv(J(:,:,i));
%             % Compute Joint Velocities
%             q_dot_rectangle(:,i) = J_inv(:,:,i) * X_dot(:,i);
%             % Compute Joint Angles using Euler Forward
%             q_rectangle(:,i+1) = q_rectangle(:,i)+(q_dot_rectangle(:,i).*(cycle_time_rectangle/360));
%             % Compute Wheel Velocities
%             phi_l_dot = (2*(q_dot_rectangle(1,i)*cos(q_rectangle(3,i)) + q_dot_rectangle(2,i)*sin(q_rectangle(3,i))) - robot_params(2)*q_dot_rectangle(3,i))/(2*robot_params(3));
%             phi_r_dot = (2*(q_dot_rectangle(1,i)*cos(q_rectangle(3,i)) + q_dot_rectangle(2,i)*sin(q_rectangle(3,i))) + robot_params(2)*q_dot_rectangle(3,i))/(2*robot_params(3));
%             % Compute Wheel Angles using Euler Forward
%             phi_l = phi_l + phi_l_dot*(cycle_time_ellipse/360);
%             phi_r = phi_r + phi_r_dot*(cycle_time_ellipse/360);
%             % Get Pose of Mobile Base
%             robot_x = q_rectangle(1,i); robot_y = q_rectangle(2,i); robot_z = 0.2850;
%             [robot_qx,robot_qy,robot_qz,robot_qw] = EA2Q(0,-pi/2,q_rectangle(3,i)-pi/2);
%             % Command Robot Joints
%             [~] = vrep.simxSetObjectPosition(clientID, WMR, -1, [robot_x,robot_y,robot_z], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetObjectQuaternion(clientID, WMR, -1, [robot_qx,robot_qy,robot_qz,robot_qw], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_L, phi_l, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_R, -phi_r, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_1, q_rectangle(4,i), vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_2, q_rectangle(5,i), vrep.simx_opmode_streaming);
%             %figure(4)
%             %PlotRobot(q_rectangle(:,i),robot_params)
%             pause(cycle_time_rectangle/360);
%         end
%         % Stop Simulation and MATLAB Script
%         disp("Simulation Completed!")
%         [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
%         break
%     end
% else
%     disp('Error Connecting to CoppeliaSim!');
% end

% %% Redundancy Resolution Using Artificial Potential Method - Ellipse
% 
% % CoppeliaSim Remote API Connection
% vrep = remApi('remoteApi');
% vrep.simxFinish(-1);
% clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
% if clientID > -1
%     disp('Connected to CoppeliaSim!');
%     % Get Object Handles from Simulator
%     [~, WMR] = vrep.simxGetObjectHandle(clientID, 'Mobile_Base', vrep.simx_opmode_blocking);
%     [~, W_L] = vrep.simxGetObjectHandle(clientID, 'Left_Wheel', vrep.simx_opmode_blocking);
%     [~, W_R] = vrep.simxGetObjectHandle(clientID, 'Right_Wheel', vrep.simx_opmode_blocking);
%     [~, J_1] = vrep.simxGetObjectHandle(clientID, 'Proximal_Link_Joint', vrep.simx_opmode_blocking);
%     [~, J_2] = vrep.simxGetObjectHandle(clientID, 'Distal_Link_Joint', vrep.simx_opmode_blocking);
%     while true
%         % Follow the elliptical trajectory
%         X_dot = [X_dot_ellipse; ones(1,length(X_dot_ellipse))*alpha_dot_ellipse; zeros(1,length(X_dot_ellipse))];
%         J = [];
%         J_inv = [];
%         q_dot_ellipse = [];
%         q_ellipse = [];
%         X_E_ellipse = [];
%         Y_E_ellipse = [];
%         phi_l = 0; phi_r = 0;
%         % Initial configuration
%         X = x_ellipse(1)-robot_params(5);
%         Y = y_ellipse(1)-robot_params(6);
%         phi = deg2rad(0);
%         theta2 = real(acos(((x_ellipse(1)-X)^2 + (y_ellipse(1)-Y)^2 - robot_params(5)^2 - robot_params(6)^2)./(2*robot_params(5)*robot_params(6))));
%         theta1 = atan2(y_ellipse(1)-Y,x_ellipse(1)-X) - atan2(robot_params(6).*sin(theta2),(robot_params(5) + robot_params(6).*cos(theta2))) - phi;
%         q_ellipse(:,1)=[X;Y;phi;theta1;theta2];
%         for i=1:traversals_ellipse*360
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i))-robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J14 = -robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i))-robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J15 = -robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i))+robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J24 = robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i))+robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J25 = robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_ellipse(3,i));
%             J42 = cos(q_ellipse(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45];
%             % Compute Jacobian Pseudoinverse
%             J_inv(:,:,i) = pinv(J(:,:,i));
%             % Artificial Potential
%             neg_grad_V = [0; 0; -1.32*q_ellipse(3,i); - 2*q_ellipse(4,i); -0.5*q_ellipse(5,i)];
%             % Compute Joint Velocities
%             q_dot_ellipse(:,i) = (J_inv(:,:,i) * X_dot(:,i)) + ((eye(5)-(J_inv(:,:,i)*J(:,:,i)))*neg_grad_V);
%             % Compute Joint Angles using Euler Forward
%             q_ellipse(:,i+1) = q_ellipse(:,i)+(q_dot_ellipse(:,i).*(cycle_time_ellipse/360));
%             % Compute Wheel Velocities
%             phi_l_dot = (2*(q_dot_ellipse(1,i)*cos(q_ellipse(3,i)) + q_dot_ellipse(2,i)*sin(q_ellipse(3,i))) - robot_params(2)*q_dot_ellipse(3,i))/(2*robot_params(3));
%             phi_r_dot = (2*(q_dot_ellipse(1,i)*cos(q_ellipse(3,i)) + q_dot_ellipse(2,i)*sin(q_ellipse(3,i))) + robot_params(2)*q_dot_ellipse(3,i))/(2*robot_params(3));
%             % Compute Wheel Angles using Euler Forward
%             phi_l = phi_l + phi_l_dot*(cycle_time_ellipse/360);
%             phi_r = phi_r + phi_r_dot*(cycle_time_ellipse/360);
%             % Get Pose of Mobile Base
%             robot_x = q_ellipse(1,i); robot_y = q_ellipse(2,i); robot_z = 0.2850;
%             [robot_qx,robot_qy,robot_qz,robot_qw] = EA2Q(0,-pi/2,q_ellipse(3,i)-pi/2);
%             % Command Robot Joints
%             [~] = vrep.simxSetObjectPosition(clientID, WMR, -1, [robot_x,robot_y,robot_z], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetObjectQuaternion(clientID, WMR, -1, [robot_qx,robot_qy,robot_qz,robot_qw], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_L, phi_l, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_R, -phi_r, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_1, q_ellipse(4,i), vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_2, q_ellipse(5,i), vrep.simx_opmode_streaming);
%             %figure(4)
%             %PlotRobot(q_ellipse(:,i),robot_params)
%             pause(cycle_time_ellipse/360);
%         end
%         % Stop Simulation and MATLAB Script
%         disp("Simulation Completed!")
%         [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
%         break
%     end
% else
%     disp('Error Connecting to CoppeliaSim!');
% end

% %% Redundancy Resolution Using Artificial Potential Method - Square
% 
% % CoppeliaSim Remote API Connection
% vrep = remApi('remoteApi');
% vrep.simxFinish(-1);
% clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
% if clientID > -1
%     disp('Connected to CoppeliaSim!');
%     % Get Object Handles from Simulator
%     [~, WMR] = vrep.simxGetObjectHandle(clientID, 'Mobile_Base', vrep.simx_opmode_blocking);
%     [~, W_L] = vrep.simxGetObjectHandle(clientID, 'Left_Wheel', vrep.simx_opmode_blocking);
%     [~, W_R] = vrep.simxGetObjectHandle(clientID, 'Right_Wheel', vrep.simx_opmode_blocking);
%     [~, J_1] = vrep.simxGetObjectHandle(clientID, 'Proximal_Link_Joint', vrep.simx_opmode_blocking);
%     [~, J_2] = vrep.simxGetObjectHandle(clientID, 'Distal_Link_Joint', vrep.simx_opmode_blocking);
%     while true
%         % Follow the rectangular trajectory
%         X_dot = [X_dot_rectangle; ones(1,length(X_dot_rectangle))*alpha_dot_rectangle; zeros(1,length(X_dot_rectangle))];
%         J = [];
%         J_inv = [];
%         q_dot_rectangle = [];
%         q_rectangle = [];
%         X_E_rectangle = [];
%         Y_E_rectangle = [];
%         phi_l = 0; phi_r = 0;
%         % Initial configuration
%         X = x_rectangle(1)-robot_params(5);
%         Y = y_rectangle(1)-robot_params(6);
%         phi = deg2rad(0);
%         theta2 = real(acos(((x_rectangle(1)-X)^2 + (y_rectangle(1)-Y)^2 - robot_params(5)^2 - robot_params(6)^2)./(2*robot_params(5)*robot_params(6))));
%         theta1 = atan2(y_rectangle(1)-Y,x_rectangle(1)-X) - atan2(robot_params(6).*sin(theta2),(robot_params(5) + robot_params(6).*cos(theta2))) - phi;
%         q_rectangle(:,1)=[X;Y;phi;theta1;theta2];
%         for i=1:traversals_rectangle*360
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i))-robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J14 = -robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i))-robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J15 = -robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i))+robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J24 = robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i))+robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J25 = robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_rectangle(3,i));
%             J42 = cos(q_rectangle(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45];
%             % Compute Jacobian Pseudoinverse
%             J_inv(:,:,i) = pinv(J(:,:,i));
%             % Artificial Potential
%             neg_grad_V = [0; 0; -1.32*q_rectangle(3,i); - 2*q_rectangle(4,i); -0.5*q_rectangle(5,i)];
%             % Compute Joint Velocities
%             q_dot_rectangle(:,i) = (J_inv(:,:,i) * X_dot(:,i)) + ((eye(5)-(J_inv(:,:,i)*J(:,:,i)))*neg_grad_V);
%             % Compute Joint Angles using Euler Forward
%             q_rectangle(:,i+1) = q_rectangle(:,i)+(q_dot_rectangle(:,i).*(cycle_time_rectangle/360));
%             % Compute Wheel Velocities
%             phi_l_dot = (2*(q_dot_rectangle(1,i)*cos(q_rectangle(3,i)) + q_dot_rectangle(2,i)*sin(q_rectangle(3,i))) - robot_params(2)*q_dot_rectangle(3,i))/(2*robot_params(3));
%             phi_r_dot = (2*(q_dot_rectangle(1,i)*cos(q_rectangle(3,i)) + q_dot_rectangle(2,i)*sin(q_rectangle(3,i))) + robot_params(2)*q_dot_rectangle(3,i))/(2*robot_params(3));
%             % Compute Wheel Angles using Euler Forward
%             phi_l = phi_l + phi_l_dot*(cycle_time_ellipse/360);
%             phi_r = phi_r + phi_r_dot*(cycle_time_ellipse/360);
%             % Get Pose of Mobile Base
%             robot_x = q_rectangle(1,i); robot_y = q_rectangle(2,i); robot_z = 0.2850;
%             [robot_qx,robot_qy,robot_qz,robot_qw] = EA2Q(0,-pi/2,q_rectangle(3,i)-pi/2);
%             % Command Robot Joints
%             [~] = vrep.simxSetObjectPosition(clientID, WMR, -1, [robot_x,robot_y,robot_z], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetObjectQuaternion(clientID, WMR, -1, [robot_qx,robot_qy,robot_qz,robot_qw], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_L, phi_l, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_R, -phi_r, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_1, q_rectangle(4,i), vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_2, q_rectangle(5,i), vrep.simx_opmode_streaming);
%             %figure(4)
%             %PlotRobot(q_rectangle(:,i),robot_params)
%             pause(cycle_time_rectangle/360);
%         end
%         % Stop Simulation and MATLAB Script
%         disp("Simulation Completed!")
%         [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
%         break
%     end
% else
%     disp('Error Connecting to CoppeliaSim!');
% end

% %% Configuration-Space Resolved-Rate Motion Control - Ellipse
% 
% % CoppeliaSim Remote API Connection
% vrep = remApi('remoteApi');
% vrep.simxFinish(-1);
% clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
% if clientID > -1
%     disp('Connected to CoppeliaSim!');
%     % Get Object Handles from Simulator
%     [~, WMR] = vrep.simxGetObjectHandle(clientID, 'Mobile_Base', vrep.simx_opmode_blocking);
%     [~, W_L] = vrep.simxGetObjectHandle(clientID, 'Left_Wheel', vrep.simx_opmode_blocking);
%     [~, W_R] = vrep.simxGetObjectHandle(clientID, 'Right_Wheel', vrep.simx_opmode_blocking);
%     [~, J_1] = vrep.simxGetObjectHandle(clientID, 'Proximal_Link_Joint', vrep.simx_opmode_blocking);
%     [~, J_2] = vrep.simxGetObjectHandle(clientID, 'Distal_Link_Joint', vrep.simx_opmode_blocking);
%     while true
%         % Desired configuration-space variables for elliptical trajectory
%         X_dot = [X_dot_ellipse; ones(1,length(X_dot_ellipse))*alpha_dot_ellipse; zeros(1,length(X_dot_ellipse)); zeros(1,length(X_dot_ellipse))];
%         J = [];
%         J_inv = [];
%         q_dot_ellipse = [];
%         q_d_ellipse = [];
%         X_d_ellipse = [];
%         Y_d_ellipse = [];
%         % Initial configuration
%         X = x_ellipse(1)-robot_params(5);
%         Y = y_ellipse(1)-robot_params(6);
%         phi = deg2rad(0);
%         theta2 = real(acos(((x_ellipse(1)-X)^2 + (y_ellipse(1)-Y)^2 - robot_params(5)^2 - robot_params(6)^2)./(2*robot_params(5)*robot_params(6))));
%         theta1 = atan2(y_ellipse(1)-Y,x_ellipse(1)-X) - atan2(robot_params(6).*sin(theta2),(robot_params(5) + robot_params(6).*cos(theta2))) - phi;
%         q_d_ellipse(:,1)=[X;Y;phi;theta1;theta2];
%         for i=1:traversals_ellipse*360
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_d_ellipse(3,i)+q_d_ellipse(4,i))-robot_params(6)*sin(q_d_ellipse(3,i)+q_d_ellipse(4,i)+q_d_ellipse(5,i));
%             J14 = -robot_params(5)*sin(q_d_ellipse(3,i)+q_d_ellipse(4,i))-robot_params(6)*sin(q_d_ellipse(3,i)+q_d_ellipse(4,i)+q_d_ellipse(5,i));
%             J15 = -robot_params(6)*sin(q_d_ellipse(3,i)+q_d_ellipse(4,i)+q_d_ellipse(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_d_ellipse(3,i)+q_d_ellipse(4,i))+robot_params(6)*cos(q_d_ellipse(3,i)+q_d_ellipse(4,i)+q_d_ellipse(5,i));
%             J24 = robot_params(5)*cos(q_d_ellipse(3,i)+q_d_ellipse(4,i))+robot_params(6)*cos(q_d_ellipse(3,i)+q_d_ellipse(4,i)+q_d_ellipse(5,i));
%             J25 = robot_params(6)*cos(q_d_ellipse(3,i)+q_d_ellipse(4,i)+q_d_ellipse(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_d_ellipse(3,i));
%             J42 = cos(q_d_ellipse(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J51 = 0;
%             J52 = 0;
%             J53 = 0;
%             J54 = 1;
%             J55 = 1;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45;
%                         J51 J52 J53 J54 J55];
%             % Compute Jacobian Pseudoinverse
%             J_inv(:,:,i) = inv(J(:,:,i));
%             % Compute Joint Velocities
%             q_dot_ellipse(:,i) = J_inv(:,:,i) * X_dot(:,i);
%             % Compute Joint Angles using Euler Forward
%             q_d_ellipse(:,i+1) = q_d_ellipse(:,i)+(q_dot_ellipse(:,i).*(cycle_time_ellipse/360));
%             % Store end-effector positions using FK
%             X_d_ellipse(i) = q_d_ellipse(1,i) + robot_params(5)*cos(q_d_ellipse(3,i)+q_d_ellipse(4,i)) + robot_params(6)*cos(q_d_ellipse(3,i)+q_d_ellipse(4,i)+q_d_ellipse(5,i));
%             Y_d_ellipse(i) = q_d_ellipse(2,i) + robot_params(5)*sin(q_d_ellipse(3,i)+q_d_ellipse(4,i)) + robot_params(6)*sin(q_d_ellipse(3,i)+q_d_ellipse(4,i)+q_d_ellipse(5,i));
%         end
%         % Follow the elliptical trajectory
%         q_d_ellipse = q_d_ellipse(:,1:end-1); % Remove last (unwanted) state update
%         X_dot = [X_dot_ellipse; ones(1,length(X_dot_ellipse))*alpha_dot_ellipse; zeros(1,length(X_dot_ellipse)); zeros(1,length(X_dot_ellipse))];
%         J = [];
%         q_dot_ellipse = [];
%         X_E_ellipse = [];
%         Y_E_ellipse = [];
%         phi_l = 0; phi_r = 0;
%         tau = 3; % Error dynamics time constant
%         q_disturb_ellipse = 2; % Disturbance in initial conditions
%         q_ellipse(:,1) = q_d_ellipse(:,1)*q_disturb_ellipse; % Initial configuration (multiplied by a disturbance)
%         for i = 1:length(q_d_ellipse)
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i))-robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J14 = -robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i))-robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J15 = -robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i))+robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J24 = robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i))+robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J25 = robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_ellipse(3,i));
%             J42 = cos(q_ellipse(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J51 = 0;
%             J52 = 0;
%             J53 = 0;
%             J54 = 1;
%             J55 = 1;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45;
%                         J51 J52 J53 J54 J55];
%             % Configuration-space closed-loop control
%             q_dot_ellipse(:,i) = inv(J(:,:,i))*X_dot(:,i)  + diag(ones(1,5)*1/tau)*(q_d_ellipse(:,i)-q_ellipse(:,i));
%             % Compute configuration variables using Euler Forward
%             q_ellipse(:,i+1) = q_ellipse(:,i) + q_dot_ellipse(:,i)*(cycle_time_ellipse/360);
%             % Compute Wheel Velocities
%             phi_l_dot = (2*(q_dot_ellipse(1,i)*cos(q_ellipse(3,i)) + q_dot_ellipse(2,i)*sin(q_ellipse(3,i))) - robot_params(2)*q_dot_ellipse(3,i))/(2*robot_params(3));
%             phi_r_dot = (2*(q_dot_ellipse(1,i)*cos(q_ellipse(3,i)) + q_dot_ellipse(2,i)*sin(q_ellipse(3,i))) + robot_params(2)*q_dot_ellipse(3,i))/(2*robot_params(3));
%             % Compute Wheel Angles using Euler Forward
%             phi_l = phi_l + phi_l_dot*(cycle_time_ellipse/360);
%             phi_r = phi_r + phi_r_dot*(cycle_time_ellipse/360);
%             % Get Pose of Mobile Base
%             robot_x = q_ellipse(1,i); robot_y = q_ellipse(2,i); robot_z = 0.2850;
%             [robot_qx,robot_qy,robot_qz,robot_qw] = EA2Q(0,-pi/2,q_ellipse(3,i)-pi/2);
%             % Command Robot Joints
%             [~] = vrep.simxSetObjectPosition(clientID, WMR, -1, [robot_x,robot_y,robot_z], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetObjectQuaternion(clientID, WMR, -1, [robot_qx,robot_qy,robot_qz,robot_qw], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_L, phi_l, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_R, -phi_r, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_1, q_ellipse(4,i), vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_2, q_ellipse(5,i), vrep.simx_opmode_streaming);
%             %figure(4)
%             %PlotRobot(q_ellipse(:,i),robot_params)
%             pause(cycle_time_ellipse/360);
%         end
%         % Stop Simulation and MATLAB Script
%         disp("Simulation Completed!")
%         [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
%         break
%     end
% else
%     disp('Error Connecting to CoppeliaSim!');
% end

% %% Configuration-Space Resolved-Rate Motion Control - Square
% 
% % CoppeliaSim Remote API Connection
% vrep = remApi('remoteApi');
% vrep.simxFinish(-1);
% clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
% if clientID > -1
%     disp('Connected to CoppeliaSim!');
%     % Get Object Handles from Simulator
%     [~, WMR] = vrep.simxGetObjectHandle(clientID, 'Mobile_Base', vrep.simx_opmode_blocking);
%     [~, W_L] = vrep.simxGetObjectHandle(clientID, 'Left_Wheel', vrep.simx_opmode_blocking);
%     [~, W_R] = vrep.simxGetObjectHandle(clientID, 'Right_Wheel', vrep.simx_opmode_blocking);
%     [~, J_1] = vrep.simxGetObjectHandle(clientID, 'Proximal_Link_Joint', vrep.simx_opmode_blocking);
%     [~, J_2] = vrep.simxGetObjectHandle(clientID, 'Distal_Link_Joint', vrep.simx_opmode_blocking);
%     while true
%         % Desired configuration-space variables for rectangular trajectory
%         X_dot = [X_dot_rectangle; ones(1,length(X_dot_rectangle))*alpha_dot_rectangle; zeros(1,length(X_dot_rectangle)); zeros(1,length(X_dot_rectangle))];
%         J = [];
%         J_inv = [];
%         q_dot_rectangle = [];
%         q_d_rectangle = [];
%         X_d_rectangle = [];
%         Y_d_rectangle = [];
%         % Initial configuration
%         X = x_rectangle(1)-robot_params(5);
%         Y = y_rectangle(1)-robot_params(6);
%         phi = deg2rad(0);
%         theta2 = real(acos(((x_rectangle(1)-X)^2 + (y_rectangle(1)-Y)^2 - robot_params(5)^2 - robot_params(6)^2)./(2*robot_params(5)*robot_params(6))));
%         theta1 = atan2(y_rectangle(1)-Y,x_rectangle(1)-X) - atan2(robot_params(6).*sin(theta2),(robot_params(5) + robot_params(6).*cos(theta2))) - phi;
%         q_d_rectangle(:,1)=[X;Y;phi;theta1;theta2];
%         for i=1:traversals_rectangle*360
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_d_rectangle(3,i)+q_d_rectangle(4,i))-robot_params(6)*sin(q_d_rectangle(3,i)+q_d_rectangle(4,i)+q_d_rectangle(5,i));
%             J14 = -robot_params(5)*sin(q_d_rectangle(3,i)+q_d_rectangle(4,i))-robot_params(6)*sin(q_d_rectangle(3,i)+q_d_rectangle(4,i)+q_d_rectangle(5,i));
%             J15 = -robot_params(6)*sin(q_d_rectangle(3,i)+q_d_rectangle(4,i)+q_d_rectangle(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_d_rectangle(3,i)+q_d_rectangle(4,i))+robot_params(6)*cos(q_d_rectangle(3,i)+q_d_rectangle(4,i)+q_d_rectangle(5,i));
%             J24 = robot_params(5)*cos(q_d_rectangle(3,i)+q_d_rectangle(4,i))+robot_params(6)*cos(q_d_rectangle(3,i)+q_d_rectangle(4,i)+q_d_rectangle(5,i));
%             J25 = robot_params(6)*cos(q_d_rectangle(3,i)+q_d_rectangle(4,i)+q_d_rectangle(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_d_rectangle(3,i));
%             J42 = cos(q_d_rectangle(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J51 = 0;
%             J52 = 0;
%             J53 = 0;
%             J54 = 1;
%             J55 = 1;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45;
%                         J51 J52 J53 J54 J55];
%             % Compute Jacobian Pseudoinverse
%             J_inv(:,:,i) = inv(J(:,:,i));
%             % Compute Joint Velocities
%             q_dot_rectangle(:,i) = J_inv(:,:,i) * X_dot(:,i);
%             % Compute Joint Angles using Euler Forward
%             q_d_rectangle(:,i+1) = q_d_rectangle(:,i)+(q_dot_rectangle(:,i).*(cycle_time_rectangle/360));
%             % Store end-effector positions using FK
%             X_d_rectangle(i) = q_d_rectangle(1,i) + robot_params(5)*cos(q_d_rectangle(3,i)+q_d_rectangle(4,i)) + robot_params(6)*cos(q_d_rectangle(3,i)+q_d_rectangle(4,i)+q_d_rectangle(5,i));
%             Y_d_rectangle(i) = q_d_rectangle(2,i) + robot_params(5)*sin(q_d_rectangle(3,i)+q_d_rectangle(4,i)) + robot_params(6)*sin(q_d_rectangle(3,i)+q_d_rectangle(4,i)+q_d_rectangle(5,i));
%         end
%         % Follow the rectangular trajectory
%         q_d_rectangle = q_d_rectangle(:,1:end-1); % Remove last (unwanted) state update
%         X_dot = [X_dot_rectangle; ones(1,length(X_dot_rectangle))*alpha_dot_rectangle; zeros(1,length(X_dot_rectangle)); zeros(1,length(X_dot_rectangle))];
%         J = [];
%         q_dot_rectangle = [];
%         X_E_rectangle = [];
%         Y_E_rectangle = [];
%         phi_l = 0; phi_r = 0;
%         tau = 3; % Error dynamics time constant
%         q_disturb_rectangle = 2; % Disturbance in initial conditions
%         q_rectangle(:,1) = q_d_rectangle(:,1)*q_disturb_rectangle; % Initial configuration (multiplied by a disturbance)
%         for i = 1:length(q_d_rectangle)
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i))-robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J14 = -robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i))-robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J15 = -robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i))+robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J24 = robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i))+robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J25 = robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_rectangle(3,i));
%             J42 = cos(q_rectangle(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J51 = 0;
%             J52 = 0;
%             J53 = 0;
%             J54 = 1;
%             J55 = 1;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45;
%                         J51 J52 J53 J54 J55];
%             % Configuration-space closed-loop control
%             q_dot_rectangle(:,i) = inv(J(:,:,i))*X_dot(:,i)  + diag(ones(1,5)*1/tau)*(q_d_rectangle(:,i)-q_rectangle(:,i));
%             % Compute configuration variables using Euler Forward
%             q_rectangle(:,i+1) = q_rectangle(:,i) + q_dot_rectangle(:,i)*(cycle_time_rectangle/360);
%             % Compute Wheel Velocities
%             phi_l_dot = (2*(q_dot_rectangle(1,i)*cos(q_rectangle(3,i)) + q_dot_rectangle(2,i)*sin(q_rectangle(3,i))) - robot_params(2)*q_dot_rectangle(3,i))/(2*robot_params(3));
%             phi_r_dot = (2*(q_dot_rectangle(1,i)*cos(q_rectangle(3,i)) + q_dot_rectangle(2,i)*sin(q_rectangle(3,i))) + robot_params(2)*q_dot_rectangle(3,i))/(2*robot_params(3));
%             % Compute Wheel Angles using Euler Forward
%             phi_l = phi_l + phi_l_dot*(cycle_time_ellipse/360);
%             phi_r = phi_r + phi_r_dot*(cycle_time_ellipse/360);
%             % Get Pose of Mobile Base
%             robot_x = q_rectangle(1,i); robot_y = q_rectangle(2,i); robot_z = 0.2850;
%             [robot_qx,robot_qy,robot_qz,robot_qw] = EA2Q(0,-pi/2,q_rectangle(3,i)-pi/2);
%             % Command Robot Joints
%             [~] = vrep.simxSetObjectPosition(clientID, WMR, -1, [robot_x,robot_y,robot_z], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetObjectQuaternion(clientID, WMR, -1, [robot_qx,robot_qy,robot_qz,robot_qw], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_L, phi_l, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_R, -phi_r, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_1, q_rectangle(4,i), vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_2, q_rectangle(5,i), vrep.simx_opmode_streaming);
%             %figure(4)
%             %PlotRobot(q_rectangle(:,i),robot_params)
%             pause(cycle_time_rectangle/360);
%         end
%         % Stop Simulation and MATLAB Script
%         disp("Simulation Completed!")
%         [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
%         break
%     end
% else
%     disp('Error Connecting to CoppeliaSim!');
% end

% %% Task-Space Resolved-Rate Motion Control - Ellipse
% 
% % CoppeliaSim Remote API Connection
% vrep = remApi('remoteApi');
% vrep.simxFinish(-1);
% clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
% if clientID > -1
%     disp('Connected to CoppeliaSim!');
%     % Get Object Handles from Simulator
%     [~, WMR] = vrep.simxGetObjectHandle(clientID, 'Mobile_Base', vrep.simx_opmode_blocking);
%     [~, W_L] = vrep.simxGetObjectHandle(clientID, 'Left_Wheel', vrep.simx_opmode_blocking);
%     [~, W_R] = vrep.simxGetObjectHandle(clientID, 'Right_Wheel', vrep.simx_opmode_blocking);
%     [~, J_1] = vrep.simxGetObjectHandle(clientID, 'Proximal_Link_Joint', vrep.simx_opmode_blocking);
%     [~, J_2] = vrep.simxGetObjectHandle(clientID, 'Distal_Link_Joint', vrep.simx_opmode_blocking);
%     while true
%         % Follow the elliptical trajectory
%         X_dot = [X_dot_ellipse; ones(1,length(X_dot_ellipse))*alpha_dot_ellipse; zeros(1,length(X_dot_ellipse)); zeros(1,length(X_dot_ellipse))];
%         J = [];
%         q_ellipse = [];
%         q_dot_ellipse = [];
%         X_E_ellipse = [];
%         Y_E_ellipse = [];
%         phi_l = 0; phi_r = 0;
%         K1 = 5; % Error dynamics negative pole 1
%         K2 = 10; % Error dynamics negative pole 2
%         % Initial configuration
%         X = x_ellipse(1)-robot_params(5);
%         Y = y_ellipse(1)-robot_params(6);
%         phi = deg2rad(0);
%         theta2 = real(acos(((x_ellipse(1)-X)^2 + (y_ellipse(1)-Y)^2 - robot_params(5)^2 - robot_params(6)^2)./(2*robot_params(5)*robot_params(6))));
%         theta1 = atan2(y_ellipse(1)-Y,x_ellipse(1)-X) - atan2(robot_params(6).*sin(theta2),(robot_params(5) + robot_params(6).*cos(theta2))) - phi;
%         q_disturb_ellipse = 2; % Disturbance in initial conditions
%         q_ellipse(:,1)=[X;Y;phi;theta1;theta2]*q_disturb_ellipse; % Initial configuration (multiplied by a disturbance)
%         for i = 1:length(x_ellipse)
%             % Compute Jacobian
%             J11 = 1;
%             J12 = 0;
%             J13 = -robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i))-robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J14 = -robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i))-robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J15 = -robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J21 = 0;
%             J22 = 1;
%             J23 = robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i))+robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J24 = robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i))+robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J25 = robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             J31 = 0;
%             J32 = 0;
%             J33 = 1;
%             J34 = 1;
%             J35 = 1;
%             J41 = -sin(q_ellipse(3,i));
%             J42 = cos(q_ellipse(3,i));
%             J43 = 0;
%             J44 = 0;
%             J45 = 0;
%             J51 = 0;
%             J52 = 0;
%             J53 = 0;
%             J54 = 1;
%             J55 = 1;
%             J(:,:,i) = [J11 J12 J13 J14 J15;
%                         J21 J22 J23 J24 J25;
%                         J31 J32 J33 J34 J35;
%                         J41 J42 J43 J44 J45;
%                         J51 J52 J53 J54 J55];
%             % Actual end-effector positions using FK
%             X_E_ellipse(i) = q_ellipse(1,i) + robot_params(5)*cos(q_ellipse(3,i)+q_ellipse(4,i)) + robot_params(6)*cos(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             Y_E_ellipse(i) = q_ellipse(2,i) + robot_params(5)*sin(q_ellipse(3,i)+q_ellipse(4,i)) + robot_params(6)*sin(q_ellipse(3,i)+q_ellipse(4,i)+q_ellipse(5,i));
%             % Task-space closed-loop control
%             q_dot_ellipse(:,i) = inv(J(:,:,i))*(X_dot(:,i) + diag([K1,K2,0,0,0])*[x_ellipse(i)-X_E_ellipse(i);y_ellipse(i)-Y_E_ellipse(i);0;0;0]);
%             % Compute configuration variables using Euler Forward
%             q_ellipse(:,i+1) = q_ellipse(:,i) + q_dot_ellipse(:,i)*(cycle_time_ellipse/360);
%             % Compute Wheel Velocities
%             phi_l_dot = (2*(q_dot_ellipse(1,i)*cos(q_ellipse(3,i)) + q_dot_ellipse(2,i)*sin(q_ellipse(3,i))) - robot_params(2)*q_dot_ellipse(3,i))/(2*robot_params(3));
%             phi_r_dot = (2*(q_dot_ellipse(1,i)*cos(q_ellipse(3,i)) + q_dot_ellipse(2,i)*sin(q_ellipse(3,i))) + robot_params(2)*q_dot_ellipse(3,i))/(2*robot_params(3));
%             % Compute Wheel Angles using Euler Forward
%             phi_l = phi_l + phi_l_dot*(cycle_time_ellipse/360);
%             phi_r = phi_r + phi_r_dot*(cycle_time_ellipse/360);
%             % Get Pose of Mobile Base
%             robot_x = q_ellipse(1,i); robot_y = q_ellipse(2,i); robot_z = 0.2850;
%             [robot_qx,robot_qy,robot_qz,robot_qw] = EA2Q(0,-pi/2,q_ellipse(3,i)-pi/2);
%             % Command Robot Joints
%             [~] = vrep.simxSetObjectPosition(clientID, WMR, -1, [robot_x,robot_y,robot_z], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetObjectQuaternion(clientID, WMR, -1, [robot_qx,robot_qy,robot_qz,robot_qw], vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_L, phi_l, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, W_R, -phi_r, vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_1, q_ellipse(4,i), vrep.simx_opmode_streaming);
%             [~] = vrep.simxSetJointPosition(clientID, J_2, q_ellipse(5,i), vrep.simx_opmode_streaming);
%             %figure(4)
%             %PlotRobot(q_ellipse(:,i),robot_params)
%             pause(cycle_time_ellipse/360);
%         end
%         % Stop Simulation and MATLAB Script
%         disp("Simulation Completed!")
%         [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
%         break
%     end
% else
%     disp('Error Connecting to CoppeliaSim!');
% end

%% Task-Space Resolved-Rate Motion Control - Square

% CoppeliaSim Remote API Connection
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
if clientID > -1
    disp('Connected to CoppeliaSim!');
    % Get Object Handles from Simulator
    [~, WMR] = vrep.simxGetObjectHandle(clientID, 'Mobile_Base', vrep.simx_opmode_blocking);
    [~, W_L] = vrep.simxGetObjectHandle(clientID, 'Left_Wheel', vrep.simx_opmode_blocking);
    [~, W_R] = vrep.simxGetObjectHandle(clientID, 'Right_Wheel', vrep.simx_opmode_blocking);
    [~, J_1] = vrep.simxGetObjectHandle(clientID, 'Proximal_Link_Joint', vrep.simx_opmode_blocking);
    [~, J_2] = vrep.simxGetObjectHandle(clientID, 'Distal_Link_Joint', vrep.simx_opmode_blocking);
    while true
        % Follow the rectangular trajectory
        X_dot = [X_dot_rectangle; ones(1,length(X_dot_rectangle))*alpha_dot_rectangle; zeros(1,length(X_dot_rectangle)); zeros(1,length(X_dot_rectangle))];
        J = [];
        q_rectangle = [];
        q_dot_rectangle = [];
        X_E_rectangle = [];
        Y_E_rectangle = [];
        phi_l = 0; phi_r = 0;
        K1 = 5; % Error dynamics negative pole 1
        K2 = 10; % Error dynamics negative pole 2
        % Initial configuration
        X = x_rectangle(1)-robot_params(5);
        Y = y_rectangle(1)-robot_params(6);
        phi = deg2rad(0);
        theta2 = real(acos(((x_rectangle(1)-X)^2 + (y_rectangle(1)-Y)^2 - robot_params(5)^2 - robot_params(6)^2)./(2*robot_params(5)*robot_params(6))));
        theta1 = atan2(y_rectangle(1)-Y,x_rectangle(1)-X) - atan2(robot_params(6).*sin(theta2),(robot_params(5) + robot_params(6).*cos(theta2))) - phi;
        q_disturb_rectangle = 2; % Disturbance in initial conditions
        q_rectangle(:,1)=[X;Y;phi;theta1;theta2]*q_disturb_rectangle; % Initial configuration (multiplied by a disturbance)
        for i = 1:length(x_rectangle)
            % Compute Jacobian
            J11 = 1;
            J12 = 0;
            J13 = -robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i))-robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
            J14 = -robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i))-robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
            J15 = -robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
            J21 = 0;
            J22 = 1;
            J23 = robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i))+robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
            J24 = robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i))+robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
            J25 = robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
            J31 = 0;
            J32 = 0;
            J33 = 1;
            J34 = 1;
            J35 = 1;
            J41 = -sin(q_rectangle(3,i));
            J42 = cos(q_rectangle(3,i));
            J43 = 0;
            J44 = 0;
            J45 = 0;
            J51 = 0;
            J52 = 0;
            J53 = 0;
            J54 = 1;
            J55 = 1;
            J(:,:,i) = [J11 J12 J13 J14 J15;
                        J21 J22 J23 J24 J25;
                        J31 J32 J33 J34 J35;
                        J41 J42 J43 J44 J45;
                        J51 J52 J53 J54 J55];
            % Actual end-effector positions using FK
            X_E_rectangle(i) = q_rectangle(1,i) + robot_params(5)*cos(q_rectangle(3,i)+q_rectangle(4,i)) + robot_params(6)*cos(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
            Y_E_rectangle(i) = q_rectangle(2,i) + robot_params(5)*sin(q_rectangle(3,i)+q_rectangle(4,i)) + robot_params(6)*sin(q_rectangle(3,i)+q_rectangle(4,i)+q_rectangle(5,i));
            % Task-space closed-loop control
            q_dot_rectangle(:,i) = inv(J(:,:,i))*(X_dot(:,i) + diag([K1,K2,0,0,0])*[x_rectangle(i)-X_E_rectangle(i);y_rectangle(i)-Y_E_rectangle(i);0;0;0]);
            % Compute configuration variables using Euler Forward
            q_rectangle(:,i+1) = q_rectangle(:,i) + q_dot_rectangle(:,i)*(cycle_time_rectangle/360);
            % Compute Wheel Velocities
            phi_l_dot = (2*(q_dot_rectangle(1,i)*cos(q_rectangle(3,i)) + q_dot_rectangle(2,i)*sin(q_rectangle(3,i))) - robot_params(2)*q_dot_rectangle(3,i))/(2*robot_params(3));
            phi_r_dot = (2*(q_dot_rectangle(1,i)*cos(q_rectangle(3,i)) + q_dot_rectangle(2,i)*sin(q_rectangle(3,i))) + robot_params(2)*q_dot_rectangle(3,i))/(2*robot_params(3));
            % Compute Wheel Angles using Euler Forward
            phi_l = phi_l + phi_l_dot*(cycle_time_ellipse/360);
            phi_r = phi_r + phi_r_dot*(cycle_time_ellipse/360);
            % Get Pose of Mobile Base
            robot_x = q_rectangle(1,i); robot_y = q_rectangle(2,i); robot_z = 0.2850;
            [robot_qx,robot_qy,robot_qz,robot_qw] = EA2Q(0,-pi/2,q_rectangle(3,i)-pi/2);
            % Command Robot Joints
            [~] = vrep.simxSetObjectPosition(clientID, WMR, -1, [robot_x,robot_y,robot_z], vrep.simx_opmode_streaming);
            [~] = vrep.simxSetObjectQuaternion(clientID, WMR, -1, [robot_qx,robot_qy,robot_qz,robot_qw], vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, W_L, phi_l, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, W_R, -phi_r, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, J_1, q_rectangle(4,i), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, J_2, q_rectangle(5,i), vrep.simx_opmode_streaming);
            %figure(4)
            %PlotRobot(q_rectangle(:,i),robot_params)
            pause(cycle_time_rectangle/360);
        end
        % Stop Simulation and MATLAB Script
        disp("Simulation Completed!")
        [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
        break
    end
else
    disp('Error Connecting to CoppeliaSim!');
end

%% Helper Functions

function [qx, qy, qz, qw] = EA2Q(yaw, pitch, roll)
    %{
    Converts Euler angle representation to its equivalent quaternion representation.
    Input:
        yaw = Yaw angle (around Z-axis)
        pitch = Pitch angle (around Y-axis)
        roll = Roll angle (around X-axis)
    Output:
        qx = The vector term of `q` (corresponds to X-component of rotation axis)
        qy = The vector term of `q` (corresponds to Y-component of rotation axis)
        qz = The vector term of `q` (corresponds to Z-component of rotation axis)
        qw = The scalar value of `q` (corresponds to angle of rotation)
    %}
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
end