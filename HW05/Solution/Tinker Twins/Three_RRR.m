%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 5: Control of Parallel-Manipulator Robot %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti, Olamide Akinyele

fprintf('\n=========================================================================\n');
fprintf('CONTROL OF PARALLEL-MANIPULATOR ROBOT USING MATLAB-COPPELIASIM\n');
fprintf('=========================================================================\n');

%% Robot Parameters
R = 300*1e-3; % Radial distance of base joints from origin (m)
L1 = 300*1e-3; % Proximal link length (m)
L2 = 250*1e-3; % Distal link length (m)
r = 100*1e-3; % Distance from centroid `P` to vertex of platform (m)
Ax = R*cosd(210); % X-coordinate of base joint A
Ay = R*sind(210); % Y-coordinate of base joint A
Bx = R*cosd(330); % X-coordinate of base joint B
By = R*sind(330); % Y-coordinate of base joint B
Cx = R*cosd(90); % X-coordinate of base joint C
Cy = R*sind(90); % Y-coordinate of base joint C
robot_params = [L1,L2,r,Ax,Ay,Bx,By,Cx,Cy]; % Array of robot parameters
% NOTE: SW2URDF exports URDF "as is" in SolidWorks assembly and allocates
% "zero" joint angles in that specific configuration (NOT w.r.t. global
% X-axis). So add/subtract offsets manually to compensate for this.
J11_Offset = deg2rad(94.67902477);
J12_Offset = deg2rad(205.09165403);
J13_Offset = deg2rad(0.80943257);

%% Simulation Results
% Time array
Time = [];
% Arm 1 joint parameters (position, velocity, acceleration)
Theta11 = [];
Theta21 = [];
Theta31 = [];
dTheta11 = [];
dTheta21 = [];
dTheta31 = [];
ddTheta11 = [];
ddTheta21 = [];
ddTheta31 = [];
% Arm 2 joint parameters (position, velocity, acceleration)
Theta12 = [];
Theta22 = [];
Theta32 = [];
dTheta12 = [];
dTheta22 = [];
dTheta32 = [];
ddTheta12 = [];
ddTheta22 = [];
ddTheta32 = [];
% Arm 3 joint parameters (position, velocity, acceleration)
Theta13 = [];
Theta23 = [];
Theta33 = [];
dTheta13 = [];
dTheta23 = [];
dTheta33 = [];
ddTheta13 = [];
ddTheta23 = [];
ddTheta33 = [];

%% CoppeliaSim Remote API Connection

% [Q1,Q2,Q3] = IK_3RRR(0.050,0.065,deg2rad(-60),robot_params);
% Q1 = rad2deg(Q1)
% Q2 = rad2deg(Q2)
% Q3 = rad2deg(Q3)

vrep = remApi('remoteApi');
vrep.simxFinish(-1);

clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if clientID > -1
    disp('Connected to CoppeliaSim!');
    % Get Data From Simulator
    [~, J11] = vrep.simxGetObjectHandle(clientID, 'Proximal_Joint_1', vrep.simx_opmode_blocking);
    [~, J12] = vrep.simxGetObjectHandle(clientID, 'Proximal_Joint_2', vrep.simx_opmode_blocking);
    [~, J13] = vrep.simxGetObjectHandle(clientID, 'Proximal_Joint_3', vrep.simx_opmode_blocking);
    [~, J21] = vrep.simxGetObjectHandle(clientID, 'Distal_Joint_1', vrep.simx_opmode_blocking);
    [~, J22] = vrep.simxGetObjectHandle(clientID, 'Distal_Joint_2', vrep.simx_opmode_blocking);
    [~, J23] = vrep.simxGetObjectHandle(clientID, 'Distal_Joint_3', vrep.simx_opmode_blocking);
    [~, J31] = vrep.simxGetObjectHandle(clientID, 'Platform_Joint_1', vrep.simx_opmode_blocking);
    [~, J32] = vrep.simxGetObjectHandle(clientID, 'Platform_Joint_2', vrep.simx_opmode_blocking);
    [~, J33] = vrep.simxGetObjectHandle(clientID, 'Platform_Joint_3', vrep.simx_opmode_blocking);
    [~, pen] = vrep.simxGetObjectHandle(clientID, 'Pen_respondable', vrep.simx_opmode_blocking);
    %while true
        % Get Pen's Pose & Velocity
        [~,penPosition]=vrep.simxGetObjectPosition(clientID,pen,-1,vrep.simx_opmode_streaming);
        [~,penOrientation]=vrep.simxGetObjectOrientation(clientID,pen,-1,vrep.simx_opmode_streaming);
        [~,penLVelocity,penAVelocity]=vrep.simxGetObjectVelocity(clientID,pen,vrep.simx_opmode_streaming);
        % Get Parallel Manipulator Robot Joint Angles
        [~,q11]=vrep.simxGetJointPosition(clientID,J11,vrep.simx_opmode_streaming);
        [~,q12]=vrep.simxGetJointPosition(clientID,J12,vrep.simx_opmode_streaming);
        [~,q13]=vrep.simxGetJointPosition(clientID,J13,vrep.simx_opmode_streaming);
        [~,q21]=vrep.simxGetJointPosition(clientID,J21,vrep.simx_opmode_streaming);
        [~,q22]=vrep.simxGetJointPosition(clientID,J22,vrep.simx_opmode_streaming);
        [~,q23]=vrep.simxGetJointPosition(clientID,J23,vrep.simx_opmode_streaming);
        [~,q31]=vrep.simxGetJointPosition(clientID,J31,vrep.simx_opmode_streaming);
        [~,q32]=vrep.simxGetJointPosition(clientID,J32,vrep.simx_opmode_streaming);
        [~,q33]=vrep.simxGetJointPosition(clientID,J33,vrep.simx_opmode_streaming);
        % Get Time
        [T]=vrep.simxGetLastCmdTime(clientID);
        % Move End Effector from (50 mm, 65 mm, -60 deg) to (0 mm, 65 mm, -60 deg)
        Ctraj = ctraj(transl(0.05,0.065,0.00)*trotz(-60), transl(0.00,0.065,0.00)*trotz(-60), 20);
        Jtraj = [];
        for i=1:length(Ctraj)
            X = Ctraj(1,4,i);
            Y = Ctraj(2,4,i);
            Q = tr2rpy(Ctraj(:,:,i));
            [Q1,Q2,Q3] = IK_3RRR(X,Y,Q(3),robot_params);
            Jtraj(i,:) = [Q1,Q2,Q3];
        end
        for i=1:length(Jtraj)
            % Record Manipulator Robot Data
%             [~,manipulatorPosition]=vrep.simxGetObjectPosition(clientID,endeffector,-1,vrep.simx_opmode_streaming);
%             [~,manipulatorOrientation]=vrep.simxGetObjectOrientation(clientID,endeffector,-1,vrep.simx_opmode_streaming);
%             [~,J0]=vrep.simxGetJointPosition(clientID,joint0,vrep.simx_opmode_streaming);
%             [~,J1]=vrep.simxGetJointPosition(clientID,joint1,vrep.simx_opmode_streaming);
%             [~,J2]=vrep.simxGetJointPosition(clientID,joint2,vrep.simx_opmode_streaming);
%             [~,J3]=vrep.simxGetJointPosition(clientID,joint3,vrep.simx_opmode_streaming);
%             [~,J4]=vrep.simxGetJointPosition(clientID,joint4,vrep.simx_opmode_streaming);
%             [Time]=vrep.simxGetLastCmdTime(clientID);
%             manipulatorPosX(end+1) = manipulatorPosition(1);
%             manipulatorPosY(end+1) = manipulatorPosition(3);
%             manipulatorTheta0(end+1) = J0;
%             manipulatorTheta1(end+1) = J1;
%             manipulatorTheta2(end+1) = J2;
%             manipulatorTheta3(end+1) = J3;
%             manipulatorTheta4(end+1) = J4;
%             T_manipulator(end+1) = Time/1000;
            % Manipulator Joints
            [~] = vrep.simxSetJointTargetPosition(clientID, J11, Jtraj(i,1)-J11_Offset, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetPosition(clientID, J12, Jtraj(i,2)-J12_Offset, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetPosition(clientID, J13, Jtraj(i,3)-J13_Offset, vrep.simx_opmode_streaming);
            pause(0.1);
        end
        % Move End Effector from (0 mm, 65 mm, -60 deg) to (0 mm, 50 mm, -60 deg)
        Ctraj = ctraj(transl(0.00,0.065,0.00)*trotz(-60), transl(0.00,0.05,0.00)*trotz(-60), 10);
        Jtraj = [];
        for i=1:length(Ctraj)
            X = Ctraj(1,4,i);
            Y = Ctraj(2,4,i);
            Q = tr2rpy(Ctraj(:,:,i));
            [Q1,Q2,Q3] = IK_3RRR(X,Y,Q(3),robot_params);
            Jtraj(i,:) = [Q1,Q2,Q3];
        end
        for i=1:length(Jtraj)
            % Record Manipulator Robot Data
%             [~,manipulatorPosition]=vrep.simxGetObjectPosition(clientID,endeffector,-1,vrep.simx_opmode_streaming);
%             [~,manipulatorOrientation]=vrep.simxGetObjectOrientation(clientID,endeffector,-1,vrep.simx_opmode_streaming);
%             [~,J0]=vrep.simxGetJointPosition(clientID,joint0,vrep.simx_opmode_streaming);
%             [~,J1]=vrep.simxGetJointPosition(clientID,joint1,vrep.simx_opmode_streaming);
%             [~,J2]=vrep.simxGetJointPosition(clientID,joint2,vrep.simx_opmode_streaming);
%             [~,J3]=vrep.simxGetJointPosition(clientID,joint3,vrep.simx_opmode_streaming);
%             [~,J4]=vrep.simxGetJointPosition(clientID,joint4,vrep.simx_opmode_streaming);
%             [Time]=vrep.simxGetLastCmdTime(clientID);
%             manipulatorPosX(end+1) = manipulatorPosition(1);
%             manipulatorPosY(end+1) = manipulatorPosition(3);
%             manipulatorTheta0(end+1) = J0;
%             manipulatorTheta1(end+1) = J1;
%             manipulatorTheta2(end+1) = J2;
%             manipulatorTheta3(end+1) = J3;
%             manipulatorTheta4(end+1) = J4;
%             T_manipulator(end+1) = Time/1000;
            % Manipulator Joints
            [~] = vrep.simxSetJointTargetPosition(clientID, J11, Jtraj(i,1)-J11_Offset, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetPosition(clientID, J12, Jtraj(i,2)-J12_Offset, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetPosition(clientID, J13, Jtraj(i,3)-J13_Offset, vrep.simx_opmode_streaming);
            pause(0.1);
        end
        % Move Manipulator in a Circular Trajectory
        Ttraj = 90:450; % Rotate from 0+90 deg to 360+90 deg 
        Radius = 50*1e-3; % Radius of circular trajectory (m)
        Center = [0,0]; % Center of circular trajectory (m)
        X_array = [];
        Y_array = [];
        for i=1:length(Ttraj)
            X = Center(1)+Radius*cosd(Ttraj(i));
            Y = Center(2)+Radius*sind(Ttraj(i));
            Q = [0,0,deg2rad(-60)];
            [Q1,Q2,Q3] = IK_3RRR(X,Y,Q(3),robot_params);
            Jtraj(i,:) = [Q1,Q2,Q3];
            X_array(end+1) = X;
            Y_array(end+1) = Y;
        end
        plot(X_array,Y_array,'o')
        for i=1:length(Jtraj)
            % Record Manipulator Robot Data
%             [~,manipulatorPosition]=vrep.simxGetObjectPosition(clientID,endeffector,-1,vrep.simx_opmode_streaming);
%             [~,manipulatorOrientation]=vrep.simxGetObjectOrientation(clientID,endeffector,-1,vrep.simx_opmode_streaming);
%             [~,J0]=vrep.simxGetJointPosition(clientID,joint0,vrep.simx_opmode_streaming);
%             [~,J1]=vrep.simxGetJointPosition(clientID,joint1,vrep.simx_opmode_streaming);
%             [~,J2]=vrep.simxGetJointPosition(clientID,joint2,vrep.simx_opmode_streaming);
%             [~,J3]=vrep.simxGetJointPosition(clientID,joint3,vrep.simx_opmode_streaming);
%             [~,J4]=vrep.simxGetJointPosition(clientID,joint4,vrep.simx_opmode_streaming);
%             [Time]=vrep.simxGetLastCmdTime(clientID);
%             manipulatorPosX(end+1) = manipulatorPosition(1);
%             manipulatorPosY(end+1) = manipulatorPosition(3);
%             manipulatorTheta0(end+1) = J0;
%             manipulatorTheta1(end+1) = J1;
%             manipulatorTheta2(end+1) = J2;
%             manipulatorTheta3(end+1) = J3;
%             manipulatorTheta4(end+1) = J4;
%             T_manipulator(end+1) = Time/1000;
            % Manipulator Joints
            [~] = vrep.simxSetJointTargetPosition(clientID, J11, Jtraj(i,1)-J11_Offset, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetPosition(clientID, J12, Jtraj(i,2)-J12_Offset, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetPosition(clientID, J13, Jtraj(i,3)-J13_Offset, vrep.simx_opmode_streaming);
            pause(0.05);
        end
%         % Plot Robot Data
%         figure(1)
%         sgtitle('Manipulator Robot Position Analysis')
%         subplot(1,4,1), plot(T_manipulator,manipulatorPosX)
%         xlabel("Simulation Time (s)")
%         ylabel("X Position (m)")
%         subplot(1,4,2), plot(T_manipulator,manipulatorPosY)
%         xlabel("Simulation Time (s)")
%         ylabel("Y Position (m)")
%         subplot(1,4,3), plot(manipulatorPosX,manipulatorPosY)
%         xlabel("X Position (m)")
%         ylabel("Y Position (m)")
%         subplot(1,4,4)
%         hold on
%         plot(T_manipulator,manipulatorTheta0)
%         plot(T_manipulator,manipulatorTheta1)
%         plot(T_manipulator,manipulatorTheta2)
%         plot(T_manipulator,manipulatorTheta3)
%         plot(T_manipulator,manipulatorTheta4)
%         hold off
%         xlabel("Simulation Time (s)")
%         ylabel("Joint Angles (rad)")
%         legend("J0","J1","J2","J3","J4",'Location','EO')
%         % Stop Simulation and MATLAB Script
%         disp("Simulation Completed!")
%         [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
%         break
    %end
else
    disp('Error Connecting to CoppeliaSim!');
end

%% Inverse Kinematics of 3RRR Parallel Maipulator Robot
function [q1,q2,q3] = IK_3RRR(Xp,Yp,Qp,robot_params)
    %{
    Input:
        Xp = End-effector (platform) position X-coordinate
        Yp = End-effector (platform) position Y-coordinate
        Qp = End-effector (platform) orientation w.r.t. X-axis
        robot_params = [L1,L2,r,Ax,Ay,Bx,By,Cx,Cy]
            L1 = Proximal link length
            L2 = Distal link length
            r = Distance from centroid (P) to vertex of platform
            Ax = X-coordinate of proximal joint 1
            Ay = Y-coordinate of proximal joint 1
            Bx = X-coordinate of proximal joint 2
            By = Y-coordinate of proximal joint 2
            Cx = X-coordinate of proximal joint 3
            Cy = Y-coordinate of proximal joint 3
      Output:
        q1 = Angle of proximal joint 1
        q2 = Angle of proximal joint 2
        q3 = Angle of proximal joint 3
    %}
    % Load robot parameters
    L1 = robot_params(1);
    L2 = robot_params(2);
    r = robot_params(3);
    Ax = robot_params(4);
    Ay = robot_params(5);
    Bx = robot_params(6);
    By = robot_params(7);
    Cx = robot_params(8);
    Cy = robot_params(9);
    % Positions of platform joints
    A2x = Xp+(r*cos(Qp-(5*pi/6)));
    A2y = Yp+(r*sin(Qp-(5*pi/6)));
    B2x = Xp+(r*cos(Qp-(pi/6)));
    B2y = Yp+(r*sin(Qp-(pi/6)));
    C2x = Xp+(r*cos(Qp-(9*pi/6)));
    C2y = Yp+(r*sin(Qp-(9*pi/6)));
    % Distance between proximal and platform joints
    dAA2 = sqrt((A2x-Ax)^2+(A2y-Ay)^2);
    dBB2 = sqrt((B2x-Bx)^2+(B2y-By)^2);
    dCC2 = sqrt((C2x-Cx)^2+(C2y-Cy)^2);
    % Angle of axis passing through proximal and platform joints w.r.t. global horizontal
    phiAA2 = atan2(A2y-Ay,A2x-Ax);
    phiBB2 = atan2(B2y-By,B2x-Bx);
    phiCC2 = atan2(C2y-Cy,C2x-Cx);
    % Angle between axis passing through proximal and platform joints, and proximal links
    phiAA1 = acos((dAA2^2+L1^2-L2^2)/(2*L1*dAA2));
    phiBB1 = acos((dBB2^2+L1^2-L2^2)/(2*L1*dBB2));
    phiCC1 = acos((dCC2^2+L1^2-L2^2)/(2*L1*dCC2));
    % Proximal joint angles
    q1 = phiAA2 + phiAA1;
    q2 = phiBB2 + phiBB1;
    q3 = phiCC2 + phiCC1;
end