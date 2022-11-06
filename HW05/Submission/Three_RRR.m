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
dTheta11 = [];
ddTheta11 = [];
% Arm 2 joint parameters (position, velocity, acceleration)
Theta12 = [];
dTheta12 = [];
ddTheta12 = [];
% Arm 3 joint parameters (position, velocity, acceleration)
Theta13 = [];
dTheta13 = [];
ddTheta13 = [];

%% CoppeliaSim Remote API Connection

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
    % Declare simulation time-step
    dt = 0.1;
    % Initialize previous joint angles for t=0
    q11_prev=0;
    q12_prev=0;
    q13_prev=0;
    % Initialize previous joint velocities for t=0
    dq11_prev=0;
    dq12_prev=0;
    dq13_prev=0;
    while true
        % Get and Record Time
        [T]=vrep.simxGetLastCmdTime(clientID);
        Time(end+1) = T/1000;
        % Get Joint Angles
        [~,q11]=vrep.simxGetJointPosition(clientID,J11,vrep.simx_opmode_streaming);
        [~,q12]=vrep.simxGetJointPosition(clientID,J12,vrep.simx_opmode_streaming);
        [~,q13]=vrep.simxGetJointPosition(clientID,J13,vrep.simx_opmode_streaming);
        % Record Joint Angles
        Theta11(end+1) = q11+J11_Offset;
        Theta12(end+1) = q12+J12_Offset;
        Theta13(end+1) = q13+J13_Offset;
        % Calculate Joint Velocities
        dq11 = (q11-q11_prev)/dt;
        dq12 = (q12-q12_prev)/dt;
        dq13 = (q13-q13_prev)/dt;
        % Record Joint Velocities
        dTheta11(end+1) = dq11;
        dTheta12(end+1) = dq12;
        dTheta13(end+1) = dq13;
        % Update Previous Joint Velocities
        q11_prev = q11;
        q12_prev = q12;
        q13_prev = q13;
        % Calculate Joint Accelerations
        ddq11 = (dq11-dq11_prev)/dt;
        ddq12 = (dq12-dq12_prev)/dt;
        ddq13 = (dq13-dq13_prev)/dt;
        % Record Joint Accelerations
        ddTheta11(end+1) = ddq11;
        ddTheta12(end+1) = ddq12;
        ddTheta13(end+1) = ddq13;
        % Update Previous Joint Velocities
        dq11_prev = dq11;
        dq12_prev = dq12;
        dq13_prev = dq13;
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
            % Get and Record Time
            [T]=vrep.simxGetLastCmdTime(clientID);
            Time(end+1) = T/1000;
            % Get Joint Angles
            [~,q11]=vrep.simxGetJointPosition(clientID,J11,vrep.simx_opmode_streaming);
            [~,q12]=vrep.simxGetJointPosition(clientID,J12,vrep.simx_opmode_streaming);
            [~,q13]=vrep.simxGetJointPosition(clientID,J13,vrep.simx_opmode_streaming);
            % Record Joint Angles
            Theta11(end+1) = q11+J11_Offset;
            Theta12(end+1) = q12+J12_Offset;
            Theta13(end+1) = q13+J13_Offset;
            % Calculate Joint Velocities
            dq11 = (q11-q11_prev)/dt;
            dq12 = (q12-q12_prev)/dt;
            dq13 = (q13-q13_prev)/dt;
            % Record Joint Velocities
            dTheta11(end+1) = dq11;
            dTheta12(end+1) = dq12;
            dTheta13(end+1) = dq13;
            % Update Previous Joint Velocities
            q11_prev = q11;
            q12_prev = q12;
            q13_prev = q13;
            % Calculate Joint Accelerations
            ddq11 = (dq11-dq11_prev)/dt;
            ddq12 = (dq12-dq12_prev)/dt;
            ddq13 = (dq13-dq13_prev)/dt;
            % Record Joint Accelerations
            ddTheta11(end+1) = ddq11;
            ddTheta12(end+1) = ddq12;
            ddTheta13(end+1) = ddq13;
            % Update Previous Joint Velocities
            dq11_prev = dq11;
            dq12_prev = dq12;
            dq13_prev = dq13;
            % Command Manipulator Joints
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
            % Get and Record Time
            [T]=vrep.simxGetLastCmdTime(clientID);
            Time(end+1) = T/1000;
            % Get Joint Angles
            [~,q11]=vrep.simxGetJointPosition(clientID,J11,vrep.simx_opmode_streaming);
            [~,q12]=vrep.simxGetJointPosition(clientID,J12,vrep.simx_opmode_streaming);
            [~,q13]=vrep.simxGetJointPosition(clientID,J13,vrep.simx_opmode_streaming);
            % Record Joint Angles
            Theta11(end+1) = q11+J11_Offset;
            Theta12(end+1) = q12+J12_Offset;
            Theta13(end+1) = q13+J13_Offset;
            % Calculate Joint Velocities
            dq11 = (q11-q11_prev)/dt;
            dq12 = (q12-q12_prev)/dt;
            dq13 = (q13-q13_prev)/dt;
            % Record Joint Velocities
            dTheta11(end+1) = dq11;
            dTheta12(end+1) = dq12;
            dTheta13(end+1) = dq13;
            % Update Previous Joint Velocities
            q11_prev = q11;
            q12_prev = q12;
            q13_prev = q13;
            % Calculate Joint Accelerations
            ddq11 = (dq11-dq11_prev)/dt;
            ddq12 = (dq12-dq12_prev)/dt;
            ddq13 = (dq13-dq13_prev)/dt;
            % Record Joint Accelerations
            ddTheta11(end+1) = ddq11;
            ddTheta12(end+1) = ddq12;
            ddTheta13(end+1) = ddq13;
            % Update Previous Joint Velocities
            dq11_prev = dq11;
            dq12_prev = dq12;
            dq13_prev = dq13;
            % Command Manipulator Joints
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
        end
        for i=1:length(Jtraj)
            % Get and Record Time
            [T]=vrep.simxGetLastCmdTime(clientID);
            Time(end+1) = T/1000;
            % Get Joint Angles
            [~,q11]=vrep.simxGetJointPosition(clientID,J11,vrep.simx_opmode_streaming);
            [~,q12]=vrep.simxGetJointPosition(clientID,J12,vrep.simx_opmode_streaming);
            [~,q13]=vrep.simxGetJointPosition(clientID,J13,vrep.simx_opmode_streaming);
            % Record Joint Angles
            Theta11(end+1) = q11+J11_Offset;
            Theta12(end+1) = q12+J12_Offset;
            Theta13(end+1) = q13+J13_Offset;
            % Calculate Joint Velocities
            dq11 = (q11-q11_prev)/dt;
            dq12 = (q12-q12_prev)/dt;
            dq13 = (q13-q13_prev)/dt;
            % Record Joint Velocities
            dTheta11(end+1) = dq11;
            dTheta12(end+1) = dq12;
            dTheta13(end+1) = dq13;
            % Update Previous Joint Velocities
            q11_prev = q11;
            q12_prev = q12;
            q13_prev = q13;
            % Calculate Joint Accelerations
            ddq11 = (dq11-dq11_prev)/dt;
            ddq12 = (dq12-dq12_prev)/dt;
            ddq13 = (dq13-dq13_prev)/dt;
            % Record Joint Accelerations
            ddTheta11(end+1) = ddq11;
            ddTheta12(end+1) = ddq12;
            ddTheta13(end+1) = ddq13;
            % Update Previous Joint Velocities
            dq11_prev = dq11;
            dq12_prev = dq12;
            dq13_prev = dq13;
            % Command Manipulator Joints
            [~] = vrep.simxSetJointTargetPosition(clientID, J11, Jtraj(i,1)-J11_Offset, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetPosition(clientID, J12, Jtraj(i,2)-J12_Offset, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetPosition(clientID, J13, Jtraj(i,3)-J13_Offset, vrep.simx_opmode_streaming);
            pause(0.05);
        end
        % Preprocess Data
        Time = Time(5:end);
        Theta11 = Theta11(5:end);
        Theta12 = Theta12(5:end);
        Theta13 = Theta13(5:end);
        dTheta11 = dTheta11(5:end);
        dTheta12 = dTheta12(5:end);
        dTheta13 = dTheta13(5:end);
        ddTheta11 = ddTheta11(5:end);
        ddTheta12 = ddTheta12(5:end);
        ddTheta13 = ddTheta13(5:end);
        % Plot Data
        figure(1)
        sgtitle('3RRR Parallel Manipulator Robot Analysis')
        subplot(1,3,1)
        hold on
        plot(Time,Theta11)
        plot(Time,Theta12)
        plot(Time,Theta13)
        hold off
        xlabel("Simulation Time (s)")
        ylabel("Joint Angles (rad)")
        legend("J11","J12","J13",'Location','NE')
        subplot(1,3,2)
        hold on
        plot(Time,dTheta11)
        plot(Time,dTheta12)
        plot(Time,dTheta13)
        hold off
        xlabel("Simulation Time (s)")
        ylabel("Joint Velocities (rad/s)")
        legend("J11","J12","J13",'Location','NE')
        subplot(1,3,3)
        hold on
        plot(Time,ddTheta11)
        plot(Time,ddTheta12)
        plot(Time,ddTheta13)
        hold off
        xlabel("Simulation Time (s)")
        ylabel("Joint Accelerations (rad/s^2)")
        legend("J11","J12","J13",'Location','NE')
        % Stop Simulation and MATLAB Script
        disp("Simulation Completed!")
        [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
        break
    end
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