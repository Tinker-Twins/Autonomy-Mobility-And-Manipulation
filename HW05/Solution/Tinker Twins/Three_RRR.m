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

%% Simulation Results
% Time array
Time = [];
% Arm 1 joint parameters (position, velocity, acceleration)
Theta11 = [];
Theta12 = [];
Theta13 = [];
dTheta11 = [];
dTheta12 = [];
dTheta13 = [];
ddTheta11 = [];
ddTheta12 = [];
ddTheta13 = [];
% Arm 2 joint parameters (position, velocity, acceleration)
Theta21 = [];
Theta22 = [];
Theta23 = [];
dTheta21 = [];
dTheta22 = [];
dTheta23 = [];
ddTheta21 = [];
ddTheta22 = [];
ddTheta23 = [];
% Arm 3 joint parameters (position, velocity, acceleration)
Theta31 = [];
Theta32 = [];
Theta33 = [];
dTheta31 = [];
dTheta32 = [];
dTheta33 = [];
ddTheta31 = [];
ddTheta32 = [];
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
    while true
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
        disp(penPosition)
        [~]=vrep.simxSetJointTargetVelocity(clientID, J11, 0.1, vrep.simx_opmode_streaming);
        % Record Robot Data
%         mobileRobotPosX(end+1) = mobilePosition(1);
%         mobileRobotPosY(end+1) = mobilePosition(2);
%         mobileRobotYaw(end+1) = mobileOrientation(2);
%         Time(end+1) = T/1000;
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
            Ax = X-coordinate of base joint A
            Ay = Y-coordinate of base joint A
            Bx = X-coordinate of base joint B
            By = Y-coordinate of base joint B
            Cx = X-coordinate of base joint C
            Cy = Y-coordinate of base joint C
      Output:
        q1 = Angle of joint A
        q2 = Angle of joint B
        q3 = Angle of joint C
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
    % Positions of final revolute joints
    A2x = Xp+(r*cos(Qp+(7*pi/6)));
    A2y = Yp+(r*sin(Qp+(7*pi/6)));
    B2x = Xp+(r*cos(Qp-(pi/6)));
    B2y = Yp+(r*sin(Qp-(pi/6)));
    C2x = Xp+(r*cos(Qp+(7*pi/2)));
    C2y = Yp+(r*sin(Qp+(7*pi/2)));
    % Distance between first and last revolute joints
    dAA2 = sqrt((A2x-Ax)^2+(A2y-Ay)^2);
    dBB2 = sqrt((B2x-Bx)^2+(B2y-By)^2);
    dCC2 = sqrt((C2x-Cx)^2+(C2y-Cy)^2);
    % Angle of axis passing through first and last revolute joints w.r.t. horizontal
    phiAA2 = atan2(A2y-Ay,A2x-Ax);
    phiBB2 = atan2(B2y-By,B2x-Bx);
    phiCC2 = atan2(C2y-Cy,C2x-Cx);
    % Angle between axis passing through first and last revolute joints, and proximal links
    phiAA1 = acos((dAA2^2+L1^2-L2^2)/(2*L1*dAA2));
    phiBB1 = acos((dBB2^2+L1^2-L2^2)/(2*L1*dBB2));
    phiCC1 = acos((dCC2^2+L1^2-L2^2)/(2*L1*dCC2));
    % Joint angles
    q1 = phiAA2 + phiAA1;
    q2 = phiBB2 + phiBB1;
    q3 = phiCC2 + phiCC1;
end