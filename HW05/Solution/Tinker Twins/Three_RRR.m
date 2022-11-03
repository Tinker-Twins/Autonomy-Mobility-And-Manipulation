%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 3: Interpolation, Inverse Kinematics %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti, Olamide Akinyele

fprintf('\n=========================================================================\n');
fprintf('CONTROL OF MOBILE-MANIPULATOR ROBOT USING MATLAB-COPPELIASIM\n');
fprintf('=========================================================================\n');

%% CoppeliaSim Remote API Connection

vrep = remApi('remoteApi');
vrep.simxFinish(-1);

clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if clientID > -1
    disp('Connected to CoppeliaSim!');
    % Get Data From Simulator
    [~, a11] = vrep.simxGetObjectHandle(clientID, 'Proximal_Joint_1', vrep.simx_opmode_blocking);
    [~, a12] = vrep.simxGetObjectHandle(clientID, 'Proximal_Joint_2', vrep.simx_opmode_blocking);
    [~, a13] = vrep.simxGetObjectHandle(clientID, 'Proximal_Joint_3', vrep.simx_opmode_blocking);
    [~, a21] = vrep.simxGetObjectHandle(clientID, 'Distal_Joint_1', vrep.simx_opmode_blocking);
    [~, a22] = vrep.simxGetObjectHandle(clientID, 'Distal_Joint_2', vrep.simx_opmode_blocking);
    [~, a23] = vrep.simxGetObjectHandle(clientID, 'Distal_Joint_3', vrep.simx_opmode_blocking);
    [~, a31] = vrep.simxGetObjectHandle(clientID, 'Platform_Joint_1', vrep.simx_opmode_blocking);
    [~, a32] = vrep.simxGetObjectHandle(clientID, 'Platform_Joint_2', vrep.simx_opmode_blocking);
    [~, a33] = vrep.simxGetObjectHandle(clientID, 'Platform_Joint_3', vrep.simx_opmode_blocking);
    [~, pen] = vrep.simxGetObjectHandle(clientID, 'Pen_respondable', vrep.simx_opmode_blocking);
    while true
        % Get Pen's Pose & Velocity
        [~,penPosition]=vrep.simxGetObjectPosition(clientID,pen,-1,vrep.simx_opmode_streaming);
        [~,penOrientation]=vrep.simxGetObjectOrientation(clientID,pen,-1,vrep.simx_opmode_streaming);
        [~,penLVelocity,penAVelocity]=vrep.simxGetObjectVelocity(clientID,pen,vrep.simx_opmode_streaming);
        % Get Parallel Manipulator Robot Joint Angles
        [~,J11]=vrep.simxGetJointPosition(clientID,a11,vrep.simx_opmode_streaming);
        [~,J12]=vrep.simxGetJointPosition(clientID,a12,vrep.simx_opmode_streaming);
        [~,J13]=vrep.simxGetJointPosition(clientID,a13,vrep.simx_opmode_streaming);
        [~,J21]=vrep.simxGetJointPosition(clientID,a21,vrep.simx_opmode_streaming);
        [~,J22]=vrep.simxGetJointPosition(clientID,a22,vrep.simx_opmode_streaming);
        [~,J23]=vrep.simxGetJointPosition(clientID,a23,vrep.simx_opmode_streaming);
        [~,J31]=vrep.simxGetJointPosition(clientID,a31,vrep.simx_opmode_streaming);
        [~,J32]=vrep.simxGetJointPosition(clientID,a32,vrep.simx_opmode_streaming);
        [~,J33]=vrep.simxGetJointPosition(clientID,a33,vrep.simx_opmode_streaming);
        % Get Time
        [Time]=vrep.simxGetLastCmdTime(clientID);
        disp(penPosition)
        [~]=vrep.simxSetJointTargetVelocity(clientID, a11, 0.1, vrep.simx_opmode_streaming);
        % Record Mobile Robot Data
%         mobileRobotPosX(end+1) = mobilePosition(1);
%         mobileRobotPosY(end+1) = mobilePosition(2);
%         mobileRobotYaw(end+1) = mobileOrientation(2);
%         T_mobile(end+1) = Time/1000;
    end
else
    disp('Error Connecting to CoppeliaSim!');
end