%% Clear Workspace

clear;
clc;

%% CoppeliaSim Remote API Connection

vrep = remApi('remoteApi');
vrep.simxFinish(-1);

clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if clientID > -1
    disp('Connected to CoppeliaSim!');
    % Get Data From Simulator
    [r, joint0] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint0', vrep.simx_opmode_blocking);
    [r, joint1] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint1', vrep.simx_opmode_blocking);
    [r, joint2] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint2', vrep.simx_opmode_blocking);
    [r, joint3] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint3', vrep.simx_opmode_blocking);
    [r, joint4] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint4', vrep.simx_opmode_blocking);
    [r, gripper1] = vrep.simxGetObjectHandle(clientID, 'youBotGripperJoint1', vrep.simx_opmode_blocking);
    [r, gripper2] = vrep.simxGetObjectHandle(clientID, 'youBotGripperJoint1', vrep.simx_opmode_blocking);
    [r, wheel1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_blocking);
    [r, wheel2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_blocking);
    [r, wheel3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_blocking);
    [r, wheel4] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_blocking);
    % Command Robot Joints
    i = 0;
    while i < 1000
        % Manipulator Joints
        [r] = vrep.simxSetJointPosition(clientID, joint0, deg2rad(30), vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointPosition(clientID, joint1, deg2rad(30), vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointPosition(clientID, joint2, deg2rad(30), vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointPosition(clientID, joint3, deg2rad(30), vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointPosition(clientID, joint4, deg2rad(30), vrep.simx_opmode_streaming);
        % Wheel Joints
        [r] = vrep.simxSetJointTargetVelocity(clientID, wheel1, 0.5, vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointTargetVelocity(clientID, wheel2, 0.5, vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointTargetVelocity(clientID, wheel3, 0.5, vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointTargetVelocity(clientID, wheel4, 0.5, vrep.simx_opmode_streaming);
        i = i+1;
    end

else
    disp('ERROR Connecting to CoppeliaSim!');
end