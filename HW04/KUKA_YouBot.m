%% Clear Workspace

clear;
clc;

%% Kuka youBot Parameters

r = (100/2)*1e-3; % Wheel radius (m)
w = 0.5*300.46*1e-3; % Half-track width (m)
l = 0.5*471*1e-3; % Half-wheelbase (m)

%% Simulation Results

T = [];
robotPosX = [];
robotPosY = [];
robotYaw = [];
manipulatorPosX = [];
manipulatorPosY = [];
manipulatorTheta1 = [];
manipulatorTheta2 = [];
manipulatorTheta3 = [];

%% CoppeliaSim Remote API Connection

vrep = remApi('remoteApi');
vrep.simxFinish(-1);

clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if clientID > -1
    disp('Connected to CoppeliaSim!');
    % Get Data From Simulator
    [~, joint0] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint0', vrep.simx_opmode_blocking);
    [~, joint1] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint1', vrep.simx_opmode_blocking);
    [~, joint2] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint2', vrep.simx_opmode_blocking);
    [~, joint3] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint3', vrep.simx_opmode_blocking);
    [~, joint4] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint4', vrep.simx_opmode_blocking);
    [~, gripper1] = vrep.simxGetObjectHandle(clientID, 'youBotGripperJoint1', vrep.simx_opmode_blocking);
    [~, gripper2] = vrep.simxGetObjectHandle(clientID, 'youBotGripperJoint1', vrep.simx_opmode_blocking);
    [~, wheel1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_blocking);
    [~, wheel2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_blocking);
    [~, wheel3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_blocking);
    [~, wheel4] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_blocking);
    [~, robot] = vrep.simxGetObjectHandle(clientID, 'youBot', vrep.simx_opmode_blocking);
    while true
        % Get Robot Position and Orientation
        [~,robotPosition]=vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_streaming);
        [~,robotOrientation]=vrep.simxGetObjectOrientation(clientID,robot,-1,vrep.simx_opmode_streaming);
        [~,robotLVelocity,robotAVelocity]=vrep.simxGetObjectVelocity(clientID,robot,vrep.simx_opmode_streaming);
        [Time]=vrep.simxGetLastCmdTime(clientID);
        robotX = robotPosition(1);
        robotY = robotPosition(2);
        robotT = robotOrientation(2);
        T(end+1) = Time/1000;
        robotPosX(end+1) = robotX;
        robotPosY(end+1) = robotY;
        robotYaw(end+1) = robotT;
        figure(1)
        sgtitle('Mobile Robot Task-Space Position Analysis')
        subplot(1,3,1), plot(T,robotPosX)
        xlabel("Time (s)")
        ylabel("X Position (m)")
        subplot(1,3,2), plot(T,robotPosY)
        xlabel("Time (s)")
        ylabel("Y Position (m)")
        subplot(1,3,3), plot(robotPosX,robotPosY)
        ylabel("X Position (m)")
        ylabel("Y Position (m)")
        % Command Robot Joints
        if robotY > -5
            v_x = 0.1; % m/s
            v_y = 0.0; % m/s
            W_z = 0.0; % rad/s
            u = 1/r*[1,1,l+w;1,-1,-l-w;1,-1,l+w;1,1,-l-w]*[v_x;v_y;W_z];
            % Manipulator Joints
            [~] = vrep.simxSetJointPosition(clientID, joint0, deg2rad(0), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint1, deg2rad(0), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint2, deg2rad(0), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint3, deg2rad(0), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint4, deg2rad(0), vrep.simx_opmode_streaming);
            % Wheel Joints
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel1, u(1), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel2, u(2), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel3, u(3), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel4, u(4), vrep.simx_opmode_streaming);
            last_yaw = robotT;
            last_time = Time/1000;
        elseif robotY<=-5
            v_x = 0.0; % m/s
            v_y = 0.0; % m/s
            W_z = 0.5; % rad/s
            u = 1/r*[1,1,l+w;1,-1,-l-w;1,-1,l+w;1,1,-l-w]*[v_x;v_y;W_z];
            % Manipulator Joints
            [~] = vrep.simxSetJointPosition(clientID, joint0, deg2rad(0), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint1, deg2rad(0), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint2, deg2rad(0), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint3, deg2rad(0), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint4, deg2rad(0), vrep.simx_opmode_streaming);
            % Wheel Joints
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel1, u(1), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel2, u(2), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel3, u(3), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel4, u(4), vrep.simx_opmode_streaming);
            if (Time/1000)-last_time >= 10 && abs(robotT-last_yaw) <= 0.174533
                v_x = 0.0; % m/s
                v_y = 0.0; % m/s
                W_z = 0.0; % rad/s
                u = 1/r*[1,1,l+w;1,-1,-l-w;1,-1,l+w;1,1,-l-w]*[v_x;v_y;W_z];
                % Manipulator Joints
                [~] = vrep.simxSetJointPosition(clientID, joint0, deg2rad(0), vrep.simx_opmode_streaming);
                [~] = vrep.simxSetJointPosition(clientID, joint1, deg2rad(0), vrep.simx_opmode_streaming);
                [~] = vrep.simxSetJointPosition(clientID, joint2, deg2rad(0), vrep.simx_opmode_streaming);
                [~] = vrep.simxSetJointPosition(clientID, joint3, deg2rad(0), vrep.simx_opmode_streaming);
                [~] = vrep.simxSetJointPosition(clientID, joint4, deg2rad(0), vrep.simx_opmode_streaming);
                % Wheel Joints
                [~] = vrep.simxSetJointTargetVelocity(clientID, wheel1, u(1), vrep.simx_opmode_streaming);
                [~] = vrep.simxSetJointTargetVelocity(clientID, wheel2, u(2), vrep.simx_opmode_streaming);
                [~] = vrep.simxSetJointTargetVelocity(clientID, wheel3, u(3), vrep.simx_opmode_streaming);
                [~] = vrep.simxSetJointTargetVelocity(clientID, wheel4, u(4), vrep.simx_opmode_streaming);
                % Stop Simulation and MATLAB Script
                disp("Simulation Completed!")
                [~] = simxStopSimulation(vrep,clientID,vrep.simx_opmode_oneshot);
                break
            end
        end
    end
else
    disp('Error Connecting to CoppeliaSim!');
end