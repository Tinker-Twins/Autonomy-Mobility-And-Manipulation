%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 3: Interpolation, Inverse Kinematics %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti, Olamide Akinyele

fprintf('\n=========================================================================\n');
fprintf('CONTROL OF MOBILE-MANIPULATOR ROBOT USING MATLAB-COPPELIASIM\n');
fprintf('=========================================================================\n');

%% KUKA youBot Parameters

% Mobile Robot
r = (100/2)*1e-3; % Wheel radius (m)
w = 0.5*300.46*1e-3; % Half-track width (m)
l = 0.5*471*1e-3; % Half-wheelbase (m)

% Manipulator Robot (considering planar configuration, i.e. J2-J4)
L(1) = Link([0 0 155.0 0]);
L(2) = Link([0 0 135.0 0]);
L(3) = Link([0 0 217.5 0]);
youBotManipulator = SerialLink(L, 'name', 'Planar KUKA youBot Manipulator')
figure(1);
youBotManipulator.plot([0 0 0]); % Home position
%% Simulation Results

T_mobile = [];
T_manipulator = [];
mobileRobotPosX = [];
mobileRobotPosY = [];
mobileRobotYaw = [];
manipulatorPosX = [];
manipulatorPosY = [];
manipulatorTheta0 = [];
manipulatorTheta1 = [];
manipulatorTheta2 = [];
manipulatorTheta3 = [];
manipulatorTheta4 = [];

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
    [~, endeffector] = vrep.simxGetObjectHandle(clientID, 'youBot_positionTip', vrep.simx_opmode_blocking);
    [~, wheel1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_blocking);
    [~, wheel2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_blocking);
    [~, wheel3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_blocking);
    [~, wheel4] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_blocking);
    [~, robot] = vrep.simxGetObjectHandle(clientID, 'youBot', vrep.simx_opmode_blocking);
    while true
        % Get Mobile Robot Position and Orientation
        [~,mobilePosition]=vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_streaming);
        [~,mobileOrientation]=vrep.simxGetObjectOrientation(clientID,robot,-1,vrep.simx_opmode_streaming);
        %[~,robotLVelocity,robotAVelocity]=vrep.simxGetObjectVelocity(clientID,robot,vrep.simx_opmode_streaming);
        % Get Manipulator Robot End Effector Position and Orientation
        [~,manipulatorPosition]=vrep.simxGetObjectPosition(clientID,endeffector,-1,vrep.simx_opmode_streaming);
        [~,manipulatorOrientation]=vrep.simxGetObjectOrientation(clientID,endeffector,-1,vrep.simx_opmode_streaming);
        % Get Manipulator Robot Joint Angles
        [~,J0]=vrep.simxGetJointPosition(clientID,joint0,vrep.simx_opmode_streaming);
        [~,J1]=vrep.simxGetJointPosition(clientID,joint1,vrep.simx_opmode_streaming);
        [~,J2]=vrep.simxGetJointPosition(clientID,joint2,vrep.simx_opmode_streaming);
        [~,J3]=vrep.simxGetJointPosition(clientID,joint3,vrep.simx_opmode_streaming);
        [~,J4]=vrep.simxGetJointPosition(clientID,joint4,vrep.simx_opmode_streaming);
        % Get Time
        [Time]=vrep.simxGetLastCmdTime(clientID);
        % Command Robot Joints
        if mobilePosition(1) < 5
            % Record Mobile Robot Data
            [~,mobilePosition]=vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_streaming);
            [~,mobileOrientation]=vrep.simxGetObjectOrientation(clientID,robot,-1,vrep.simx_opmode_streaming);
            %[~,robotLVelocity,robotAVelocity]=vrep.simxGetObjectVelocity(clientID,robot,vrep.simx_opmode_streaming);
            [Time]=vrep.simxGetLastCmdTime(clientID);
            mobileRobotPosX(end+1) = mobilePosition(1);
            mobileRobotPosY(end+1) = mobilePosition(2);
            mobileRobotYaw(end+1) = mobileOrientation(2);
            T_mobile(end+1) = Time/1000;
            % Move Mobile Robot Straight 5 m with v = 0.1 m/s
            v_x = -0.1; % m/s
            v_y = 0.0; % m/s
            W_z = 0.0; % rad/s
            u = 1/r*[1,1,l+w;1,-1,-l-w;1,-1,l+w;1,1,-l-w]*[v_x;v_y;W_z];
            % Wheel Joints
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel1, u(1), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel2, u(2), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel3, u(3), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel4, u(4), vrep.simx_opmode_streaming);
            % Manipulator Joints
            [~] = vrep.simxSetJointPosition(clientID, joint0, 0, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint1, 0, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint2, -pi/2, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint3, 0, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint4, 0, vrep.simx_opmode_streaming);
            % Keep track of Yaw and Time to check for 1 rotation later
            last_yaw = mobileOrientation(2);
            last_time = Time/1000;
        elseif mobilePosition(1) >= 5
            % Record Mobile Robot Data
            [~,mobilePosition]=vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_streaming);
            [~,mobileOrientation]=vrep.simxGetObjectOrientation(clientID,robot,-1,vrep.simx_opmode_streaming);
            %[~,robotLVelocity,robotAVelocity]=vrep.simxGetObjectVelocity(clientID,robot,vrep.simx_opmode_streaming);
            [Time]=vrep.simxGetLastCmdTime(clientID);
            mobileRobotPosX(end+1) = mobilePosition(1);
            mobileRobotPosY(end+1) = mobilePosition(2);
            mobileRobotYaw(end+1) = mobileOrientation(2);
            T_mobile(end+1) = Time/1000;
            % Stop and Turn Mobile Robot with W = 0.5 rad/s
            v_x = 0.0; % m/s
            v_y = 0.0; % m/s
            W_z = 0.5; % rad/s
            u = 1/r*[1,1,l+w;1,-1,-l-w;1,-1,l+w;1,1,-l-w]*[v_x;v_y;W_z];
            % Wheel Joints
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel1, u(1), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel2, u(2), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel3, u(3), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointTargetVelocity(clientID, wheel4, u(4), vrep.simx_opmode_streaming);
            % Manipulator Joints
            [~] = vrep.simxSetJointPosition(clientID, joint0, 0, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint1, 0, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint2, -pi/2, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint3, 0, vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(clientID, joint4, 0, vrep.simx_opmode_streaming);
            if (Time/1000)-last_time >= 10 && abs(mobileOrientation(2)-last_yaw) <= 0.0174533
                % Record Mobile Robot Data
                [~,mobilePosition]=vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_streaming);
                [~,mobileOrientation]=vrep.simxGetObjectOrientation(clientID,robot,-1,vrep.simx_opmode_streaming);
                %[~,robotLVelocity,robotAVelocity]=vrep.simxGetObjectVelocity(clientID,robot,vrep.simx_opmode_streaming);
                [Time]=vrep.simxGetLastCmdTime(clientID);
                mobileRobotPosX(end+1) = mobilePosition(1);
                mobileRobotPosY(end+1) = mobilePosition(2);
                mobileRobotYaw(end+1) = mobileOrientation(2);
                T_mobile(end+1) = Time/1000;
                % Stop Mobile Robot
                v_x = 0.0; % m/s
                v_y = 0.0; % m/s
                W_z = 0.0; % rad/s
                u = 1/r*[1,1,l+w;1,-1,-l-w;1,-1,l+w;1,1,-l-w]*[v_x;v_y;W_z];
                % Wheel Joints
                [~] = vrep.simxSetJointTargetVelocity(clientID, wheel1, u(1), vrep.simx_opmode_streaming);
                [~] = vrep.simxSetJointTargetVelocity(clientID, wheel2, u(2), vrep.simx_opmode_streaming);
                [~] = vrep.simxSetJointTargetVelocity(clientID, wheel3, u(3), vrep.simx_opmode_streaming);
                [~] = vrep.simxSetJointTargetVelocity(clientID, wheel4, u(4), vrep.simx_opmode_streaming);
                % Plot Mobile Robot Data
                figure(2)
                sgtitle('Mobile Robot Position Analysis')
                subplot(1,3,1), plot(T_mobile,mobileRobotPosX)
                xlabel("Simulation Time (s)")
                ylabel("X Position (m)")
                subplot(1,3,2), plot(T_mobile,mobileRobotPosY)
                xlabel("Simulation Time (s)")
                ylabel("Y Position (m)")
                subplot(1,3,3), plot(mobileRobotPosX,mobileRobotPosY)
                xlabel("X Position (m)")
                ylabel("Y Position (m)")
                % Move Manipulator to Start Point of Linear Trajectory
                Ji = [pi/2,-pi/2,0];
                Jf = youBotManipulator.ikine(transl(350,0,0),'q0',[pi/2,-pi/2,0],'mask',[1,1,0,0,0,0]);
                Jtraj = jtraj(Ji,Jf,50);
                for i=1:length(Jtraj)
                    % Record Manipulator Robot Data
                    [~,manipulatorPosition]=vrep.simxGetObjectPosition(clientID,endeffector,-1,vrep.simx_opmode_streaming);
                    [~,manipulatorOrientation]=vrep.simxGetObjectOrientation(clientID,endeffector,-1,vrep.simx_opmode_streaming);
                    [~,J0]=vrep.simxGetJointPosition(clientID,joint0,vrep.simx_opmode_streaming);
                    [~,J1]=vrep.simxGetJointPosition(clientID,joint1,vrep.simx_opmode_streaming);
                    [~,J2]=vrep.simxGetJointPosition(clientID,joint2,vrep.simx_opmode_streaming);
                    [~,J3]=vrep.simxGetJointPosition(clientID,joint3,vrep.simx_opmode_streaming);
                    [~,J4]=vrep.simxGetJointPosition(clientID,joint4,vrep.simx_opmode_streaming);
                    [Time]=vrep.simxGetLastCmdTime(clientID);
                    manipulatorPosX(end+1) = manipulatorPosition(1);
                    manipulatorPosY(end+1) = manipulatorPosition(3);
                    manipulatorTheta0(end+1) = J0;
                    manipulatorTheta1(end+1) = J1;
                    manipulatorTheta2(end+1) = J2;
                    manipulatorTheta3(end+1) = J3;
                    manipulatorTheta4(end+1) = J4;
                    T_manipulator(end+1) = Time/1000;
                    % Manipulator Joints
                    [~] = vrep.simxSetJointPosition(clientID, joint0, 0, vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint1, Jtraj(i,1)-pi/2, vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint2, Jtraj(i,2), vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint3, Jtraj(i,3), vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint4, 0, vrep.simx_opmode_streaming);
                    pause(0.1);
                end
                % Move Manipulator in a Linear Trajectory
                Ctraj = ctraj(transl(350,0,0), transl(350,350,0), 100);
                for i=1:length(Ctraj)
                    Jtraj(i,:) = youBotManipulator.ikine(Ctraj(:,:,i),'q0',[pi/2,-pi/2,0],'mask',[1,1,0,0,0,0]);
                end
                for i=1:length(Jtraj)
                    % Record Manipulator Robot Data
                    [~,manipulatorPosition]=vrep.simxGetObjectPosition(clientID,endeffector,-1,vrep.simx_opmode_streaming);
                    [~,manipulatorOrientation]=vrep.simxGetObjectOrientation(clientID,endeffector,-1,vrep.simx_opmode_streaming);
                    [~,J0]=vrep.simxGetJointPosition(clientID,joint0,vrep.simx_opmode_streaming);
                    [~,J1]=vrep.simxGetJointPosition(clientID,joint1,vrep.simx_opmode_streaming);
                    [~,J2]=vrep.simxGetJointPosition(clientID,joint2,vrep.simx_opmode_streaming);
                    [~,J3]=vrep.simxGetJointPosition(clientID,joint3,vrep.simx_opmode_streaming);
                    [~,J4]=vrep.simxGetJointPosition(clientID,joint4,vrep.simx_opmode_streaming);
                    [Time]=vrep.simxGetLastCmdTime(clientID);
                    manipulatorPosX(end+1) = manipulatorPosition(1);
                    manipulatorPosY(end+1) = manipulatorPosition(3);
                    manipulatorTheta0(end+1) = J0;
                    manipulatorTheta1(end+1) = J1;
                    manipulatorTheta2(end+1) = J2;
                    manipulatorTheta3(end+1) = J3;
                    manipulatorTheta4(end+1) = J4;
                    T_manipulator(end+1) = Time/1000;
                    % Manipulator Joints
                    [~] = vrep.simxSetJointPosition(clientID, joint0, 0, vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint1, Jtraj(i,1)-pi/2, vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint2, Jtraj(i,2), vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint3, Jtraj(i,3), vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint4, 0, vrep.simx_opmode_streaming);
                    pause(0.1);
                end
                % Move Manipulator to Start Point of Semicircular Trajectory
                Ji = Jtraj(end,:);
                Jf = [0,0,0];
                Jtraj = jtraj(Ji,Jf,100);
                for i=1:length(Jtraj)
                    % Record Manipulator Robot Data
                    [~,manipulatorPosition]=vrep.simxGetObjectPosition(clientID,endeffector,-1,vrep.simx_opmode_streaming);
                    [~,manipulatorOrientation]=vrep.simxGetObjectOrientation(clientID,endeffector,-1,vrep.simx_opmode_streaming);
                    [~,J0]=vrep.simxGetJointPosition(clientID,joint0,vrep.simx_opmode_streaming);
                    [~,J1]=vrep.simxGetJointPosition(clientID,joint1,vrep.simx_opmode_streaming);
                    [~,J2]=vrep.simxGetJointPosition(clientID,joint2,vrep.simx_opmode_streaming);
                    [~,J3]=vrep.simxGetJointPosition(clientID,joint3,vrep.simx_opmode_streaming);
                    [~,J4]=vrep.simxGetJointPosition(clientID,joint4,vrep.simx_opmode_streaming);
                    [Time]=vrep.simxGetLastCmdTime(clientID);
                    manipulatorPosX(end+1) = manipulatorPosition(1);
                    manipulatorPosY(end+1) = manipulatorPosition(3);
                    manipulatorTheta0(end+1) = J0;
                    manipulatorTheta1(end+1) = J1;
                    manipulatorTheta2(end+1) = J2;
                    manipulatorTheta3(end+1) = J3;
                    manipulatorTheta4(end+1) = J4;
                    T_manipulator(end+1) = Time/1000;
                    % Manipulator Joints
                    [~] = vrep.simxSetJointPosition(clientID, joint0, 0, vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint1, Jtraj(i,1)-pi/2, vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint2, Jtraj(i,2), vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint3, Jtraj(i,3), vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint4, 0, vrep.simx_opmode_streaming);
                    pause(0.1);
                end
                % Move Manipulator in a Semicircular Trajectory
                Ji = Jf;
                Jf = [deg2rad(155),0,0];
                Jtraj = jtraj(Ji,Jf,150);
                for i=1:length(Jtraj)
                    % Record Manipulator Robot Data
                    [~,manipulatorPosition]=vrep.simxGetObjectPosition(clientID,endeffector,-1,vrep.simx_opmode_streaming);
                    [~,manipulatorOrientation]=vrep.simxGetObjectOrientation(clientID,endeffector,-1,vrep.simx_opmode_streaming);
                    [~,J0]=vrep.simxGetJointPosition(clientID,joint0,vrep.simx_opmode_streaming);
                    [~,J1]=vrep.simxGetJointPosition(clientID,joint1,vrep.simx_opmode_streaming);
                    [~,J2]=vrep.simxGetJointPosition(clientID,joint2,vrep.simx_opmode_streaming);
                    [~,J3]=vrep.simxGetJointPosition(clientID,joint3,vrep.simx_opmode_streaming);
                    [~,J4]=vrep.simxGetJointPosition(clientID,joint4,vrep.simx_opmode_streaming);
                    [Time]=vrep.simxGetLastCmdTime(clientID);
                    manipulatorPosX(end+1) = manipulatorPosition(1);
                    manipulatorPosY(end+1) = manipulatorPosition(3);
                    manipulatorTheta0(end+1) = J0;
                    manipulatorTheta1(end+1) = J1;
                    manipulatorTheta2(end+1) = J2;
                    manipulatorTheta3(end+1) = J3;
                    manipulatorTheta4(end+1) = J4;
                    T_manipulator(end+1) = Time/1000;
                    % Manipulator Joints
                    [~] = vrep.simxSetJointPosition(clientID, joint0, 0, vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint1, Jtraj(i,1)-pi/2, vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint2, Jtraj(i,2), vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint3, Jtraj(i,3), vrep.simx_opmode_streaming);
                    [~] = vrep.simxSetJointPosition(clientID, joint4, 0, vrep.simx_opmode_streaming);
                    pause(0.1);
                end
                % Plot Manipulator Robot Data
                figure(3)
                sgtitle('Manipulator Robot Position Analysis')
                subplot(1,4,1), plot(T_manipulator,manipulatorPosX)
                xlabel("Simulation Time (s)")
                ylabel("X Position (m)")
                subplot(1,4,2), plot(T_manipulator,manipulatorPosY)
                xlabel("Simulation Time (s)")
                ylabel("Y Position (m)")
                subplot(1,4,3), plot(manipulatorPosX,manipulatorPosY)
                xlabel("X Position (m)")
                ylabel("Y Position (m)")
                subplot(1,4,4)
                hold on
                plot(T_manipulator,manipulatorTheta0)
                plot(T_manipulator,manipulatorTheta1)
                plot(T_manipulator,manipulatorTheta2)
                plot(T_manipulator,manipulatorTheta3)
                plot(T_manipulator,manipulatorTheta4)
                hold off
                xlabel("Simulation Time (s)")
                ylabel("Joint Angles (rad)")
                legend("J0","J1","J2","J3","J4",'Location','EO')
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