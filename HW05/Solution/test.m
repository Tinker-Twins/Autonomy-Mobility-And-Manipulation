clc;
clear;
vrep = remApi('remoteApi');
vrep.simxFinish(-1);

clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
%     [r, jo] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint0', vrep.simx_opmode_blocking);
%     [r, j1] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint1', vrep.simx_opmode_blocking);
%     [r, j2] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint2', vrep.simx_opmode_blocking);
%     [r, j3] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint3', vrep.simx_opmode_blocking);
%     [r, j4] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint4', vrep.simx_opmode_blocking);
%     [r, gripper1] = vrep.simxGetObjectHandle(clientID, 'youBotGripperjoint1', vrep.simx_opmode_blocking);
%     [r, gripper2] = vrep.simxGetObjectHandle(clientID, 'youBotGripperjoint2', vrep.simx_opmode_blocking);
   
    i = 0;
%     while (i<500)
%         i = i+1;
%         [r] = vrep.simxSetJointPosition(clientID, jo, deg2rad(4), vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointPosition(clientID, j1, deg2rad(40), vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointPosition(clientID, j2, deg2rad(45), vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointPosition(clientID, j3, deg2rad(80), vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointPosition(clientID, j4, deg2rad(0), vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointTargetVelocity(clientID, gripper2, 0.1, vrep.simx_opmode_streaming);
%     end
%         [r] = vrep.simxSetJointPosition(clientID, jo, deg2rad(3), vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointPosition(clientID, j1, deg2rad(10), vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointPosition(clientID, j2, deg2rad(45), vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointPosition(clientID, j3, deg2rad(80), vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointPosition(clientID, j4, deg2rad(0), vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointTargetVelocity(clientID, gripper2, 0.1, vrep.simx_opmode_streaming);