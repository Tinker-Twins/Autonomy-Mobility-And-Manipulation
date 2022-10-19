clc;
clear;
%  r=rad of wheel, l=distance from robot center to wheel hub (meters), w
%  w=wheel seperation width 
r=50/1000; l=471/1000; wid=300.46/1000; %m
WHEEL_SEPARATION_WIDTH= wid/2; %m
WHEEL_SEPARATION_LENGTH= l/2; %m
%Case 1 no rotation about z, move in y at 0.1 for 5
linear_y1=0.1; %m/s
linear_x1=0; %m/s
ang_z_1=0.0; %rad/s of z rotation needed 
W_1_1 = (1/r) * (linear_x1 - linear_y1 + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z_1); %rr
W_2_1 = (1/r) * (linear_x1 + linear_y1 + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z_1); %fr
W_3_1 = (1/r) * (linear_x1 + linear_y1 - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z_1); %rl
W_4_1 = (1/r) * (linear_x1 - linear_y1 - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z_1); %fl
W1=[W_1_1;W_2_1;W_3_1;W_4_1];
%case 2 
linear_y2=0.0; %m/s
linear_x2=0; %m/s
ang_z_2=0.5; %rad/s of z rotation needed 
W_1_2 = (1/r) * (linear_x2 - linear_y2 + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z_2); %rr
W_2_2 = (1/r) * (linear_x2 + linear_y2 + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z_2); %fr
W_3_2 = (1/r) * (linear_x2 + linear_y2 - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z_2); %rl
W_4_2 = (1/r) * (linear_x2 - linear_y2 - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z_2); %fl


vrep = remApi('remoteApi');
vrep.simxFinish(-1);

clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected');
    [r, w1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_blocking);
    [r, w2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_blocking);
    [r, w3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_blocking);
    [r, w4] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_blocking);
    [r, robot] = vrep.simxGetObjectHandle(clientID, 'youBot', vrep.simx_opmode_blocking);
    [r, floor] = vrep.simxGetObjectHandle(clientID, 'infiniteFloor', vrep.simx_opmode_blocking);

while true
    [r,positionRobot]=vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_streaming);
    robotX = positionRobot(1);
    robotY = positionRobot(2);
 
if robotY<1
        [r] = vrep.simxSetJointTargetVelocity(clientID, w1, -W_1_1, vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointTargetVelocity(clientID, w2, W_2_1, vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointTargetVelocity(clientID, w3, W_3_1, vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointTargetVelocity(clientID, w4, -W_4_1, vrep.simx_opmode_streaming);
        [r,positionRobot]=vrep.simxGetObjectPosition(clientID,robot,floor,vrep.simx_opmode_buffer);
        robotX = positionRobot(1);
        robotY = positionRobot(2);

elseif robotY>=1 
%         [r] = vrep.simxSetJointTargetVelocity(clientID, w1, 0, vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointTargetVelocity(clientID, w2,  0, vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointTargetVelocity(clientID, w3,  0, vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointTargetVelocity(clientID, w4,  0, vrep.simx_opmode_streaming);
%      
        [r] = vrep.simxSetJointTargetVelocity(clientID,w1, W_1_2, vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointTargetVelocity(clientID,w2, W_2_2, vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointTargetVelocity(clientID,w3, W_3_2, vrep.simx_opmode_streaming);
        [r] = vrep.simxSetJointTargetVelocity(clientID,w4, W_4_2, vrep.simx_opmode_streaming);
end
% 
%         [r] = vrep.simxSetJointTargetVelocity(clientID, w1, 0, vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointTargetVelocity(clientID, w2,  0, vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointTargetVelocity(clientID, w3,  0, vrep.simx_opmode_streaming);
%         [r] = vrep.simxSetJointTargetVelocity(clientID, w4,  0, vrep.simx_opmode_streaming);
%      
     
      
end
    
 else
        disp('Failed connecting to remote API server');
 end
    vrep.sim.delete(); % call the destructor!
    
    disp('Program ended');