clc;
clear;
%  r=rad of wheel, l=distance from robot center to wheel hub (meters), w
%  w=wheel seperation width 
r=50/1000; l=471/1000; wid=300.46/1000; %m
WHEEL_SEPARATION_WIDTH= wid/2; %m
WHEEL_SEPARATION_LENGTH= l/2; %m
%dont move
%case 0 
linear_y2=0.0; %m/s
linear_x2=0; %m/s
ang_z_2=0; %rad/s of z rotation needed 
W_1_0 = (1/r) * (linear_x2 - linear_y2 + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z_2); %rr
W_2_0 = (1/r) * (linear_x2 + linear_y2 + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z_2); %fr
W_3_0 = (1/r) * (linear_x2 + linear_y2 - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z_2); %rl
W_4_0 = (1/r) * (linear_x2 - linear_y2 - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z_2); %fl



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
    [return_code, w1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_blocking);
    [return_code, w2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_blocking);
    [return_code, w3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_blocking);
    [return_code, w4] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_blocking);
    [return_code, robot] = vrep.simxGetObjectHandle(clientID, 'youBot', vrep.simx_opmode_blocking);
    [return_code, floor] = vrep.simxGetObjectHandle(clientID, 'Floor', vrep.simx_opmode_blocking);
    tableX=[];
    tableY=[]; 
    Time=[];
k=1;
while true
    [return_code,positionRobot]=vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_streaming);
    robotX = positionRobot(1);
    robotY = positionRobot(2);
    [Time]=vrep.simxGetLastCmdTime(clientID);
    [return_code,array_eulerAngles]=vrep.simxGetObjectOrientation(clientID,robot,-1,vrep.simx_opmode_streaming);

%case 1
if robotY>-5
        [return_code] = vrep.simxSetJointTargetVelocity(clientID, w1, -W_1_1, vrep.simx_opmode_streaming);
        [return_code] = vrep.simxSetJointTargetVelocity(clientID, w2, W_2_1, vrep.simx_opmode_streaming);
        [return_code] = vrep.simxSetJointTargetVelocity(clientID, w3, W_3_1, vrep.simx_opmode_streaming);
        [return_code] = vrep.simxSetJointTargetVelocity(clientID, w4, -W_4_1, vrep.simx_opmode_streaming);
        tableX(k)=robotX; %storing X and Y position 
        tableY(k)=robotY;
        T(k)=Time/1000;
%case 2
elseif robotY<=-5
    
        [return_code] = vrep.simxSetJointTargetVelocity(clientID,w1, W_1_2, vrep.simx_opmode_streaming);
        [return_code] = vrep.simxSetJointTargetVelocity(clientID,w2, W_2_2, vrep.simx_opmode_streaming);
        [return_code] = vrep.simxSetJointTargetVelocity(clientID,w3, W_3_2, vrep.simx_opmode_streaming);
        [return_code] = vrep.simxSetJointTargetVelocity(clientID,w4, W_4_2, vrep.simx_opmode_streaming);     
%once robot completes one rotation stop 
if abs(array_eulerAngles(2))>=(1.98*pi)
        [return_code] = vrep.simxSetJointTargetVelocity(clientID, w1, 0, vrep.simx_opmode_streaming);
        [return_code] = vrep.simxSetJointTargetVelocity(clientID, w2,  0, vrep.simx_opmode_streaming);
        [return_code] = vrep.simxSetJointTargetVelocity(clientID, w3,  0, vrep.simx_opmode_streaming);
        [return_code] = vrep.simxSetJointTargetVelocity(clientID, w4,  0, vrep.simx_opmode_streaming);
[return_code] = simxStopSimulation(clientID,simx_opmode_oneshot);
end
 
end 

k=k+1;
end


vrep.delete();
disp('Program Ended');
 %i was not able to use thes figures outside of the command window but the
 %data was stored in the 'tableX and tableY'
figure 
scatter(T,tableX)
title('X vs. time')
xlabel('Time(s)')
ylabel('X Position')


figure 
scatter(T,tableY)
title('Y vs. time')
xlabel('Time(s)')
ylabel('Y Position')


figure 
scatter(tableX,tableY)
title('X vs. Y')
xlabel('X position')
ylabel('Y Position')
end

