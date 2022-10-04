%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 2: Interpolation, Inverse Kinematics %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti, Olamide Akinyele

fprintf('\n=========================================================================\n');
fprintf('INTERPOLATION AND INVERSE KINEMATICS\n');
fprintf('=========================================================================\n');

clc 
close all 
clear all 
%% Problem 1 - A %%
clc 
close all 
clear all
fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1 - A\n');
fprintf('----------------------------------------------------------------\n\n');

Rz_1=linspace(0,((pi/4)),100);
Rz_2=linspace(0,((pi/9)),100);
Ry_1=linspace(0,((pi/6)),100);


for i=1:length(Rz_1)
   
Rz11(:,:,i) = [cos(Rz_1(i)) -sin(Rz_1(i)) 0;
        sin(Rz_1(i)) cos(Rz_1(i))  0;
        0        0           1];

Ry12(:,:,i) = [cos(Ry_1(i))  0 sin(Ry_1(i));
        0          1 0        ;
        -sin(Ry_1(i)) 0 cos(Ry_1(i))];

Rz13(:,:,i) = [cos(Rz_2(i)) -sin(Rz_2(i)) 0;
        sin(Rz_2(i)) cos(Rz_2(i))  0;
        0         0          1];
Rzyz(:,:,i) = Rz11(:,:,i)*Ry12(:,:,i)*Rz13(:,:,i);
end 
fprintf("Final orientation after relative Rzyz rotations:\n")
disp(Rzyz(:,:,100))
figure('Name','Animation for 1-A')
tranimate(Rzyz,'rgb','fps',100);


%% -------------1B----------------------------
 
Rz11= [cos((pi/4)) -sin((pi/4)) 0;
        sin((pi/4)) cos((pi/4))  0;
        0        0           1];

Ry12= [cos(pi/6)  0 sin(pi/6);
        0          1 0        ;
        -sin(pi/6) 0 cos(pi/6)];

Rz13= [cos(pi/9) -sin(pi/9) 0;
        sin(pi/9) cos(pi/9)  0;
        0         0          1];
Rzyz= Rz11*Ry12*Rz13;


%finding roll pitch yaw angles given Rzyz 
theta1=atan2(sqrt(1-(Rzyz(3,3)^2)),Rzyz(3,3));
theta2=atan2(-sqrt(1-(Rzyz(3,3)^2)),Rzyz(3,3));
phi1 = atan2( Rzyz(2,3),Rzyz(1,3));
phi2 = atan2(-Rzyz(2,3),-Rzyz(1,3));
psi1 = atan2(Rzyz(3,2),-Rzyz(3,1));
psi2 = atan2(-Rzyz(3,2),Rzyz(3,1));

Rz_phi=linspace(0,phi1,100);
Rz_psi=linspace(0,psi1,100);
Ry_theta=linspace(0,theta1,100);
i=0;
for i=1:length(Rz_phi)
   
Rz_phi1(:,:,i) = [cos(Rz_phi(i)) -sin(Rz_phi(i)) 0;
        sin(Rz_phi(i)) cos(Rz_phi(i))  0;
        0        0           1];

Ry_theta1(:,:,i) = [cos(Ry_theta(i))  0 sin(Ry_theta(i));
        0          1 0        ;
        -sin(Ry_theta(i)) 0 cos(Ry_theta(i))];

Rz_psi1(:,:,i) = [cos(Rz_psi(i)) -sin(Rz_psi(i)) 0;
        sin(Rz_psi(i)) cos(Rz_psi(i))  0;
        0         0          1];
Rxyz(:,:,i) = Rz_phi1(:,:,i)*Ry_theta1(:,:,i)*Rz_psi1(:,:,i);
end 
fprintf("Final orientation after relative Rxyz rotations:\n")
disp(Rxyz(:,:,end))
figure('Name','Animation for 1-B')
tranimate(Rxyz,'rgb','fps',100);

%% Problem 1 - C %%
fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1 - C\n');
fprintf('----------------------------------------------------------------\n\n');
% ZYZ relative rotation
Ri = rotz(0)*roty(0)*rotz(0);
Rf = rotz(rad2deg(pi/4))*roty(rad2deg(pi/6))*rotz(rad2deg(pi/9));
% Convert to homogenous transformation
Ti = r2t(Ri);
Tf = r2t(Rf);
% Interpolate and plot
Tint = trinterp(Ti, Tf, 100);
figure('Name','Plot for 1-C')
trplot(Ti,'rgb','frame',num2str(0))
hold on
for i=1:length(Tint)
    if mod(i,20)==0
        trplot(Tint(:,:,i),'rgb','frame',num2str(i/20))
    end
end
hold off
% Animate
figure('Name','Animation for 1-C')
tranimate(Tint,'rgb','fps',10);

%% Problem 1 - D %%
fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1 - D\n');
fprintf('----------------------------------------------------------------\n\n');
% ZYZ relative rotation
Ri = rotz(0)*roty(0)*rotz(0);
Rf = rotz(rad2deg(pi/4))*roty(rad2deg(pi/6))*rotz(rad2deg(pi/9));
% Convert to PRY absolute roll-pitch-yaw angles
RPYi = rad2deg(tr2rpy(Ri)); % PCRTB v10 default is absolute XYZ convention (i.e., relative ZYX)
RPYf = rad2deg(tr2rpy(Rf)); % PCRTB v10 default is absolute XYZ convention (i.e., relative ZYX)
% Convert to homogenous transformation
Ti = rpy2tr(RPYi);
Tf = rpy2tr(RPYf);
% Interpolate and plot
Tint = trinterp(Ti, Tf, 100);
figure('Name','Plot for 1-D')
trplot(Ti,'rgb','frame',num2str(0))
hold on
for i=1:length(Tint)
    if mod(i,20)==0
        trplot(Tint(:,:,i),'rgb','frame',num2str(i/20))
    end
end
hold off
% Animate
figure('Name','Animation for 1-D')
tranimate(Tint,'rgb','fps',10);

%% Problem 2 - B %%

clc
close all 
clear all
%%%%%------------------ X position trajectory----------------------------

q0_x =-1; %initial position
v0 = 0; %initial velocity
q1_x =2; %final position
v1 = 0; %final velocity
t0 =0; %initial time
tf =10; %final time

d_x = [q0_x,v0,q1_x,v1,t0,tf];
q0_x = d_x(1); v0_x = d_x(2); q1_x = d_x(3); v1_x = d_x(4);
t0_x = d_x(5); tf = d_x(6);
t = linspace(t0,tf,100*(tf-t0));
c = ones(size(t));

M_x = [ 1 t0_x t0_x^2 t0_x^3;
0 1 2*t0_x 3*t0_x^2;
1 tf tf^2 tf^3;
0 1 2*tf 3*tf^2];

b_x = [q0_x; v0_x; q1_x; v1_x];
a_x = inv(M_x)*b_x;

% qd = reference position trajectory
% vd = reference velocity trajectory
% ad = reference acceleration trajectory

qd_x = a_x(1).*c + a_x(2).*t +a_x(3).*t.^2 + a_x(4).*t.^3;
vd_x = a_x(2).*c +2*a_x(3).*t +3*a_x(4).*t.^2;
ad_x = 2*a_x(3).*c + 6*a_x(4).*t;


%%%%%------------------ Y position trajectory----------------------------
q0_y =1; %initial position
v0 = 0; %initial velocity
q1_y =1; %final position
v1 = 0; %final velocity
t0 =0; %initial time
tf =10; %final time

d_y = [q0_y,v0,q1_y,v1,t0,tf];
q0_y = d_y(1); v0_y = d_y(2); q1_y = d_y(3); v1_y = d_y(4);
t0_y = d_y(5); tf = d_y(6);
t = linspace(t0,tf,100*(tf-t0));
c = ones(size(t));

M_y = [ 1 t0_y t0_y^2 t0_y^3;
0 1 2*t0_y 3*t0_y^2;
1 tf tf^2 tf^3;
0 1 2*tf 3*tf^2];
%
b_y = [q0_y; v0_y; q1_y; v1_y];
a_y = inv(M_y)*b_y;

qd_y = a_y(1).*c + a_y(2).*t +a_y(3).*t.^2 + a_y(4).*t.^3;
vd_y = a_y(2).*c +2*a_y(3).*t +3*a_y(4).*t.^2;
ad_y = 2*a_y(3).*c + 6*a_y(4).*t;

%x vx  t 
figure 
hold on
plot(t,qd_x)
title('X vs. t')
xlabel('Time(s)')
ylabel('X position')
hold off

%y vx  t 
figure 
hold on
plot(t,qd_y)
title('Y vs. t')
xlabel('Time(s)')
ylabel('Y position')
hold off

%y vs x
figure 
hold on
plot(qd_x,qd_y)
title('Y vs. X Position')
xlabel('X position')
ylabel('Y position')
hold off




