%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 2: Interpolation, Inverse Kinematics %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti, Olamide Akinyele

fprintf('\n=========================================================================\n');
fprintf('INTERPOLATION AND INVERSE KINEMATICS\n');
fprintf('=========================================================================\n');

%% Problem 1 - A %%
fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1 - A\n');
fprintf('----------------------------------------------------------------\n\n');

% Interpolate individual angles (relative ZYZ)
z11=linspace(0,((pi/4)),100);
y12=linspace(0,((pi/6)),100);
z13=linspace(0,((pi/9)),100);

% Compute rotation for each of the interpolation
for i=1:length(z11)
Rz11(:,:,i) = [cos(z11(i)) -sin(z11(i)) 0;
               sin(z11(i)) cos(z11(i))  0;
               0           0            1];

Ry12(:,:,i) = [cos(y12(i))  0 sin(y12(i));
               0            1 0          ;
               -sin(y12(i)) 0 cos(y12(i))];

Rz13(:,:,i) = [cos(z13(i)) -sin(z13(i)) 0;
               sin(z13(i)) cos(z13(i))  0;
               0           0            1];
Rzyz(:,:,i) = Rz11(:,:,i)*Ry12(:,:,i)*Rz13(:,:,i);
end

% Output
fprintf("Final orientation after relative Rzyz rotations:\n")
disp(Rzyz(:,:,end))

% Plot
figure('Name','Plot 1-A')
trplot(Rzyz(:,:,1),'rgb','frame',num2str(0)) % Using PCRTB ONLY for plotting
%poseplot(Rzyz(:,:,1)) % Another way to plot rotation matrix, but the convention is weird
hold on
for i=1:length(Rzyz)
    if mod(i,20)==0
        trplot(Rzyz(:,:,i),'rgb','frame',num2str(i/20)) % Using PCRTB ONLY for plotting
        %poseplot(Rzyz(:,:,i)) % Another way to plot rotation matrix, but the convention is weird
        hold on
    end
end
hold off

% Animation
figure('Name','Animation 1-A')
tranimate(Rzyz,'rgb','fps',10);

%% Problem 1 - B %%
fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1 - B\n');
fprintf('----------------------------------------------------------------\n\n');

% Compute relative Rzyz from given angles
Rz11 = [cos(pi/4) -sin(pi/4) 0;
        sin(pi/4) cos(pi/4)  0;
        0         0          1];
Ry12 = [cos(pi/6)  0 sin(pi/6);
        0          1 0        ;
        -sin(pi/6) 0 cos(pi/6)];
Rz13 = [cos(pi/9) -sin(pi/9) 0;
        sin(pi/9) cos(pi/9)  0;
        0         0          1];
Rzyz = Rz11*Ry12*Rz13;

% Compute absolute Rxyz from relative Rzyz 
theta1=atan2(sqrt(1-(Rzyz(3,3)^2)),Rzyz(3,3));
theta2=atan2(-sqrt(1-(Rzyz(3,3)^2)),Rzyz(3,3));
phi1 = atan2(Rzyz(2,3),Rzyz(1,3));
phi2 = atan2(-Rzyz(2,3),-Rzyz(1,3));
psi1 = atan2(Rzyz(3,2),-Rzyz(3,1));
psi2 = atan2(-Rzyz(3,2),Rzyz(3,1));

% Interpolate individual angles (one set of absolute XYZ)
phi=linspace(0,phi1,100);
theta=linspace(0,theta1,100);
psi=linspace(0,psi1,100);

% Compute rotation for each of the interpolation
for i=1:length(phi)
Rz_phi(:,:,i) = [cos(phi(i)) -sin(phi(i)) 0;
                 sin(phi(i)) cos(phi(i))  0;
                 0           0            1];

Ry_theta(:,:,i) = [cos(theta(i))  0 sin(theta(i));
                   0              1 0            ;
                   -sin(theta(i)) 0 cos(theta(i))];

Rz_psi(:,:,i) = [cos(psi(i)) -sin(psi(i)) 0;
                 sin(psi(i)) cos(psi(i))  0;
                 0           0            1];
Rxyz(:,:,i) = Rz_phi(:,:,i)*Ry_theta(:,:,i)*Rz_psi(:,:,i);
end

% Output
fprintf("Final orientation after absolute Rxyz rotations:\n")
disp(Rxyz(:,:,end))

% Plot
figure('Name','Plot 1-B')
trplot(Rxyz(:,:,1),'rgb','frame',num2str(0)) % Using PCRTB ONLY for plotting
%poseplot(Rxyz(:,:,1)) % Another way to plot rotation matrix, but the convention is weird
hold on
for i=1:length(Rxyz)
    if mod(i,20)==0
        trplot(Rxyz(:,:,i),'rgb','frame',num2str(i/20)) % Using PCRTB ONLY for plotting
        %poseplot(Rxyz(:,:,i)) % Another way to plot rotation matrix, but the convention is weird
        hold on
    end
end
hold off

% Animation
figure('Name','Animation 1-B')
tranimate(Rxyz,'rgb','fps',10);

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

% Output
fprintf("Final orientation after relative Rzyz rotations:\n")
disp(Rf)

% Interpolate and plot
Tint = trinterp(Ti, Tf, 100);
figure('Name','Plot 1-C')
trplot(Ti,'rgb','frame',num2str(0))
hold on
for i=1:length(Tint)
    if mod(i,20)==0
        trplot(Tint(:,:,i),'rgb','frame',num2str(i/20))
    end
end
hold off

% Animation
figure('Name','Animation 1-C')
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

% Output
fprintf("Final RPY after absolute Rxyz rotations:\n")
disp(RPYf)

% Interpolate and plot
Tint = trinterp(Ti, Tf, 100);
figure('Name','Plot 1-D')
trplot(Ti,'rgb','frame',num2str(0))
hold on
for i=1:length(Tint)
    if mod(i,20)==0
        trplot(Tint(:,:,i),'rgb','frame',num2str(i/20))
    end
end
hold off

% Animation
figure('Name','Animation 1-D')
tranimate(Tint,'rgb','fps',10);

%% Problem 2 - A %%
fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 2 - A\n');
fprintf('----------------------------------------------------------------\n\n');

close all;
clear;
clc;

% Robot parameters
a1 = 2;
a2 = 1;

% Robot definition using PCRTB
L(1) = Link([0 0 2 0 0],'R'); % First '0' doesn't mean 0. MATLAB takes it in as a variable 'theta' because we specify 'R', which defines a revolute joint.
L(2) = Link([0 0 1 0 0],'R');
Robot = SerialLink(L, 'name', '2-R Planar Robot') % Concatenate the links as a serial robot

% Cubic polynomial interpolation for x coordinate
ti = 0; % Initial time
xi = -1; % Initial position
vi = 0; % Initial velocity
tf = 10; % Final time
xf = 2; % Final position
vf = 0; % Final velocity
% x(t) = a0 + a1t + a2t^2 + a3t^3
a = [1 ti ti^2 ti^3; % Matrix of time (t^0 - t^3) terms
     0 1  2*ti 3*ti^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2];
c = [xi; vi; xf; vf]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a3)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
x_2a = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3; % x trajectory

% Cubic polynomial interpolation for y coordinate
ti = 0; % Initial time
yi = 1; % Initial position
vi = 0; % Initial velocity
tf = 10; % Final time
yf = 1; % Final position
vf = 0; % Final velocity
% y(t) = b0 + b1t + b2t^2 + b3t^3
a = [1 ti ti^2 ti^3; % Matrix of time (t^0 - t^3) terms
     0 1  2*ti 3*ti^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2];
c = [yi; vi; yf; vf]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a3)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
y_2a = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3; % y trajectory

% IK of 2R planar robot to find theta1 and theta2 for all (x,y) positions
for i=1:length(t)
theta2_2a(i) = real(acosd((x_2a(i)^2+y_2a(i)^2-a1^2-a2^2)/2*a1*a2));
theta1_2a(i) = real(atand((y_2a(i)*(a1+a2*cosd(theta2_2a(i)))-x_2a*(a2*sind(theta2_2a(i))))/ ...
                          (x_2a(i)*(a1+a2*cosd(theta2_2a(i)))+y_2a*(a2*sind(theta2_2a(i))))));

theta(:,i) = Robot.ikine(transl(x_2a(i),y_2a(i),0), 'mask',[1, 1, 0, 0, 0, 0], 'ru');
end

% Plots
figure('Name','Plot 2-A')
hold on
plot(t,x_2a)
xlabel('Time (s)')
ylabel('Position (x)')
hold off

figure()
plot(t,y_2a)
xlabel('Time (s)')
ylabel('Position (y)')

figure()
plot(x_2a,y_2a)
xlabel('Position (x)')
ylabel('Position (y)')

figure()
hold on
plot(t,theta1_2a)
plot(t,rad2deg(theta(1,:)))
xlabel('Time (s)')
ylabel('\theta_1 (deg)')
hold off

figure()
hold on
plot(t,theta2_2a)
plot(t,rad2deg(theta(2,:)))
xlabel('Time (s)')
ylabel('\theta_2 (deg)')
hold off
