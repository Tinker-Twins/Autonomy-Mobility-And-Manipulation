%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 3: Interpolation, Inverse Kinematics %
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
plot_1 = figure('Name','Plot 1');
figure(plot_1)
trplot(Rzyz(:,:,1),'rgb','frame',num2str(0)) % Using PCRTB ONLY for plotting
%poseplot(Rzyz(:,:,1)) % Another way to plot rotation matrix, but the convention is weird
hold on
for i=1:length(Rzyz)
    if mod(i,20)==0
        trplot(Rzyz(:,:,i),'rgb','frame',num2str(i/20)) % Using PCRTB ONLY for plotting
        %poseplot(Rzyz(:,:,i)) % Another way to plot rotation matrix, but the convention is weird
    end
end

% Animation
figure('Name','Animation 1-A')
tranimate(Rzyz,'rgb','fps',10); % Visualize the transformation
%tranimate(Rzyz,'rgb','fps',10,'movie','Results\Problem 1\Animation 1-A.mp4'); % Save transformation animation

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
phi1 = atan2(Rzyz(3,2),Rzyz(3,3));
phi2 = atan2(-Rzyz(3,2),-Rzyz(3,3));
theta1 = atan2(-Rzyz(3,1),sqrt((Rzyz(1,1)^2)+(Rzyz(2,1)^2)));
theta2 = atan2(-Rzyz(3,1),-sqrt((Rzyz(1,1)^2)+(Rzyz(2,1)^2)));
psi1 = atan2(Rzyz(2,1),Rzyz(1,1));
psi2 = atan2(-Rzyz(2,1),-Rzyz(1,1));

% Interpolate individual angles (one set of absolute XYZ)
phi=linspace(0,phi1,100);
theta=linspace(0,theta1,100);
psi=linspace(0,psi1,100);

% Compute rotation for each of the interpolation
for i=1:length(phi)
Rx_phi(:,:,i) = [1 0           0           ;
                 0 cos(phi(i)) -sin(phi(i));
                 0 sin(phi(i)) cos(phi(i)) ];

Ry_theta(:,:,i) = [cos(theta(i))  0 sin(theta(i));
                   0              1 0            ;
                   -sin(theta(i)) 0 cos(theta(i))];

Rz_psi(:,:,i) = [cos(psi(i)) -sin(psi(i)) 0;
                 sin(psi(i)) cos(psi(i))  0;
                 0           0            1];
Rxyz(:,:,i) = Rz_psi(:,:,i)*Ry_theta(:,:,i)*Rx_phi(:,:,i);
end

% Output
fprintf("Final RPY after absolute Rxyz rotations:\n")
RPY = [phi1 theta1 psi1];
disp(rad2deg(RPY))

% Plot
figure(plot_1)
hold on
trplot(Rxyz(:,:,1),'rgb','frame',num2str(0)) % Using PCRTB ONLY for plotting
%poseplot(Rxyz(:,:,1)) % Another way to plot rotation matrix, but the convention is weird
for i=1:length(Rxyz)
    if mod(i,20)==0
        trplot(Rxyz(:,:,i),'rgb','frame',num2str(i/20)) % Using PCRTB ONLY for plotting
        %poseplot(Rxyz(:,:,i)) % Another way to plot rotation matrix, but the convention is weird
    end
end

% Animation
figure('Name','Animation 1-B')
tranimate(Rxyz,'rgb','fps',10); % Visualize the transformation
%tranimate(Rxyz,'rgb','fps',10,'movie','Results\Problem 1\Animation 1-B.mp4'); % Save transformation animation

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
figure(plot_1)
hold on
trplot(Ti,'rgb','frame',num2str(0))
for i=1:length(Tint)
    if mod(i,20)==0
        trplot(Tint(:,:,i),'rgb','frame',num2str(i/20))
    end
end

% Animation
figure('Name','Animation 1-C')
tranimate(Tint,'rgb','fps',10); % Visualize the transformation
%tranimate(Tint,'rgb','fps',10,'movie','Results\Problem 1\Animation 1-C.mp4'); % Save transformation animation

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
figure(plot_1)
hold on
trplot(Ti,'rgb','frame',num2str(0))
for i=1:length(Tint)
    if mod(i,20)==0
        trplot(Tint(:,:,i),'rgb','frame',num2str(i/20))
    end
end
hold off

% Animation
figure('Name','Animation 1-D')
tranimate(Tint,'rgb','fps',10); % Visualize the transformation
%tranimate(Tint,'rgb','fps',10,'movie','Results\Problem 1\Animation 1-D.mp4'); % Save transformation animation

%% Problem 2 - A %%
fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 2 - A\n');
fprintf('----------------------------------------------------------------\n\n');

% Robot parameters
a1 = 2;
a2 = 1;

% Cubic polynomial interpolation for x coordinate
ti = 0; % Initial time
xi = -1; % Initial x position
vi = 0; % Initial x velocity
tf = 10; % Final time
xf = 2; % Final x position
vf = 0; % Final x velocity
fprintf('x(t) = a0 + a1t + a2t^2 + a3t^3\n\n')
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
yi = 1; % Initial y position
vi = 0; % Initial y velocity
tf = 10; % Final time
yf = 1; % Final y position
vf = 0; % Final y velocity
fprintf('y(t) = b0 + b1t + b2t^2 + b3t^3\n\n')
a = [1 ti ti^2 ti^3; % Matrix of time (t^0 - t^3) terms
     0 1  2*ti 3*ti^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2];
c = [yi; vi; yf; vf]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a3)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
y_2a = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3; % y trajectory

% IK of 2R planar robot to find theta1 and theta2 for all (x,y) positions
% NOTE: Using only 1 (elbow down) of 2 possible solutions
for i=1:length(t)
    theta2_2a(i) = acos((x_2a(i)^2+y_2a(i)^2-a1^2-a2^2)/(2*a1*a2));
    theta1_2a(i) = atan2(y_2a(i),x_2a(i)) - atan2((a2*sin(theta2_2a(i))),(a1+a2*cos(theta2_2a(i))));
end

%% Problem 2 - B %%
fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 2 - B\n');
fprintf('----------------------------------------------------------------\n\n');

% Robot parameters
a1 = 2;
a2 = 1;

% Quintic polynomial interpolation for joint angle 1 (theta1)
ti = 0; % Initial time
qi = theta1_2a(1); % Initial theta1 position
vi = 0; % Initial theta1 velocity
ai = 0; % Initial theta1 acceleration
tf = 10; % Final time
qf = theta1_2a(end); % Final theta1 position
vf = 0; % Final theta1 velocity
af = 0; % Final theta1 acceleration
fprintf('theta_1(t) = a0 + a1t + a2t^2 + a3t^3 + a4t^4 + a5t^5\n\n')
a = [1 ti ti^2 ti^3   ti^4    ti^5; % Matrix of time (t^0 - t^5) terms
     0 1  2*ti 3*ti^2 4*ti^3  5*ti^4;
     0 0  2    6*ti   12*ti^2 20*ti^3;
     1 tf tf^2 tf^3   tf^4    tf^5;
     0 1  2*tf 3*tf^2 4*tf^3  5*tf^4;
     0 0  2    6*tf   12*tf^2 20*tf^3;];
c = [qi; vi; ai; qf; vf; af]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a5)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
theta1_2b = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3 + b(5).*t.^4 + b(6).*t.^5; % theta1 trajectory

% Quintic polynomial interpolation for joint angle 2 (theta2)
ti = 0; % Initial time
qi = theta2_2a(1); % Initial theta2 position
vi = 0; % Initial theta2 velocity
ai = 0; % Initial theta2 acceleration
tf = 10; % Final time
qf = theta2_2a(end); % Final theta2 position
vf = 0; % Final theta2 velocity
af = 0; % Final theta2 acceleration
fprintf('theta_2(t) = b0 + b1t + b2t^2 + b3t^3 + b4t^4 + b5t^5\n\n')
a = [1 ti ti^2 ti^3   ti^4    ti^5; % Matrix of time (t^0 - t^5) terms
     0 1  2*ti 3*ti^2 4*ti^3  5*ti^4;
     0 0  2    6*ti   12*ti^2 20*ti^3;
     1 tf tf^2 tf^3   tf^4    tf^5;
     0 1  2*tf 3*tf^2 4*tf^3  5*tf^4;
     0 0  2    6*tf   12*tf^2 20*tf^3;];
c = [qi; vi; ai; qf; vf; af]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a5)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
theta2_2b = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3 + b(5).*t.^4 + b(6).*t.^5; % theta2 trajectory

% FK of 2R planar robot to find (x,y) positions for all theta1 and theta2
for i=1:length(t)
    x_2b(i) = a1*cos(theta1_2b(i))+a2*cos(theta1_2b(i)+theta2_2b(i));
    y_2b(i) = a1*sin(theta1_2b(i))+a2*sin(theta1_2b(i)+theta2_2b(i));
end

%% Problem 2 - C %%
fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 2 - C\n');
fprintf('----------------------------------------------------------------\n\n');

% Robot definition using PCRTB (DH parametrization)
L(1) = Link([0 0 2 0 0],'R'); % First '0' doesn't mean 0. MATLAB takes it in as a variable 'theta' because we specify 'R', which defines a revolute joint
L(2) = Link([0 0 1 0 0],'R');
Robot = SerialLink(L, 'name', '2-R Planar Robot') % Concatenate the links as a serial robot

% Quintic polynomial interpolation for x coordinate
x_2c = tpoly(-1,2,100);
% Quintic polynomial interpolation for y coordinate
y_2c = tpoly(1,1,100);

% IK of 2R planar robot to find theta1 and theta2 for all (x,y) positions
% NOTE: Using only 1 (elbow down) of 2 possible solutions by initializing
%       'q0' values for each joint angle using IK solution from 2 - A.
for i=1:length(t)
    theta_2c(i,:) = Robot.ikine(transl(x_2c(i),y_2c(i), 0),'q0',[theta1_2a(i),theta2_2a(i)],'mask',[1,1,0,0,0,0]); % Mask vector (1x6 - xyzrpy) specifies the Cartesian DOF that will be ignored in reaching IK solution
end

%% Problem 2 - D %%
fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 2 - D\n');
fprintf('----------------------------------------------------------------\n\n');

% Robot definition using PCRTB (DH parametrization)
L(1) = Link([0 0 2 0 0],'R'); % First '0' doesn't mean 0. MATLAB takes it in as a variable 'theta' because we specify 'R', which defines a revolute joint
L(2) = Link([0 0 1 0 0],'R');
Robot = SerialLink(L, 'name', '2-R Planar Robot') % Concatenate the links as a serial robot

% Quintic polynomial interpolation for theta1 position
theta1_2d = jtraj(theta1_2b(1),theta1_2b(end),100);
% Quintic polynomial interpolation for theta2 position
theta2_2d = jtraj(theta2_2b(1),theta2_2b(end),100);

% FK of 2R planar robot to find (x,y) positions for all theta1 and theta2
for i=1:length(t)
    T_2d = Robot.fkine([theta1_2d,theta2_2d]);
    xyz_2d(:,i) = transl(T_2d(i));
end

%% Problem 2 - Plots and Animations %%

% x vs. t
figure('Name','x vs. t')
hold on
plot(t,x_2a)
plot(t,x_2b)
plot(t,x_2c)
plot(t,xyz_2d(1,:))
xlabel('Time (s)')
ylabel('Position (x)')
legend('Solution 2A','Solution 2B','Solution 2C','Solution 2D','Location','NW')
hold off

% y vs. t
figure('Name','y vs. t')
hold on
plot(t,y_2a)
plot(t,y_2b)
plot(t,y_2c)
plot(t,xyz_2d(2,:))
xlabel('Time (s)')
ylabel('Position (y)')
legend('Solution 2A','Solution 2B','Solution 2C','Solution 2D','Location','NE')
hold off

% y vs. x
figure('Name','y vs. x')
hold on
plot(x_2a,y_2a)
plot(x_2b,y_2b)
plot(x_2c,y_2c)
plot(xyz_2d(1,:),xyz_2d(2,:))
xlabel('Position (x)')
ylabel('Position (y)')
legend('Solution 2A','Solution 2B','Solution 2C','Solution 2D','Location','NE')
hold off

% theta1 vs. t
figure('Name','theta1 vs. t')
hold on
plot(t,rad2deg(theta1_2a))
plot(t,rad2deg(theta1_2b))
plot(t,rad2deg(theta_2c(:,1)))
plot(t,rad2deg(theta1_2d))
xlabel('Time (s)')
ylabel('\theta_1 (deg)')
legend('Solution 2A','Solution 2B','Solution 2C','Solution 2D','Location','NE')
hold off

% theta2 vs. t
figure('Name','theta2 vs. t')
hold on
plot(t,rad2deg(theta2_2a))
plot(t,rad2deg(theta2_2b))
plot(t,rad2deg(theta_2c(:,2)))
plot(t,rad2deg(theta2_2d))
xlabel('Time (s)')
ylabel('\theta_2 (deg)')
legend('Solution 2A','Solution 2B','Solution 2C','Solution 2D','Location','NE')
hold off

% Robot animation (2-A)
figure('Name', 'Robot 2-A');
plot(x_2a,y_2a);
Robot.plot([theta1_2a', theta2_2a'],'fps',10); % Visuaize the robot
%Robot.plot([theta1_2a', theta2_2a'],'fps',10,'movie','Results\Problem 2\Animation 2-A.mp4'); % Save robot animation

% Robot animation (2-B)
figure('Name', 'Robot 2-B');
plot(x_2b,y_2b);
Robot.plot([theta1_2b', theta2_2b'],'fps',10); % Visuaize the robot
%Robot.plot([theta1_2b', theta2_2b'],'fps',10,'movie','Results\Problem 2\Animation 2-B.mp4'); % Save robot animation

% Robot animation (2-C)
figure('Name', 'Robot 2-C');
plot(x_2c,y_2c);
Robot.plot([theta_2c(:,1), theta_2c(:,2)],'fps',10); % Visuaize the robot
%Robot.plot([theta_2c(:,1), theta_2c(:,2)],'fps',10,'movie','Results\Problem 2\Animation 2-C.mp4'); % Save robot animation

% Robot animation (2-D)
figure('Name', 'Robot 2-D');
plot(xyz_2d(1,:),xyz_2d(2,:));
Robot.plot([theta1_2d, theta2_2d],'fps',10); % Visuaize the robot
%Robot.plot([theta1_2d, theta2_2d],'fps',10,'movie','Results\Problem 2\Animation 2-D.mp4'); % Save robot animation