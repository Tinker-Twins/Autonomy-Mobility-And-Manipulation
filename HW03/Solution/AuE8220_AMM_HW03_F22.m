%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 2: Interpolation, Inverse Kinematics %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti, Olamide Akinyele

fprintf('\n=========================================================================\n');
fprintf('INTERPOLATION AND INVERSE KINEMATICS\n');
fprintf('=========================================================================\n');

%% Problem 1 - A %%

% fprintf('\n----------------------------------------------------------------\n');
% fprintf('Problem 1 - A\n');
% fprintf('----------------------------------------------------------------\n\n');
% 
% Rz11 = [cos(pi/4) -sin(pi/4) 0;
%         sin(pi/4) cos(pi/4)  0;
%         0        0           1];
% 
% Ry12 = [cos(pi/6)  0 sin(pi/6);
%         0          1 0        ;
%         -sin(pi/6) 0 cos(pi/6)];
% 
% Rz13 = [cos(pi/9) -sin(pi/9) 0;
%         sin(pi/9) cos(pi/9)  0;
%         0         0          1];
% 
% fprintf("Final orientation after relative Rzyz rotations:\n")
% Rzyz = Rz11*Ry12*Rz13;
% disp(Rzyz);

% THIS IS WRONG
% fprintf("Alternate soultions for absolute Rxyz to achieve the same orientation:\n")
% phi1 = atan2(Rzxz_abs(1,1), Rzxz_abs(2,1));
% phi2 = atan2(-Rzxz_abs(1,1), -Rzxz_abs(2,1));
% fprintf("phi = %.4f rad\tphi = %.4f rad\n", phi1, phi2)
% theta1 = atan2(-Rzxz_abs(3,1), sqrt(1+Rzxz_abs(3,1)^2));
% theta2 = atan2(-Rzxz_abs(3,1), -sqrt(1+Rzxz_abs(3,1)^2));
% fprintf("theta = %.4f rad\ttheta = %.4f rad\n", theta1, theta2)
% psi1 = atan2(Rzxz_abs(3,3), Rzxz_abs(3,2));
% psi2 = atan2(-Rzxz_abs(3,3), -Rzxz_abs(3,2));
% fprintf("psi = %.4f rad\tpsi = %.4f rad\n", psi1, psi2)

%% Problem 1 - C %%
% ZYZ relative rotation
Ri = rotz(0)*roty(0)*rotz(0);
Rf = rotz(rad2deg(pi/9))*roty(rad2deg(pi/6))*rotz(rad2deg(pi/4)); % Convert to absolute angles
% Convert to homogenous transformation
Ti = r2t(Ri);
Tf = r2t(Rf);
% Interpolate and plot
Tint = trinterp(Ti, Tf, 100);
figure('Name','Plot for 1-C')
for i=1:length(Tint)
    if mod(i,20)==0
        trplot(Tint(:,:,i), 'color','red')
        hold on
    end
end
hold off
% Animate
figure('Name','Animation for 1-C')
tranimate(Tint,'fps',10);

%% Problem 1 - D %%
% ZYZ relative rotation
Ri = rotz(0)*roty(0)*rotz(0);
Rf = rotz(rad2deg(pi/9))*roty(rad2deg(pi/6))*rotz(rad2deg(pi/4)); % Convert to absolute angles
% PRY absolute roll-pitch-yaw angles
RPYi = rad2deg(tr2rpy(Ri));
RPYf = rad2deg(tr2rpy(Rf));
% Interpolate and plot RPY absolute roll-pitch-yaw angles
RPY = jtraj(RPYi, RPYf, 100);
figure('Name','Plot for 1-D')
for i=1:length(RPY)
    if mod(i,20)==0
        trplot(rpy2tr(RPY(i,:)), 'color','green')
        hold on
    end
end
hold off
% Animate
figure('Name','Animation for 1-D')
tranimate(rpy2tr(RPY),'fps',10);