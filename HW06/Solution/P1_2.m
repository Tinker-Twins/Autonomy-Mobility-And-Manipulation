%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 6: Jacobians (Related Design and Control Issues) %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti, Olamide Akinyele

close all;
clear;
clc;

fprintf('\n=========================================================================\n');
fprintf('JACOBIANS (RELATED DESIGN AND CONTROL ISSUES)\n');
fprintf('=========================================================================\n');

%% Problem 1-RP (i) %%

% Clear workspace to avoid variable overloading
clear;

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1-RP (i)\n');
fprintf('----------------------------------------------------------------\n\n');

% RP robot parameters
d_min = 0.5;
d_max = 1.5;

% Define workspace grid
i = 1;
for R = d_min:0.1:d_max
    for theta = 0:359
        grid_x(i) = R*cosd(theta);
        grid_y(i) = R*sind(theta);
        i = i+1;
    end
end

% Verify workspace grid
figure()
plot(grid_x,grid_y,'.')
title("Workspace Grid")
xlabel("Workspace X (m)")
ylabel("Workspace Y (m)")
xlim([-d_max-0.5,d_max+0.5])
ylim([-d_max-0.5,d_max+0.5])
pbaspect([1 1 1])

% Compute IK at each point in workspace grid
for i=1:length(grid_x)
    t1(i) = atan2(grid_y(i),grid_x(i));
    d2(i) = sqrt(grid_x(i)^2+grid_y(i)^2);
end

% Verify IK solution using FK
robot_x = d2.*cos(t1);
robot_y = d2.*sin(t1);
figure()
plot(robot_x,robot_y,'.')
title("IK-FK Verification Throughout Workspace Grid")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-d_max-0.5,d_max+0.5])
ylim([-d_max-0.5,d_max+0.5])
pbaspect([1 1 1])

% Analyse manipulability
for i=1:length(d2)
    % Compute Jacobian for each IK solution
    J(:,:,i) = [-d2(i)*sin(t1(i)) cos(t1(i));
               d2(i)*cos(t1(i))  sin(t1(i))];
    % Perform singular value decomposition (SVD) of Jacobian
    %[u s v] = svd(J(:,:,i));
    %U(:,:,i) = u;
    %S(:,:,i) = s;
    %V(:,:,i) = v;
    sigma(:,i) = svd(J(:,:,i));
    % Compute isotropy index as a measure of manipulability
    w_i(i) = sigma(2,i)/sigma(1,i);
end

% Plot manipulability surface
figure()
% Fit triangular mesh to X-Y workspace grid
tri=delaunay(robot_x,robot_y);
% Generate 3D plot by pinching and moving each X-Y tri-mesh vertex to the coRPesponding Z-height
trisurf(tri,robot_x,robot_y,w_i,'FaceAlpha',0.75,'LineStyle','None')
title("Manipulability Surface")
xlabel("Workspace X (m)")
ylabel("Workspace Y (m)")
zlabel("Isotropy Index, w_i")
xlim([-d_max-0.5,d_max+0.5])
ylim([-d_max-0.5,d_max+0.5])
zlim([0,1])
pbaspect([1 1 1])

fprintf("The minimum value of isotropy index is %.4f.\n", min(w_i))
t1_min = t1(find(w_i-min(w_i) <= 1e-6));
d2_min = d2(find(w_i-min(w_i) <= 1e-6));
figure()
plot(t1_min, '.')
hold on
plot(d2_min, '.')
title("Joint-Space Configurations at Minimum Isotropy Index")
xlabel("Index")
ylabel("Joint Variables")
legend("\theta_1 (rad)","d_2 (m)")
figure()
plot(d2_min.*cos(t1_min), d2_min.*sin(t1_min), '.')
title("Task-Space Configurations at Minimum Isotropy Index")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-d_max-0.5,d_max+0.5])
ylim([-d_max-0.5,d_max+0.5])
pbaspect([1 1 1])

fprintf("The maximum value of isotropy index is %.4f.\n", max(w_i))
t1_max = t1(find(max(w_i)-w_i <= 1e-6));
d2_max = d2(find(max(w_i)-w_i <= 1e-6));
figure()
plot(t1_max, '.')
hold on
plot(d2_max, '.')
title("Joint-Space Configurations at Maximum Isotropy Index")
xlabel("Index")
ylabel("Joint Variables")
legend("\theta_1 (rad)","d_2 (m)")
figure()
plot(d2_max.*cos(t1_max), d2_max.*sin(t1_max), '.')
title("Task-Space Configurations at Maximum Isotropy Index")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-d_max-0.5,d_max+0.5])
ylim([-d_max-0.5,d_max+0.5])
pbaspect([1 1 1])

%% Problem 1-RP (ii) %%

% Clear workspace to avoid variable overloading
clear;

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1-RP (ii)\n');
fprintf('----------------------------------------------------------------\n\n');

% RP robot parameters
d_min = 0.5;
d_max = 1.5;

% Define workspace grid
i = 1;
for R = d_min:0.1:d_max
    for theta = 0:359
        grid_x(i) = R*cosd(theta);
        grid_y(i) = R*sind(theta);
        i = i+1;
    end
end

% Verify workspace grid
figure()
plot(grid_x,grid_y,'.')
title("Workspace Grid")
xlabel("Workspace X (m)")
ylabel("Workspace Y (m)")
xlim([-d_max-0.5,d_max+0.5])
ylim([-d_max-0.5,d_max+0.5])
pbaspect([1 1 1])

% Compute IK at each point in workspace grid
for i=1:length(grid_x)
    t1(i) = atan2(grid_y(i),grid_x(i));
    d2(i) = sqrt(grid_x(i)^2+grid_y(i)^2);
end

% Verify IK solution using FK
robot_x = d2.*cos(t1);
robot_y = d2.*sin(t1);
figure()
plot(robot_x,robot_y,'.')
title("IK-FK Verification Throughout Workspace Grid")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-d_max-0.5,d_max+0.5])
ylim([-d_max-0.5,d_max+0.5])
pbaspect([1 1 1])

% Analyse manipulability
for i=1:length(d2)
    % Compute Jacobian for each IK solution
    J(:,:,i) = [-d2(i)*sin(t1(i)) cos(t1(i));
               d2(i)*cos(t1(i))  sin(t1(i))];
    % Perform singular value decomposition (SVD) of Jacobian
    [U S V] = svd(J(:,:,i));
    % Compute isotropy index as a measure of manipulability
    %w(i) = real(sqrt(det(J(:,:,i)*J(:,:,i)')));
    w(i) = det(S);
end

% Plot manipulability surface
figure()
% Fit triangular mesh to X-Y workspace grid
tri=delaunay(robot_x,robot_y);
% Generate 3D plot by pinching and moving each X-Y tri-mesh vertex to the coRPesponding Z-height
trisurf(tri,robot_x,robot_y,w,'FaceAlpha',0.75,'LineStyle','None')
title("Manipulability Surface")
xlabel("Workspace X (m)")
ylabel("Workspace Y (m)")
zlabel("Yoshikawa's MOM, w")
xlim([-d_max-0.5,d_max+0.5])
ylim([-d_max-0.5,d_max+0.5])
pbaspect([1 1 1])

fprintf("The minimum value of MOM is %.4f.\n", min(w))
t1_min = t1(find(w-min(w) <= 1e-6));
d2_min = d2(find(w-min(w) <= 1e-6));
figure()
plot(t1_min, '.')
hold on
plot(d2_min, '.')
title("Joint-Space Configurations at Minimum MOM")
xlabel("Index")
ylabel("Joint Variables")
legend("\theta_1 (rad)","d_2 (m)")
figure()
plot(d2_min.*cos(t1_min), d2_min.*sin(t1_min), '.')
title("Task-Space Configurations at Minimum MOM")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-d_max-0.5,d_max+0.5])
ylim([-d_max-0.5,d_max+0.5])
pbaspect([1 1 1])

fprintf("The maximum value of MOM is %.4f.\n", max(w))
t1_max = t1(find(max(w)-w <= 1e-6));
d2_max = d2(find(max(w)-w <= 1e-6));
figure()
plot(t1_max, '.')
hold on
plot(d2_max, '.')
title("Joint-Space Configurations at Maximum MOM")
xlabel("Index")
ylabel("Joint Variables")
legend("\theta_1 (rad)","d_2 (m)")
figure()
plot(d2_max.*cos(t1_max), d2_max.*sin(t1_max), '.')
title("Task-Space Configurations at Maximum MOM")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-d_max-0.5,d_max+0.5])
ylim([-d_max-0.5,d_max+0.5])
pbaspect([1 1 1])

%% Problem 1-RP (iii) %%

% Clear workspace to avoid variable overloading
clear;

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1-RP (iii)\n');
fprintf('----------------------------------------------------------------\n\n');

% RP robot parameters
d_min = 0.5;
d_max = 1.5;

% Define workspace grid
i = 1;
for R = d_min:0.5:d_max
    for theta = 0:15:359
        grid_x(i) = R*cosd(theta);
        grid_y(i) = R*sind(theta);
        i = i+1;
    end
end

% Verify workspace grid
figure()
plot(grid_x,grid_y,'.')
title("Workspace Grid")
xlabel("Workspace X (m)")
ylabel("Workspace Y (m)")
xlim([-d_max-0.5,d_max+0.5])
ylim([-d_max-0.5,d_max+0.5])
pbaspect([1 1 1])

% Compute IK at each point in workspace grid
for i=1:length(grid_x)
    t1(i) = atan2(grid_y(i),grid_x(i));
    d2(i) = sqrt(grid_x(i)^2+grid_y(i)^2);
end

% Verify IK solution using FK
robot_x = d2.*cos(t1);
robot_y = d2.*sin(t1);
figure()
plot(robot_x,robot_y,'.')
title("IK-FK Verification Throughout Workspace Grid")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-d_max-0.5,d_max+0.5])
ylim([-d_max-0.5,d_max+0.5])
pbaspect([1 1 1])

% Analyse manipulability
for i=1:length(d2)
    % Compute Jacobian for each IK solution
    J(:,:,i) = [-d2(i)*sin(t1(i)) cos(t1(i));
               d2(i)*cos(t1(i))  sin(t1(i))];
    % Perform singular value decomposition (SVD) of Jacobian
    [u s v] = svd(J(:,:,i));
    U(:,:,i) = u;
    S(:,:,i) = s;
    V(:,:,i) = v;
end

% Plot manipulability ellipsoids for each point in workspace grid
scale = 15;
fprintf("The scale for manipulability ellipsoids is 1:%d m/s.\n", scale)
figure()
for i=1:length(grid_x)
    t = -pi:0.01:pi;
    x0 = grid_x(i);
    y0 = grid_y(i);
    USV(:,:,i) = U(:,:,i)*(S(:,:,i)/scale)*V(:,:,i)';
    x = x0 + USV(1,1,i)*cos(t) + USV(1,2,i)*sin(t);
    y = y0 + USV(2,1,i)*cos(t) + USV(2,2,i)*sin(t);
    plot(x,y,'color',[0, 0.4470, 0.7410])
    hold on
    title("Manipulability Ellipsoids in Workspace")
    xlabel("Workspace X (m)")
    ylabel("Workspace Y (m)")
    xlim([-d_max-0.5,d_max+0.5])
    ylim([-d_max-0.5,d_max+0.5])
    pbaspect([1 1 1])
end
hold off