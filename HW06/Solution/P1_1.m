%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 6: Jacobians (Related Design and Control Issues) %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti, Olamide Akinyele

close all;
clear;
clc;

fprintf('\n=========================================================================\n');
fprintf('JACOBIANS (RELATED DESIGN AND CONTROL ISSUES)\n');
fprintf('=========================================================================\n');

%% Problem 1-RR (i) %%

% Clear workspace to avoid variable overloading
clear;

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1-RR (i)\n');
fprintf('----------------------------------------------------------------\n\n');

% RR robot parameters
a1 = 3;
a2 = 2;

% Define workspace grid
i = 1;
for R = (a1-a2):0.1:(a1+a2)
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
xlim([-(a1+a2)-1,(a1+a2)+1])
ylim([-(a1+a2)-1,(a1+a2)+1])
pbaspect([1 1 1])

% Compute IK (elbow up) at each point in workspace grid
for i=1:length(grid_x)
    theta2(i) = real(-acos((grid_x(i)^2+grid_y(i)^2-a1^2-a2^2)/(2*a1*a2)));
    theta1(i) = atan2(grid_y(i),grid_x(i)) - atan2((a2*sin(theta2(i))),(a1+a2*cos(theta2(i))));
end

% Verify IK solution using FK
robot_x = a1*cos(theta1) + a2*cos(theta1+theta2);
robot_y = a1*sin(theta1) + a2*sin(theta1+theta2);
figure()
plot(robot_x,robot_y,'.')
title("IK-FK Verification Throughout Workspace Grid")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-(a1+a2)-1,(a1+a2)+1])
ylim([-(a1+a2)-1,(a1+a2)+1])
pbaspect([1 1 1])

% Analyse manipulability
for i=1:length(theta1)
    % Compute Jacobian for each IK solution
    J(:,:,i) = [-a1*sin(theta1(i)) -a2*sin(theta1(i)+theta2(i));
               a1*cos(theta1(i))  a2*cos(theta1(i)+theta2(i))];
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
% Generate 3D plot by pinching and moving each X-Y tri-mesh vertex to the corresponding Z-height
trisurf(tri,robot_x,robot_y,w_i,'FaceAlpha',0.75,'LineStyle','None')
title("Manipulability Surface")
xlabel("Workspace X (m)")
ylabel("Workspace Y (m)")
zlabel("Isotropy Index, w_i")
xlim([-(a1+a2)-1,(a1+a2)+1])
ylim([-(a1+a2)-1,(a1+a2)+1])
zlim([0,1])
pbaspect([1 1 1])

fprintf("The minimum value of isotropy index is %.4f.\n", min(w_i))
t1_min = theta1(find(w_i-min(w_i) <= 1e-6));
t2_min = theta2(find(w_i-min(w_i) <= 1e-6));
figure()
plot(t1_min, '.')
hold on
plot(t2_min, '.')
title("Joint-Space Configurations at Minimum Isotropy Index")
xlabel("Index")
ylabel("Joint Variables")
legend("\theta_1 (rad)","\theta_2 (rad)")
figure()
plot(a1*cos(t1_min) + a2*cos(t1_min+t2_min), a1*sin(t1_min) + a2*sin(t1_min+t2_min), '.')
title("Task-Space Configurations at Minimum Isotropy Index")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-(a1+a2)-1,(a1+a2)+1])
ylim([-(a1+a2)-1,(a1+a2)+1])
pbaspect([1 1 1])

fprintf("The maximum value of isotropy index is %.4f.\n", max(w_i))
t1_max = theta1(find(max(w_i)-w_i <= 1e-6));
t2_max = theta2(find(max(w_i)-w_i <= 1e-6));
figure()
plot(t1_max, '.')
hold on
plot(t2_max, '.')
title("Joint-Space Configurations at Maximum Isotropy Index")
xlabel("Index")
ylabel("Joint Variables")
legend("\theta_1 (rad)","\theta_2 (rad)")
figure()
plot(a1*cos(t1_max) + a2*cos(t1_max+t2_max), a1*sin(t1_max) + a2*sin(t1_max+t2_max), '.')
title("Task-Space Configurations at Maximum Isotropy Index")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-(a1+a2)-1,(a1+a2)+1])
ylim([-(a1+a2)-1,(a1+a2)+1])
pbaspect([1 1 1])

%% Problem 1-RR (ii) %%

% Clear workspace to avoid variable overloading
clear;

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1-RR (ii)\n');
fprintf('----------------------------------------------------------------\n\n');

% RR robot parameters
a1 = 3;
a2 = 2;

% Define workspace grid
i = 1;
for R = (a1-a2):0.1:(a1+a2)
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
xlim([-(a1+a2)-1,(a1+a2)+1])
ylim([-(a1+a2)-1,(a1+a2)+1])
pbaspect([1 1 1])

% Compute IK (elbow up) at each point in workspace grid
for i=1:length(grid_x)
    theta2(i) = real(-acos((grid_x(i)^2+grid_y(i)^2-a1^2-a2^2)/(2*a1*a2)));
    theta1(i) = atan2(grid_y(i),grid_x(i)) - atan2((a2*sin(theta2(i))),(a1+a2*cos(theta2(i))));
end

% Verify IK solution using FK
robot_x = a1*cos(theta1) + a2*cos(theta1+theta2);
robot_y = a1*sin(theta1) + a2*sin(theta1+theta2);
figure()
plot(robot_x,robot_y,'.')
title("IK-FK Verification Throughout Workspace Grid")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-(a1+a2)-1,(a1+a2)+1])
ylim([-(a1+a2)-1,(a1+a2)+1])
pbaspect([1 1 1])

% Analyse manipulability
for i=1:length(theta1)
    % Compute Jacobian for each IK solution
    J(:,:,i) = [-a1*sin(theta1(i)) -a2*sin(theta1(i)+theta2(i));
               a1*cos(theta1(i))  a2*cos(theta1(i)+theta2(i))];
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
% Generate 3D plot by pinching and moving each X-Y tri-mesh vertex to the corresponding Z-height
trisurf(tri,robot_x,robot_y,w,'FaceAlpha',0.75,'LineStyle','None')
title("Manipulability Surface")
xlabel("Workspace X (m)")
ylabel("Workspace Y (m)")
zlabel("Yoshikawa's MOM, w")
xlim([-(a1+a2)-1,(a1+a2)+1])
ylim([-(a1+a2)-1,(a1+a2)+1])
pbaspect([1 1 1])

fprintf("The minimum value of MOM is %.4f.\n", min(w))
t1_min = theta1(find(w-min(w) <= 1e-6));
t2_min = theta2(find(w-min(w) <= 1e-6));
figure()
plot(t1_min, '.')
hold on
plot(t2_min, '.')
title("Joint-Space Configurations at Minimum MOM")
xlabel("Index")
ylabel("Joint Variables")
legend("\theta_1 (rad)","\theta_2 (rad)")
figure()
plot(a1*cos(t1_min) + a2*cos(t1_min+t2_min), a1*sin(t1_min) + a2*sin(t1_min+t2_min), '.')
title("Task-Space Configurations at Minimum MOM")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-(a1+a2)-1,(a1+a2)+1])
ylim([-(a1+a2)-1,(a1+a2)+1])
pbaspect([1 1 1])

fprintf("The maximum value of MOM is %.4f.\n", max(w))
t1_max = theta1(find(max(w)-w <= 1e-6));
t2_max = theta2(find(max(w)-w <= 1e-6));
figure()
plot(t1_max, '.')
hold on
plot(t2_max, '.')
title("Joint-Space Configurations at Maximum MOM")
xlabel("Index")
ylabel("Joint Variables")
legend("\theta_1 (rad)","\theta_2 (rad)")
figure()
plot(a1*cos(t1_max) + a2*cos(t1_max+t2_max), a1*sin(t1_max) + a2*sin(t1_max+t2_max), '.')
title("Task-Space Configurations at Maximum MOM")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-(a1+a2)-1,(a1+a2)+1])
ylim([-(a1+a2)-1,(a1+a2)+1])
pbaspect([1 1 1])

%% Problem 1-RR (iii) %%

% Clear workspace to avoid variable overloading
clear;

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1-RR (iii)\n');
fprintf('----------------------------------------------------------------\n\n');

% RR robot parameters
a1 = 3;
a2 = 2;

% Define sparse workspace grid
i = 1;
for R = (a1-a2):(a1+a2)
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
xlim([-(a1+a2)-1,(a1+a2)+1])
ylim([-(a1+a2)-1,(a1+a2)+1])
pbaspect([1 1 1])

% Compute IK (elbow up) at each point in workspace grid
for i=1:length(grid_x)
    theta2(i) = real(-acos((grid_x(i)^2+grid_y(i)^2-a1^2-a2^2)/(2*a1*a2)));
    theta1(i) = atan2(grid_y(i),grid_x(i)) - atan2((a2*sin(theta2(i))),(a1+a2*cos(theta2(i))));
end

% Verify IK solution using FK
robot_x = a1*cos(theta1) + a2*cos(theta1+theta2);
robot_y = a1*sin(theta1) + a2*sin(theta1+theta2);
figure()
plot(robot_x,robot_y,'.')
title("IK-FK Verification Throughout Workspace Grid")
xlabel("End-Effector X (m)")
ylabel("End-Effector Y (m)")
xlim([-(a1+a2)-1,(a1+a2)+1])
ylim([-(a1+a2)-1,(a1+a2)+1])
pbaspect([1 1 1])

% Analyse manipulability
for i=1:length(theta1)
    % Compute Jacobian for each IK solution
    J(:,:,i) = [-a1*sin(theta1(i)) -a2*sin(theta1(i)+theta2(i));
               a1*cos(theta1(i))  a2*cos(theta1(i)+theta2(i))];
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
    xlim([-(a1+a2)-1,(a1+a2)+1])
    ylim([-(a1+a2)-1,(a1+a2)+1])
    pbaspect([1 1 1])
end

% %% Problem 1-RR (iii) with Elbow-Down Configuration (JUST FOR VERIFICATION) %%
% 
% % Clear workspace to avoid variable overloading
% clear;
% 
% fprintf('\n----------------------------------------------------------------\n');
% fprintf('Problem 1-RR (iii) with Elbow-Down Configuration\n');
% fprintf('----------------------------------------------------------------\n\n');
% 
% % RR robot parameters
% a1 = 3;
% a2 = 2;
% 
% % Define sparse workspace grid
% i = 1;
% for R = (a1-a2):(a1+a2)
%     for theta = 0:15:359
%         grid_x(i) = R*cosd(theta);
%         grid_y(i) = R*sind(theta);
%         i = i+1;
%     end
% end
% 
% % Verify workspace grid
% % figure()
% % plot(grid_x,grid_y,'.')
% % title("Workspace Grid")
% % xlabel("Workspace X (m)")
% % ylabel("Workspace Y (m)")
% % xlim([-(a1+a2)-1,(a1+a2)+1])
% % ylim([-(a1+a2)-1,(a1+a2)+1])
% % pbaspect([1 1 1])
% 
% % Compute IK (elbow down) at each point in workspace grid
% for i=1:length(grid_x)
%     theta2(i) = real(acos((grid_x(i)^2+grid_y(i)^2-a1^2-a2^2)/(2*a1*a2)));
%     theta1(i) = atan2(grid_y(i),grid_x(i)) - atan2((a2*sin(theta2(i))),(a1+a2*cos(theta2(i))));
% end
% 
% % Verify IK solution using FK
% % robot_x = a1*cos(theta1) + a2*cos(theta1+theta2);
% % robot_y = a1*sin(theta1) + a2*sin(theta1+theta2);
% % figure()
% % plot(robot_x,robot_y,'.')
% % title("IK-FK Verification Throughout Workspace Grid")
% % xlabel("End-Effector X (m)")
% % ylabel("End-Effector Y (m)")
% % xlim([-(a1+a2)-1,(a1+a2)+1])
% % ylim([-(a1+a2)-1,(a1+a2)+1])
% % pbaspect([1 1 1])
% 
% % Analyse manipulability
% for i=1:length(theta1)
%     % Compute Jacobian for each IK solution
%     J(:,:,i) = [-a1*sin(theta1(i)) -a2*sin(theta1(i)+theta2(i));
%                a1*cos(theta1(i))  a2*cos(theta1(i)+theta2(i))];
%     % Perform singular value decomposition (SVD) of Jacobian
%     [u s v] = svd(J(:,:,i));
%     U(:,:,i) = u;
%     S(:,:,i) = s;
%     V(:,:,i) = v;
% end
% 
% % Plot manipulability ellipsoids for each point in workspace grid
% scale = 15;
% fprintf("The scale for manipulability ellipsoids is 1:%d m/s.\n", scale)
% figure()
% for i=1:length(grid_x)
%     t = -pi:0.01:pi;
%     x0 = grid_x(i);
%     y0 = grid_y(i);
%     USV(:,:,i) = U(:,:,i)*(S(:,:,i)/scale)*V(:,:,i)';
%     x = x0 + USV(1,1,i)*cos(t) + USV(1,2,i)*sin(t);
%     y = y0 + USV(2,1,i)*cos(t) + USV(2,2,i)*sin(t);
%     plot(x,y,'color',[0.8500, 0.3250, 0.0980])
%     hold on
%     title("Manipulability Ellipsoids in Workspace")
%     xlabel("Workspace X (m)")
%     ylabel("Workspace Y (m)")
%     xlim([-(a1+a2)-1,(a1+a2)+1])
%     ylim([-(a1+a2)-1,(a1+a2)+1])
%     pbaspect([1 1 1])
% end
hold off