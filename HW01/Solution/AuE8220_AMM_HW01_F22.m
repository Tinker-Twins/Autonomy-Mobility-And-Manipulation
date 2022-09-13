%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 1: Fourbar Position Analyses: Method of Circles, Method of Loop Closure, Numerical Method %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti

%% Problem 1 %%

fprintf('\n================================================================\n');
fprintf('FOURBAR POSITION ANALYSES\n');
fprintf('================================================================\n');

r1 = 4.0; % Ground Link Length (m)
r2 = 2.0; % Crank Link Length (m)
r3 = 3.0; % Coupler Link Length (m)
r4 = 6.0; % Follower Link Length (m)
r5 = 5.0; % Coupler Link Length (m)
t1 = 30; % Ground Link Angle (deg)
t2 = 0:1:359; % Crank Angles (deg)
psi = 30; % Coupler Link Angle (deg)

%% Problem 1 - A %%

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1 - A | Method of Circles\n');
fprintf('----------------------------------------------------------------\n\n');

% Initialize
[t3,t4,Cx,Cy] = deal(zeros(1, 360));

% Solve
fprintf('Solving for uncrossed configuration...\n');
for i = 1:360
    fprintf('theta_2 = %.3d\t', i-1);
    [t3(i),t4(i),Cx(i),Cy(i)] = MethodOfCircles(r1,r2,r3,r4,r5,t1,t2(i),psi,"uncrossed");
    fprintf('theta_3 = %.4f\ttheta_4 = %.4f\n', t3(i), t4(i));
end

% Plot
figure(1)
plot(t2, t3)
title('Fourbar Position Analysis: Method of Circles')
subtitle('\theta_3 vs. \theta_2')
xlabel('\theta_2 (deg)')
ylabel('\theta_3 (deg)')
figure(2)
plot(t2, t4)
title('Fourbar Position Analysis: Method of Circles')
subtitle('\theta_4 vs. \theta_2')
xlabel('\theta_2 (deg)')
ylabel('\theta_4 (deg)')
figure(3)
plot(Cx, Cy)
title('Fourbar Position Analysis: Method of Circles')
subtitle('C_y vs. C_x')
xlabel('C_x (m)')
ylabel('C_y (m)')

% Store variables for comparison
t3_a = t3; t4_a = t4; Cx_a = Cx; Cy_a = Cy;

%% Problem 1 - B %%

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1 - B | Method of Loop Closure\n');
fprintf('----------------------------------------------------------------\n\n');

% Initialize
[t3,t4,Cx,Cy] = deal(zeros(1, 360));

% Solve
fprintf('Solving for crossed configuration...\n');
for i = 1:360
    fprintf('theta_2 = %.3d\t', i-1);
    [t3(i),t4(i),Cx(i),Cy(i)] = MethodOfLoopClosure(r1,r2,r3,r4,r5,t1,t2(i),psi,"crossed");
    fprintf('theta_3 = %.4f\ttheta_4 = %.4f\n', t3(i), t4(i));
end

% Plot
figure(4)
plot(t2, t3)
title('Fourbar Position Analysis: Method of Loop Closure')
subtitle('\theta_3 vs. \theta_2')
xlabel('\theta_2 (deg)')
ylabel('\theta_3 (deg)')
figure(5)
plot(t2, t4)
title('Fourbar Position Analysis: Method of Loop Closure')
subtitle('\theta_4 vs. \theta_2')
xlabel('\theta_2 (deg)')
ylabel('\theta_4 (deg)')
figure(6)
plot(Cx, Cy)
title('Fourbar Position Analysis: Method of Loop Closure')
subtitle('C_y vs. C_x')
xlabel('C_x (m)')
ylabel('C_y (m)')

% Store variables for comparison
t3_b = t3; t4_b = t4; Cx_b = Cx; Cy_b = Cy;

%% Problem 1 - C-1 %%

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1 - C | Newton-Raphson Method\n');
fprintf('----------------------------------------------------------------\n\n');

% Initialize
t3_init = t3_a + (5*(-1+2*rand(1,1))); % Analytical solution +/- random noise [-5, 5]
t4_init = t4_a + (5*(-1+2*rand(1,1))); % Analytical solution +/- random noise [-5, 5]
t3_init_c1 = t3_init; % Store variable for benchmarking in Problem 1 - C-2
t4_init_c1 = t4_init; % Store variable for benchmarking in Problem 1 - C-2
[t3,t4,Cx,Cy] = deal(zeros(1, 360));

% Solve
fprintf('Solving for uncrossed configuration...\n');
for i = 1:360
    fprintf('theta_2 = %.3d\t', i-1);
    [t3(i),t4(i),Cx(i),Cy(i)] = NewtonRaphsonMethod(r1,r2,r3,r4,r5,t1,t2(i),t3_init(i),t4_init(i),psi);
    fprintf('theta_3 = %.4f\ttheta_4 = %.4f\n', t3(i), t4(i));
end

% Plot
figure(7)
plot(t2, t3)
title('Fourbar Position Analysis: Newton-Raphson Method')
subtitle('\theta_3 vs. \theta_2')
xlabel('\theta_2 (deg)')
ylabel('\theta_3 (deg)')
figure(8)
plot(t2, t4)
title('Fourbar Position Analysis: Newton-Raphson Method')
subtitle('\theta_4 vs. \theta_2')
xlabel('\theta_2 (deg)')
ylabel('\theta_4 (deg)')
figure(9)
plot(Cx, Cy)
title('Fourbar Position Analysis: Newton-Raphson Method')
subtitle('C_y vs. C_x')
xlabel('C_x (m)')
ylabel('C_y (m)')

figure(10)
plot(t3_a-t3)
title('Fourbar Position Analysis: Newton-Raphson Method')
subtitle('Error in \theta_3')
xlabel('\theta_2 (deg)')
ylabel('\theta_3_{ analytical} - \theta_3_{ numerical} (deg)')
figure(11)
plot(t4_a-t4)
title('Fourbar Position Analysis: Newton-Raphson Method')
subtitle('Error in \theta_4')
xlabel('\theta_2 (deg)')
ylabel('\theta_4_{ analytical} - \theta_4_{ numerical} (deg)')

%% Problem 1 - C-2 %%

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1 - C | MATLAB FSOLVE Method\n');
fprintf('----------------------------------------------------------------\n\n');

% Initialize
t3_init = t3_init_c1; % Use same initial value for t3 as in case of Problem 1 - C-1
t4_init = t4_init_c1; % Use same initial value for t4 as in case of Problem 1 - C-1
[t3,t4,Cx,Cy] = deal(zeros(1, 360));

% Solve
fprintf('Solving for uncrossed configuration...\n');
for i = 1:360
    fprintf('theta_2 = %.3d\t', i-1);
    [t3(i),t4(i),Cx(i),Cy(i)] = FSOLVEMethod(r1,r2,r3,r4,r5,t1,t2(i),t3_init(i),t4_init(i),psi);
    fprintf('theta_3 = %.4f\ttheta_4 = %.4f\n', t3(i), t4(i));
end

% Plot
figure(12)
plot(t2, t3)
title('Fourbar Position Analysis: FSOLVE Method')
subtitle('\theta_3 vs. \theta_2')
xlabel('\theta_2 (deg)')
ylabel('\theta_3 (deg)')
figure(13)
plot(t2, t4)
title('Fourbar Position Analysis: FSOLVE Method')
subtitle('\theta_4 vs. \theta_2')
xlabel('\theta_2 (deg)')
ylabel('\theta_4 (deg)')
figure(14)
plot(Cx, Cy)
title('Fourbar Position Analysis: FSOLVE Method')
subtitle('C_y vs. C_x')
xlabel('C_x (m)')
ylabel('C_y (m)')

figure(15)
plot(t3_a-t3)
title('Fourbar Position Analysis: FSOLVE Method')
subtitle('Error in \theta_3')
xlabel('\theta_2 (deg)')
ylabel('\theta_3_{ analytical} - \theta_3_{ numerical} (deg)')
figure(16)
plot(t4_a-t4)
title('Fourbar Position Analysis: FSOLVE Method')
subtitle('Error in \theta_4')
xlabel('\theta_2 (deg)')
ylabel('\theta_4_{ analytical} - \theta_4_{ numerical} (deg)')

%% Helper Functions

% Analytical Approach: Method of Circles

function [t3,t4,Cx,Cy] = MethodOfCircles(r1,r2,r3,r4,r5,t1,t2,psi,config)
    
    % Given the link lengths (r1, r2, r3, r4, r5) and link angles (t1, t2,
    % psi), solves for the remaining link angles (t3, t4) and positional
    % coordinates of point of contact on coupler (Cx, Cy) for the given
    % configuration (uncrossed/crossed), using the method of circles.
    
    % Compute positional coordinates of points A & D
    Ax = r2*cosd(t2); Ay = r2*sind(t2);
    Dx = r1*cosd(t1); Dy = r1*sind(t1);
    % Compute intermediate terms (refer manual derivation)
    k1 = (r3^2 - r4^2 + r1^2 - r2^2)/(2 * (Dx - Ax));
    k2 = (2 * (Ay - Dy))/(2 * (Dx - Ax));
    k3 = k1 - Dx;
    P = k2^2 + 1;
    Q = 2*k2*k3 - 2*Dy;
    R = k3^2 + Dy^2 - r4^2;
    % Only solve for feasible (real) solutions
    if Q^2-(4*P*R) >= 0
        % Compute By
        By_1 = (-Q+sqrt(Q^2-(4*P*R)))/(2*P);
        By_2 = (-Q-sqrt(Q^2-(4*P*R)))/(2*P);
        % Compute Bx
        Bx_1 = k1+k2*By_1;
        Bx_2 = k1+k2*By_2;
        % Compute t3
        t3_1 = atan2d(By_1-Ay, Bx_1-Ax);
        if t3_1 < 0
            t3_1 = t3_1+360; % Change angle representation from [-180,-0] to [180,360]
        end
        t3_2 = atan2d(By_2-Ay, Bx_2-Ax);
        if t3_2 < 0
            t3_2 = t3_2+360; % Change angle representation from [-180,-0] to [180,360]
        end
        % Compute t4
        t4_1 = atan2d(By_1-Dy, Bx_1-Dx);
        if t4_1 < 0
            t4_1 = t4_1+360; % Change angle representation from [-180,-0] to [180,360]
        end
        t4_2 = atan2d(By_2-Dy, Bx_2-Dx);
        if t4_2 < 0
            t4_2 = t4_2+360; % Change angle representation from [-180,-0] to [180,360]
        end
        % Compute t5
        t5_1 = t3_1 + psi;
        t5_2 = t3_2 + psi;
        % Compute Cx
        Cx_1 = Ax + r5*cosd(t5_1);
        Cx_2 = Ax + r5*cosd(t5_2);
        % Compute Cy
        Cy_1 = Ay + r5*sind(t5_1);
        Cy_2 = Ay + r5*sind(t5_2);
        % Compute gamma for one set (the other set gives mirror of this)
        gamma_1 = t4_1 - t3_1;
        % Check for crossed and uncrossed configurations
        if gamma_1 > 0
            % Uncrossed configuration
            t3_u = t3_1;
            t4_u = t4_1;
            Cx_u = Cx_1;
            Cy_u = Cy_1;
            % Crossed configuration
            t3_c = t3_2;
            t4_c = t4_2;
            Cx_c = Cx_2;
            Cy_c = Cy_2;
        else
            % Uncrossed configuration
            t3_u = t3_2;
            t4_u = t4_2;
            Cx_u = Cx_2;
            Cy_u = Cy_2;
            % Crossed configuration
            t3_c = t3_1;
            t4_c = t4_1;
            Cx_c = Cx_1;
            Cy_c = Cy_1;
        end
        % Choose what values to return based on given config
        if config == "uncrossed"
            % Uncrossed configuration
            t3 = t3_u;
            t4 = t4_u;
            Cx = Cx_u;
            Cy = Cy_u;
        elseif config == "crossed"
            % Crossed configuration
            t3 = t3_c;
            t4 = t4_c;
            Cx = Cx_c;
            Cy = Cy_c;
        end
    else
        [t3,t4,Cx,Cy] = deal(inf);
    end
end

% Analytical Approach: Method of Loop Closure

function [t3,t4,Cx,Cy] = MethodOfLoopClosure(r1,r2,r3,r4,r5,t1,t2,psi,config)
    
    % Given the link lengths (r1, r2, r3, r4, r5) and link angles (t1, t2,
    % psi), solves for the remaining link angles (t3, t4) and positional
    % coordinates of point of contact on coupler (Cx, Cy) for the given
    % configuration (uncrossed/crossed), using the method of loop closure.
    
    % Compute positional coordinates of points A
    Ax = r2*cosd(t2); Ay = r2*sind(t2);
    % Compute intermediate terms (refer manual derivation)
    M = r1*cosd(t1) - r2*cosd(t2);
    N = r1*sind(t1) - r2*sind(t2);
    P = 2*M*r4;
    Q = 2*N*r4;
    R = M^2 + N^2 + r4^2 - r3^2;
    % Only solve for feasible (real) solutions
    if Q^2 - R^2 + P^2 >= 0
        % Compute t4
        t4_1 = 2*atan2d(-Q+sqrt(Q^2-R^2+P^2), R-P);
        if t4_1 < 0
            t4_1 = t4_1+360; % Change angle representation from [-180,-0] to [180,360]
        end
        t4_2 = 2*atan2d(-Q-sqrt(Q^2-R^2+P^2), R-P);
        if t4_2 < 0
            t4_2 = t4_2+360; % Change angle representation from [-180,-0] to [180,360]
        end
        % Compute t3
        t3_1 = atan2d(N+(r4*sind(t4_1)), M+(r4*cosd(t4_1)));
        if t3_1 < 0
            t3_1 = t3_1+360; % Change angle representation from [-180,-0] to [180,360]
        end
        t3_2 = atan2d(N+(r4*sind(t4_2)), M+(r4*cosd(t4_2)));
        if t3_2 < 0
            t3_2 = t3_2+360; % Change angle representation from [-180,-0] to [180,360]
        end
        % Compute t5
        t5_1 = t3_1 + psi;
        t5_2 = t3_2 + psi;
        % Compute Cx
        Cx_1 = Ax + r5*cosd(t5_1);
        Cx_2 = Ax + r5*cosd(t5_2);
        % Compute Cy
        Cy_1 = Ay + r5*sind(t5_1);
        Cy_2 = Ay + r5*sind(t5_2);
        % Compute gamma for one set (the other set gives mirror of this)
        gamma_1 = t4_1 - t3_1;
        % Check for crossed and uncrossed configurations
        if gamma_1 > 0
            % Uncrossed configuration
            t3_u = t3_1;
            t4_u = t4_1;
            Cx_u = Cx_1;
            Cy_u = Cy_1;
            % Crossed configuration
            t3_c = t3_2;
            t4_c = t4_2;
            Cx_c = Cx_2;
            Cy_c = Cy_2;
        else
            % Uncrossed configuration
            t3_u = t3_2;
            t4_u = t4_2;
            Cx_u = Cx_2;
            Cy_u = Cy_2;
            % Crossed configuration
            t3_c = t3_1;
            t4_c = t4_1;
            Cx_c = Cx_1;
            Cy_c = Cy_1;
        end
        % Choose what values to return based on given config
        if config == "uncrossed"
            % Uncrossed configuration
            t3 = t3_u;
            t4 = t4_u;
            Cx = Cx_u;
            Cy = Cy_u;
        elseif config == "crossed"
            % Crossed configuration
            t3 = t3_c;
            t4 = t4_c;
            Cx = Cx_c;
            Cy = Cy_c;
        end
    else
        [t3,t4,Cx,Cy] = deal(inf);
    end
end

% Numerical Approach: Newton-Raphson Method

function [t3,t4,Cx,Cy] = NewtonRaphsonMethod(r1,r2,r3,r4,r5,t1,t2,t3_init,t4_init,psi)
    
    % Given the link lengths (r1, r2, r3, r4, r5) and link angles (t1, t2,
    % psi), solves for the remaining link angles (t3, t4) and positional
    % coordinates of point of contact on coupler (Cx, Cy) for the given
    % initial guess of t3 and t4, using the Newton-Raphson method.
    
    % Compute positional coordinates of points A
    Ax = r2*cosd(t2); Ay = r2*sind(t2);
    % Initialize t3 and t4
    T = [t3_init; t4_init];
    % Initialize F
    F = [1; 1];
    % Numerical loop to compute t3 and t4
    while(norm(F) > 1e-10)
        % Formulate the function F and its partial derivative
        F = [r2*cosd(t2)+r3*cosd(T(1))-r1*cosd(t1)-r4*cosd(T(2)); r2*sind(t2)+r3*sind(T(1))-r1*sind(t1)-r4*sind(T(2))]; % Formulate the function F
        dFdt = [-r3*sind(T(1)), r4*sind(T(2)); r3*cosd(T(1)), -r4*cosd(T(2))]; % Compute partial derivative of F w.r.t. theta (i.e., dF/dt)
        dFdt_inv = pinv(dFdt); % Compute inverse (computing pseudo-inverse since dFdt matrix may become singular for certain cases)
        delta_t = -dFdt_inv * F; % Compute deltaTheta
        T = T + delta_t; % Update theta values (theta_k+1 = theta_k + deltaTheta)
    end
    % Converged solutions for t3 and t4
    t3 = T(1); t4 = T(2);
    % Compute t5
    t5 = t3 + psi;
    % Compute Cx
    Cx = Ax + r5*cosd(t5);
    % Compute Cy
    Cy = Ay + r5*sind(t5);
end

% Numerical Approach: MATLAB FSOLVE Method

function [t3,t4,Cx,Cy] = FSOLVEMethod(r1,r2,r3,r4,r5,t1,t2,t3_init,t4_init,psi)
    
    % Given the link lengths (r1, r2, r3, r4, r5) and link angles (t1, t2,
    % psi), solves for the remaining link angles (t3, t4) and positional
    % coordinates of point of contact on coupler (Cx, Cy) for the given
    % initial guess of t3 and t4, using MATLAB's FSOLVE method.
    
    % Compute positional coordinates of points A
    Ax = r2*cosd(t2); Ay = r2*sind(t2);
    % Initialize t3 and t4
    T = [t3_init, t4_init];
    if not(isequal(T, [Inf, Inf])) % Solve for feasible solutions only
        options = optimset('Display','off'); % Turn off fsolve output messages
        % Formulate the function F(x(1), x(2)) where x(1) = t3 and x(2) = t4
        F = @(x) [r2*cosd(t2)+r3*cosd(x(1))-r1*cosd(t1)-r4*cosd(x(2)); r2*sind(t2)+r3*sind(x(1))-r1*sind(t1)-r4*sind(x(2))];
        % Solve the function F numerically for values of x(1) (i.e. t3) and x(2) (i.e. t4) starting from [t3_init; t4_init]
        T = fsolve(F, T, options);
    end
    % Converged solutions for t3 and t4
    t3 = T(1); t4 = T(2);
    % Compute t5
    t5 = t3 + psi;
    % Compute Cx
    Cx = Ax + r5*cosd(t5);
    % Compute Cy
    Cy = Ay + r5*sind(t5);
end
