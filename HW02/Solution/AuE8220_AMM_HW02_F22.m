%% AuE 8220 - Autonomy: Mobility and Manipulation %%
% Homework 2: Rotation Matrices, Rotation Parameterizations, Homogenous Transformations %
% Authors: Tanmay Samak, Chinmay Samak, Riccardo Setti, Olamide Akinyele

fprintf('\n=========================================================================\n');
fprintf('ROTATION MATRICES, ROTATION PARAMETERIZATIONS, HOMOGENOUS TRANSFORMATIONS\n');
fprintf('=========================================================================\n');

%% Problem 1 %%

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1 (a)\n');
fprintf('----------------------------------------------------------------\n\n');

Rz11 = [cos(pi/3) -sin(pi/3) 0;
        sin(pi/3) cos(pi/3)  0;
        0        0           1];

Rx12 = [1 0         0         ;
        0 cos(pi/3) -sin(pi/3);
        0 sin(pi/3) cos(pi/3) ];

Rz13 = [cos(pi/4) -sin(pi/4) 0;
        sin(pi/4) cos(pi/4)  0;
        0         0          1];

fprintf("Final orientation after relative Rzxz rotations:\n")
Rzxz = Rz11*Rx12*Rz13;
disp(Rzxz);

fprintf("Alternate soultions for relative Rzyz to achieve the same orientation:\n")
phi1 = atan2(Rzxz(2,3), Rzxz(3,3));
phi2 = atan2(-Rzxz(2,3), -Rzxz(3,3));
fprintf("phi = %.4f rad\tphi = %.4f rad\n", phi1, phi2)
theta1 = atan2(sqrt(1-Rzxz(3,3)^2), Rzxz(3,3));
theta2 = atan2(-sqrt(1-Rzxz(3,3)^2), Rzxz(3,3));
fprintf("theta = %.4f rad\ttheta = %.4f rad\n", theta1, theta2)
psi1 = atan2(Rzxz(3,2), -Rzxz(3,1));
psi2 = atan2(-Rzxz(3,2), Rzxz(3,1));
fprintf("psi = %.4f rad\tpsi = %.4f rad\n", psi1, psi2)

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1 (b)\n');
fprintf('----------------------------------------------------------------\n\n');

fprintf("Final orientation after absolute Rzxz rotations:\n")
Rzxz_abs = Rz13*Rx12*Rz11;
disp(Rzxz_abs);

fprintf("Alternate soultions for absolute Rxyz to achieve the same orientation:\n")
phi1 = atan2(Rzxz_abs(1,1), Rzxz_abs(2,1));
phi2 = atan2(-Rzxz_abs(1,1), -Rzxz_abs(2,1));
fprintf("phi = %.4f rad\tphi = %.4f rad\n", phi1, phi2)
theta1 = atan2(-Rzxz_abs(3,1), sqrt(1+Rzxz_abs(3,1)^2));
theta2 = atan2(-Rzxz_abs(3,1), -sqrt(1+Rzxz_abs(3,1)^2));
fprintf("theta = %.4f rad\ttheta = %.4f rad\n", theta1, theta2)
psi1 = atan2(Rzxz_abs(3,3), Rzxz_abs(3,2));
psi2 = atan2(-Rzxz_abs(3,3), -Rzxz_abs(3,2));
fprintf("psi = %.4f rad\tpsi = %.4f rad\n", psi1, psi2)

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 1 (c)\n');
fprintf('----------------------------------------------------------------\n\n');

fprintf("Final orientation after absolute Rzxz rotations:\n")
disp(Rzxz_abs);

fprintf("Alternate axis/angle representation of the same orientation:\n")
theta = acos((Rzxz_abs(1,1)+Rzxz_abs(2,2)+Rzxz_abs(3,3)-1)/2);
fprintf("Angle: %.4f rad\n", theta)
k = 1/(2*sin(theta)) * [Rzxz_abs(3,2)-Rzxz_abs(2,3);
                        Rzxz_abs(1,3)-Rzxz_abs(3,1);
                        Rzxz_abs(2,1)-Rzxz_abs(1,2)];
fprintf("Axis:\n"); disp(k);

%% Problem 2 %%

Ai = [0 1 1 0 0 1 1 0;
      0 0 1 1 0 0 1 1;
      0 0 0 0 1 1 1 1];

Af = [0.5000 0.9698  1.0805  0.6107  1.3758 1.8456  1.9563  1.4865;
      0      -0.8660 -0.6160 0.2500  0.4330 -0.4330 -0.1830 0.6830;
      0      -0.1710 -1.1329 -0.9619 0.2133 0.0423  -0.9196 -0.7486];

Ai = [Ai; 1 1 1 1 1 1 1 1];
Af = [Af; 1 1 1 1 1 1 1 1];

Af*pinv(Ai) % Also works for H = mrdivide(Af, Ai)

%% Problem 3 %%

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 3\n');
fprintf('----------------------------------------------------------------\n\n');

syms theta psi real

R = [cos(theta)*cos(psi),  -cos(theta)*sin(psi), -sin(theta);
     sin(psi),             cos(psi),             0          ;
     -sin(theta)*cos(psi), -sin(theta)*sin(psi), cos(theta) ];

R_D = simplify(det(R))
R_T = simplify(R')
R_R_T = simplify(simplify(R) * R_T)
if R_D == sym(1)
    if isequaln(R_R_T, sym(eye(3)))
        fprintf("R is a generic rotation matrix!\n")
    else
        fprintf("R is NOT a generic rotation matrix!\n")
    end
else
    fprintf("R is NOT a generic rotation matrix!\n")
end

%% Problem 4

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 4\n');
fprintf('----------------------------------------------------------------\n\n');

syms r11 r13 r22 r23 r31 r32 real

F_R_M = [r11       1/sqrt(2) r13;
         1/sqrt(2) r22       r23;
         r31       r32       0];

[r11, r13, r22, r23, r31, r32] = solve(F_R_M * F_R_M' == eye(3));
if isempty(r11)
    fprintf("No fesible solution found to construct a rotation matrix!\n")
else
    for k=1:length(r11)
        F_R_M = [r11(k)    1/sqrt(2) r13(k);
                 1/sqrt(2) r22(k)    r23(k);
                 r31(k)    r32(k)    0     ];
        if det(F_R_M) == 1
            disp(F_R_M)
        else
            fprintf("No fesible solution found to construct a rotation matrix!\n")
        end
    end
end

%% Problem 5

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 5 (a)\n');
fprintf('----------------------------------------------------------------\n\n');

A_0_1 = transl(-3.5,3,1) * trotx(180)
A_1_2 = transl(0,3,1) * trotx(-90) * troty(90)
A_2_3 = transl(0,5,0) * trotx(-90) * trotz(-90) * trotx(-53.13)
A_3_4 = transl(3.5,0,0) * troty(180)

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 5 (b)\n');
fprintf('----------------------------------------------------------------\n\n');

A_0_1 = A_0_1
A_0_2 = A_0_1 * A_1_2
A_0_3 = A_0_2 * A_2_3
A_0_4 = A_0_3 * A_3_4

%% Problem 6

fprintf('\n----------------------------------------------------------------\n');
fprintf('Problem 6\n');
fprintf('----------------------------------------------------------------\n\n');

syms phi ux uy uz real

u = [ux/(sqrt(ux^2+uy^2+uz^2)); uy/(sqrt(ux^2+uy^2+uz^2)); uz/(sqrt(ux^2+uy^2+uz^2))];
U = [0   -u(3) u(2) ;
     u(3)  0   -u(1);
     -u(2) u(1)  0   ];
R = eye(3)*cos(phi) + u*u'*(1-cos(phi)) + U*sin(phi);

R_D = simplify(det(R))
R_T = simplify(R');
R_R_T = simplify(simplify(R) * R_T)
if R_D == sym(1)
    if isequaln(R_R_T, sym(eye(3)))
        fprintf("R is a rotation matrix!\n")
    else
        fprintf("R is NOT a rotation matrix!\n")
    end
else
    fprintf("R is NOT a rotation matrix!\n")
end