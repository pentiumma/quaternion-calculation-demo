clear
clc
addpath('functions')

% Quaternion calculation notes
     
% Define three coordinate systems (CS):
% A: x - East, y - North, z - Up
% B: x - West, y - South, z - Up   (A rotate around its z axis with 180 degree)
% C: x - Down, y - North, z - East (A rotate around its y axis with 90  degree)

% 1. Quaternion construction
% Rotation from A to B:
r1_A = [0.0 0.0 1.0];
theta1 = D2R(90.0);
% Rotation from A to C:
r2_A = [0.0 1.0 0.0];
theta2 = D2R(90.0);

q_A_B = axangle2quat(r1_A,theta1)
q_A_C = axangle2quat(r2_A,theta2)

% 2. Vector coordinate transformation
xA_A = [1.0 0.0 0.0];
xA_B = quatrotate(xA_A,q_A_B);
xA_B = xA_B(2:end)
xA_C = quatrotate(xA_A,q_A_C);
xA_C = xA_C(2:end)

q_B_C = quatmultiply(q_A_C,quatinv(q_A_B))
xB_B = [1.0 0.0 0.0];
xB_C = quatrotate(xB_B,q_B_C);
xB_C = xB_C(2:end)

% 3. Quatrenion and rotational matrix conversion
M_A_B_1 = quat2rotm(q_A_B)
yA_B = [1.0 0.0 0.0];
zA_B = [0.0 0.0 1.0];
M_A_B_2 = [xA_B;yA_B;zA_B]'

M_A_C_1 = quat2rotm(q_A_C)
yA_C = [0.0 1.0 0.0];
zA_C = [-1.0 0.0 0.0];
M_A_C_2 = [xA_C;yA_C;zA_C]'

% xA_A to xA_B & xA_A to xA_C
xA_B = M_A_B_1 * xA_A'
xA_C = M_A_C_1 * xA_A'

%M_B_C
M_B_C = M_A_C_1 * (M_A_B_1)'
yB_C = [0.0, 0.0, -1.0];
zB_C = [-1.0, 0.0, 0.0];
M_B_C_2 = [xB_C;yB_C;zB_C]'

% 4. Quaternion/Rotational matrix and Euler angle conversion
% rotate B by gamma, beta, alpha with 'ZYX' order to get C
alpha = D2R(0.0);  % rotation angle around x axis
beta = D2R(90.0);  % rotation angle around y axis
gamma = D2R(-90.0);  % rotation angle around z axis
q_B_C = quatinv(angle2quat(gamma, beta, alpha, 'ZYX'))

[gamma_res, beta_res, alpha_res] = quat2angle(quatinv(q_B_C), 'ZYX');
R2D(gamma_res)
R2D(beta_res)
R2D(alpha_res)

function b = D2R(a)
b = a*3.14159/180.0;
end
function a = R2D(b)
a = b*180.0/3.14159;
end