% Human joint angle calculation demo

% Define three coordinate systems (CS):
% 1. Global(North-Up-East): x - North, y - Up, z - East (G)
% 2. Pelvis(when standing upright): x - Left, y - Up, z - front (P)
% 3. Right Thigh(when standing upright): x - Left, y - Up, z - front (T)

% Firstly, consider human standing facing North, then
% Pelvis & Thigh's x axis is pointing West: -z of Global CS
% Pelvis & Thigh's y axis is pointing Up: y of Global CS
% Pelvis & Thigh's z axis is pointing North: x of Global CS

% Pelvis's axes in Global CS:
xP_G = [0.0 0.0 -1.0];
yP_G = [0.0 1.0 0.0];
zP_G = [1.0 0.0 0.0];
M_P_G = [xP_G;yP_G;zP_G]';
q_P_G = rotm2quat(M_P_G)

% Secondly, consider thigh flex upward 45 degree, then abbduct 30 degree
% rotate P by alpha, beta, gamma with 'XYZ' order to get T 
alpha = D2R(-45.0);
beta = D2R(-30.0);
gamma = D2R(0.0);
q_T_P = angle2quat(alpha, beta, gamma, 'XYZ')
q_T_G = quatmultiply(q_P_G, q_T_P)

% Now we have two quaternion q_P_G & q_T_G, both related to Global
% Calculate relative rotation of right thigh with respect to pelvis
q_T_P = quatmultiply(quatinv(q_P_G), q_T_G)
[alpha_res, gamma_res, beta_res] = quat2angle(q_T_P, 'XZY')
R2D(alpha_res)
R2D(beta_res)
R2D(gamma_res)

function b = D2R(a)
b = a*3.14159/180.0;
end
function a = R2D(b)
a = b*180.0/3.14159;
end



