% define parameters
% v = 16m/s in this version
m = 3.0;
g = 9.8;
rho = 1.225;
h = 500;
snoicVelocity = 338.37;
v = 16;
Ma = v / snoicVelocity;
W = m * g;
trans_rad = pi / 180;

Ix = 0.11931;
Iy = 0.31096;
Iz = 0.42318;
Ixz = -0.01537;

S = 0.332;
b = 1.6;
c_bar = 0.2075;
e = 1;
A = b / c_bar;
k = 1 / (pi * e * A);

alpha = 2;
theta = 0;

v1 = 12;
v2 = 16;
v3 = 26;
C_L1 = 2 * W / (rho * S * v1^2);
C_L2 = 2 * W / (rho * S * v2^2);
C_L3 = 2 * W / (rho * S * v3^2);  

% process value
U_0 = v;
c_1 = c_bar / 2 / U_0;
m_1 = 2 * m / rho / U_0 / S;
Iy_1 = Iy / (0.5 * rho * U_0^2 * S * c_bar);

C_mq = -16.76863;
% change any variable, alpha = 2 in this code
C_L = 0.5648;
C_D = 0.031307;
C_m_alpha = (-0.24777 - 0.177658) / (14 * trans_rad);
C_D_alpha = (0.035806 - 0.02757) / (2 * trans_rad);  % Be advised, C_D_alpha is not basically linearable
C_L_alpha = (1.273466 - 0.026789) / (14 * trans_rad);

    % d_alpha
    C_D_dAlpha = 0;
    C_x_dAlpha = -C_D_dAlpha;
    C_z_dAlpha = 0;
    C_m_dAlpha = 0.3 * C_mq;
    % xi_1 and xi_2
    xi_1 = (c_1 * C_x_dAlpha) / (m_1 - c_1 * C_z_dAlpha);
    xi_2 = (c_1 * C_m_dAlpha) / (m_1 - c_1 * C_z_dAlpha);

    % a_11
    C_DU = 0;
    C_LU = 0;
    C_XU = -2 * C_D - C_DU;
    C_ZU = -2 * C_L - C_LU;
    % a_12
    C_X_alpha = C_L - C_D_alpha;
    C_Z_alpha = -C_L_alpha - C_D;
    % a_13
    C_Dq = 0;
    C_Lq = 10.06864;
    C_Xq = -C_Dq;
    C_Zq = -C_Lq;
    % a_14
    C_X_theta = -C_L * cos(theta);
    C_Z_theta = -C_L * sin(theta);
    % a_31
    C_mu = 0;
    % b_1
    C_D_delta_e = 0;
    C_L_delta_e = 0;
    C_X_delta_e = -C_D_delta_e;
    C_Z_delta_e = -C_L_delta_e;
    % b_3
    C_m_delta_e = 0;

% get matrix A
a11 = (C_XU + xi_1 * C_ZU) / m_1;
a12 = (C_X_alpha + xi_1 * C_Z_alpha) / m_1;
a13 = (C_Xq * c_1 + xi_1 * (m_1 + c_1 * C_Zq)) / m_1;
a14 = (C_X_theta + xi_1 * C_Z_theta) / m_1;
a21 = C_ZU / (m_1 - C_z_dAlpha * c_1);
a22 = C_Z_alpha / (m_1 - C_z_dAlpha * c_1);
a23 = (m_1 + c_1 * C_Zq) / (m_1 - c_1 * C_z_dAlpha);
a24 = C_Z_theta / (m_1 - c_1 * C_z_dAlpha);
a31 = (C_mu + xi_2 * C_ZU) / Iy_1;
a32 = (C_m_alpha + xi_2 * C_Z_alpha) / Iy_1;
a33 = (C_mq * c_1 + xi_2 * (m_1 + c_1 * C_Zq)) / Iy_1;
a34 = (xi_2 * C_Z_theta) / Iy_1;
a41 = 0;
a42 = 0;
a43 = 1;
a44 = 0;

A = [a11 a12 a13 a14;
     a21 a22 a23 a24;
     a31 a32 a33 a34;
     a41 a42 a43 a44];

[x, y] = eig(A);
lambda = diag(y);

% get matrix B
b1 = (C_X_delta_e + xi_1* C_Z_delta_e) / m_1;
b2 = C_Z_delta_e / (m_1 - c_1 * C_z_dAlpha);
b3 = (C_m_delta_e + xi_2 * C_Z_delta_e) / Iy_1;
b4 = 0;

B = [b1; b2; b3; b4];

% test flight control
x_0 = [10; 5; 0; 0];
span = 40;
t_final = span * 10 - 0.01;
t = 0:0.01:t_final;
u = [5*ones(1, length(t)/span) zeros(1, length(t)/span * (span - 1))];
C = [0 0 0 1];
D = 0;
sys = ss(A, B, C, D);

C1 = eye(4);
D1 = zeros(4, 1);
[num, den] = ss2tf(A, B, C1, D1);
Gs = tf(mat2cell(num, [1, 1, 1, 1], 5), den);

lsim(sys, u, t, x_0);
axis([0 span * 10 -12 14]);


% flight quality
% lambda_12 = -5.2221\pm 7.0919i
% lambda_34 = -0.0105\pm 0.7649i
r_short = 5.2221;
s_short = 7.0919;
xi_short = r_short / (sqrt(r_short^2 + s_short^2));
omega_short = sqrt(r_short^2 + s_short^2);
r_long = 0.0105;
s_long = 0.7649;
xi_long = r_long / (sqrt(r_long^2 + s_long^2));
omega_long = sqrt(r_long^2 + s_long^2);

n = 1;
n_div_alpha = rho * U_0^2 * S * C_L_alpha / 2 / W;
omega_nsp = omega_short^2 / n_div_alpha;

t_a = 0.6931 / r_long;


















