# Code Demo 示例代码

不包含控制律设计，不包含插值代码

纵向原始代码参考自**ZSChen**，横航向借鉴了纵向代码的风格和结构

**如有必要，请注明出处**

#### 1. 纵向

```matlab
clear;clc;

% INPUT
u = 16;

% Plane Parameters
m = 3;
c = 0.2075;
S = 0.332;
Ibxx = 0.11931;
Ibyy = 0.31096;
Ibzz = 0.42318;
Ibxz = -0.01537;
rho = 1.16727;  % Density of Air at 500m
W = m * 9.8;
trans_rad = pi / 180;

% Calculate Parameters 
CL = 2 * W / (rho * S * u^2);
alpha = 
CD = 

disp('alpha='), disp(alpha);
disp('CL='), disp(CL);
disp('CD='), disp(CD);

% Derivative
CLa = 
CDa = 
Cma = 

% Given Parameters
CLq = 10.06864;
Cmq = -16.76863;
Cmadot = 0.3 * Cmq;

% Temp Parameters
Cxu = -2 * CD;
Cxa = CL - CDa;     % a represents alpha
Cxth = -CL;         % th represents theta
Cxadot = 0;
Cxq = 0;
Czu = -2 * CL;
Cza = -CLa -CD;
Czadot = 0;
Czq = -CLq;
Czth = 0;
Cmu = 0;
m1 = 2 * m / (rho * u * S);
c1 = c / 2 / u;
Iy1 = 2 * Ibyy / (rho * u^2 * S * c);
k1 = Cxadot * c1 / (m1 - Czadot * c1);
k2 = Cmadot * c1 / (m1 - Czadot * c1);

% StateSpace
A = zeros(4, 4);
A(1, 1) = (Cxu + k1 * Czu) / m1;
A(1, 2) = (Cxa + k1 * Cza) / m1;
A(1, 3) = (Cxq + k1 * (m1 + Czq * c1)) / m1;
A(1, 4) = (Cxth + k1 * Czth) / m1;
A(2, 1) = Czu / (m1 - Czadot * c1);
A(2, 2) = Cza / (m1 - Czadot * c1);
A(2, 3) = (m1 + Czq * c1) / (m1 - Czadot * c1);
A(2, 4) = Czth / (m1 - Czadot * c1);
A(3, 1) = (Cmu + k2 * Czu) / Iy1;
A(3, 2) = (Cma + k2 * Cza) / Iy1;
A(3, 3) = (Cmq * c1 + k2 * (m1 + Czq * c1)) / Iy1;
A(3, 4) = 0;
A(4, :) = [0, 0, 1, 0];
A;

% eigenvalue
eigenvalue = eig(A);
disp('lambda='), disp(eigenvalue);

% Params Related to Delta_e
ElevaorData = readmatrix('ElevatorData.xlsx');
ele_alpha0 = [-4, 0, 4, 8];
de0 = -20:5:20;     % de represents delta_e
% [ele_alpha0, de0]=meshgrid(ele_alpha0, de0);
ele_CL0 = reshape(ElevaorData(:, 3), 9, 4);
ele_CD0 = reshape(ElevaorData(:, 4), 9, 4);
ele_Cm0 = reshape(ElevaorData(:, 5), 9, 4);

% Find Current Delta_e
de = 

% Derivatives related to Delta_e
CLde = 
CDde = 
Cmde = 

% TempParams for Matrix B
Cxde = -CDde;
Czde = -CLde;

% Matrix B
B = zeros(4, 1);
B(1) = (Cxde + k1 * Czde) / m1;
B(2) = Czde / (m1 - c1 * Czadot);
B(3) = (Cmde + k2 * Czde) / Iy1;
B(4) = 0;
B;

disp('A='), disp(A);
disp('B='), disp(B);

% transfor function
C = eye(4);
D = zeros(4, 1);
[num, den] = ss2tf(A, B, C, D);
Gs = tf(mat2cell(num, [1, 1, 1, 1], 5), den); % 4+1=5

A_short = zeros(2, 2);
B_short = zeros(2, 1);
As = A_short;
Bs = B_short;
As(1, 1) = Cza / (m1 - Czadot * c1);
As(1, 2) = (m1 + Czq * c1) / (m1 - Czadot * c1);
As(2, 1) = (1 / Iy1) * (Cma + Cmadot * c1 * Cza / (m1 - Czadot * c1));
As(2, 2) = (1 / Iy1) * (Cmq * c1 + (m1 + c1 * Czq) * c1 * Cmadot / (m1 - Czadot * c1));
Bs(1, 1) = Czde / (m1 - Czadot * c1);
Bs(2, 1) = (1 / Iy1) * (Cmde + Cmadot * c1 * Czde / (m1 - Czadot * c1));
Cs = eye(2);
Ds = zeros(2, 1);
[num_s, den_s] = ss2tf(As, Bs, Cs, Ds);
Gs_short = tf(mat2cell(num_s, [1, 1], 3), den_s);
Gs_short
Gss = tf(num(4, :), den);
num_sp = num_s(2, :);
den_sp = den_s;

% ThrustData
DragForce = CD * 0.5 * rho * u^2 * S;
T = DragForce;

Tdp = 

% Entire 4 order system
B = [B, zeros(4, 1)];
B(1, 2) = Tdp * cos(alpha * trans_rad) / m;
B(2, 2) = -Tdp * sin(alpha * trans_rad) / m / u;
B(3, 2) = B(2, 2) * Cmadot * (c / 2 / u) * 0.5 * rho * u^2 * S * c / Ibyy;
D = [D, zeros(4, 1)];

% flight quality
lambda = eigenvalue;
r_lp = abs(real(lambda(3)));
s_lp = abs(imag(lambda(3)));
r_sp = abs(real(lambda(1)));
s_sp = abs(imag(lambda(1)));

xi_lp = r_lp / sqrt(r_lp^2 + s_lp^2);
omega_lp = sqrt(r_lp^2 + s_lp^2);
xi_sp = r_sp / sqrt(r_sp^2 + s_sp^2);
omega_sp = sqrt(r_sp^2 + s_sp^2);

n = 1;
n_divide_alpha = 0.5 * rho * u^2 * S * CLa / W;
CAP = omega_sp^2 / n_divide_alpha;

ta = 0.6931 / abs(r_lp);

disp('xi_sp,min='), disp(xi_sp);
disp('xi_lp='), disp(xi_lp);
disp('CAP='), disp(CAP);
disp('ta='), disp(ta);
```

#### 2. 横航向

```matlab
clear;clc;

% INPUT
u0 = 16;

% Plane Parameters
m = 3;
c = 0.2075;
S = 0.332;
W = m * 9.8;
b = 1.6;
Ibxx = 0.11931;
Ibyy = 0.31096;
Ibzz = 0.42318;
Ibxz = -0.01537;
rho = 1.16727;  % Density of Air at 500m
trans_rad = pi / 180; % transformation from degree to rad 

% Calculate Parameters
CL = 2 * W / (rho * S * u0^2);
alpha = 
if alpha > 4
    alpha = 4; % set limit for alpha
end
if alpha < -4
    alpha = -4;
end

% Given Parameters
alpha0 = 2 * trans_rad;
theta0 = 0 * trans_rad;
Cy_dBeta = 0;
Cl_dBeta = 0;
Cn_dBeta = 0;
Cyp = -0.07474;
Clp = -0.51302;
Cnp = -0.09846;
Cyr = 0.38431;
Clr = 0.14807;
Cnr = -0.12997;
Clr = 0.5 * Clr; % fix Clr
Cnr = 1.5 * Cnr; % fix Cnr

% Derivatives
Cy_phi = CL * cos(theta0);
% interp2 2 dimension for Cy, Cl, Cn
Cy_beta = 
Cl_beta = 
Cn_beta = 

% interp2 2 dimension for Cy, Cl, Cn in rudder
Cy_delta_r = 
Cl_delta_r = 
Cn_delta_r = 

Cy_delta_a = 
Cl_delta_a = 
Cn_delta_a = 

% Temp Parameters
b1 = b / (2 * u0);
m1 = 2 * m / (rho * u0 * S);
Ix1 = 2 * Ibxx / (rho * u0^2 * S * b);
Iz1 = 2 * Ibzz / (rho * u0^2 * S * b);
Ixz1 = 2 * Ibxz / (rho * u0^2 * S * b);
Ix2 = Ix1 / (Ix1 * Iz1 - Ixz1^2);
Iz2 = Iz1 / (Ix1 * Iz1 - Ixz1^2);
Ixz2 = Ixz1 / (Ix1 * Iz1 - Ixz1^2);
xi1 = Iz2 * Cl_dBeta + Ixz2 * Cn_dBeta;
xi2 = Ix2 * Cn_dBeta + Ixz2 * Cl_dBeta;

% Matrix A
A = zeros(5, 5);
A(1, 1) = Cy_beta / (m1 - b1 * Cy_dBeta);
A(1, 2) = Cy_phi / (m1 - b1 * Cy_dBeta);
A(1, 3) = Cyp * b1 / (m1 - b1 * Cy_dBeta);
A(1, 4) = 0;
A(1, 5) = -(m1 - b1 * Cyr) / (m1 - b1 * Cy_dBeta);
A(2, :) = [0, 0, 1, 0, 0];
A(3, 1) = Cl_beta * Iz2 + Cn_beta * Ixz2 + xi1 * b1 * A(1, 1);
A(3, 2) = xi1 * b1 * A(1, 2);
A(3, 3) = Clp * b1 * Iz2 + Cnp * Ixz2 * b1 + xi1 * b1 * A(1, 3);
A(3, 4) = 0;
A(3, 5) = Clr * b1 * Iz2 + Cnr * Ixz2 * b1 + xi1 * b1 * A(1, 5);
A(4, :) = [0, 0, 0, 0, 1];
A(5, 1) = Ix2 * Cn_beta + Ixz2 * Cl_beta + b1 * xi2 * A(1, 1);
A(5, 2) = xi2 * b1 * A(1, 2);
A(5, 3) = b1 * (Cnp * Ix2 + Clp * Ixz2 + xi2 * A(1, 3));
A(5, 4) = 0;
A(5, 5) = b1 * (Ix2 * Cnr + Ixz2 * Clr + xi2 * A(1, 5));

% eigen value
lambda = eig(A);

% Matrix B
B(2, :) = [0, 0];
B(4, :) = [0, 0];
B(1, 1) = Cy_delta_a / (m1 - b1 * Cy_dBeta);
B(3, 1) = Cl_delta_a * Iz2 + Cn_delta_a * Ixz2 + xi1 * b1 * B(1, 1);
B(5, 1) = Cn_delta_a * Ix2 + Cl_delta_a * Ixz2 + xi2 * b1 * B(1, 1);
B(1, 2) = Cy_delta_r / (m1 - b1 * Cy_dBeta);
B(3, 2) = Cl_delta_r * Iz2 + Cn_delta_r * Ixz2 + xi1 * b1 * B(1, 2);
B(5, 2) = Cn_delta_r * Ix2 + Cl_delta_r * Ixz2 + xi2 * b1 * B(1, 2);

% flight quality
lambda_roll = lambda(5);
lambda_spiral = lambda(2);
lambda_dutch = lambda(3);

tau_roll = -Ix1 / Clp / b1;
tau_roll1 = 1 / abs(real(lambda_roll));

r_dutch = abs(real(lambda_dutch));
s_dutch = abs(imag(lambda_dutch));
xi_dutch = r_dutch / sqrt(r_dutch^2 + s_dutch^2);
omega_n_dutch = sqrt(r_dutch^2 + s_dutch^2);
xi_omega_dutch = xi_dutch * omega_n_dutch;

ta_spiral = 0.6931 / abs(real(lambda_spiral));

disp('A='), disp(A);
disp('B='), disp(B);

disp('lambda:'), disp(lambda);

disp('roll: Time constant='), disp(tau_roll);

disp('dutch roll:')
dutch_info = ['damp=', num2str(xi_dutch), ', damp*omega=', num2str(xi_omega_dutch), ', omega=', num2str(omega_n_dutch)];
disp(dutch_info);
disp(' ');

disp('spiral: double time='), disp(ta_spiral);

% transfer function
C = eye(5);
D = zeros(5, 1); % zeros(5, 2) actually

B_aileron = B(:, 1);
B_rudder = B(:, 2);

[num_a, den_a] = ss2tf(A, B_aileron, C, D);
G_cell_aileron = tf(mat2cell(num_a, ones(1, 5), 6), den_a);

[num_r, den_r] = ss2tf(A, B_rudder, C, D);
G_cell_rudder = tf(mat2cell(num_r, ones(1, 5), 6), den_r);

% UAV.linear.r_dutch_dr.transfer
A_dutch = zeros(2, 2);
A_dutch(1, 1) = Cy_beta / (m1 - Cy_dBeta * b1);
A_dutch(1, 2) = - (m1 - b1 * Cyr) / (m1 - b1 * Cy_dBeta);
A_dutch(2, 1) = (1 / Iz1) * (Cn_beta + (Cn_dBeta * b1 * Cy_beta) / (m1 - b1 * Cy_dBeta));
A_dutch(2, 2) = (1 / Iz1) * (b1 * Cnr + (-Cn_dBeta * b1 * (m1 - b1 * Cyr)) / (m1 - Cy_dBeta * b1));
B_dutch_dr = zeros(2, 1);
B_dutch_dr(1, 1) = Cy_delta_r / (m1 - Cy_dBeta * b1);
B_dutch_dr(2, 1) = Cn_delta_r + Cn_dBeta * b1 * Cy_delta_r / (m1 - Cy_dBeta * b1);
B_dutch_dr(2, 1) = B_dutch_dr(2, 1) / Iz1;
C_dutch = eye(2);
D_dutch_dr = zeros(2, 1);

[num_dutch_r, den_dutch_r] = ss2tf(A_dutch, B_dutch_dr, C_dutch, D_dutch_dr);
Gr_dutch_dr = tf(num_dutch_r(2, :), den_dutch_r);
G_cell_dutch_r = tf(mat2cell(num_dutch_r, ones(1, 2), 3), den_dutch_r);

Gr_dutch_dr

% get lateral_roll transfer function
num_da2p = Cl_delta_a / Ix1;
den_da2p = [1, - Clp * b1 / Ix1];
G_da2p = tf(num_da2p, den_da2p);
G_da2p
```

