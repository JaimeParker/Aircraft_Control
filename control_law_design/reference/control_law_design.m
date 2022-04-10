% 控制律设计
% 每一节对应一个回路
% 配合"hand_linear_analyse.slx"使用
% 在该slx文件中，要把传函的分子分母和状态空间的ABCD矩阵换成自己的动力学模型

%% 传递函数设计q回路

% 从升降舵到q的传函的参考值：
% UAV.linear.q_sp_de.transfer = 
%      -33.9 s - 173.8
%   ---------------------
%   s^2 + 10.62 s + 79.51

sisotool(UAV.linear.q_sp_de.transfer);  

% q_controller = tf([0.5,0.2],[1,0]);	% q_controller(s) = 0.5+0.2/s = Kq*(1+0.4/s); % Kq = 0.5, KqI = 0.2

%% 传递函数设计俯仰角theta回路
% 输入in为qc，输出为theta
q_controller= 0.5;  % q回路的设计结果
tf_de2theta = tf(1,[1,0])*feedback(UAV.linear.q_sp_de.transfer * q_controller, -1);
sisotool(tf_de2theta);

%% 4阶完整纵向小扰动方程优化设计控制律
% 在Control law design框中
% 断开 Ktheta,输入为thetac，输出为theta
[num_theta,den_theta]=linmod('hand_linear_analyse');
tf_theta_loop = tf(num_theta,den_theta);
tf_theta_loop = minreal(tf_theta_loop);
sisotool(tf_theta_loop);

%% 横航向
%% 偏航角速度r
% 从方向舵到r的传函的参考值：
% UAV.linear.r_dutch_dr.transfer = 
%     -2.877 s + 1.277
%   ---------------------
%   s^2 + 2.468 s + 27.99
  
tf_dr2r = UAV.linear.r_dutch_dr.transfer;
sisotool(tf_dr2r); 

%% 滚转角速度p

% 从副翼舵到p的传函的参考值：
% UAV.linear.lateral_roll = 
%    -21.65
%   ---------
%   s + 17.54
  
sisotool(UAV.linear.lateral_roll); 

%% 滚转角phi
p_controller= 0.8;  % p回路的设计结果
tf_da2phi = tf(1,[1,0])*feedback(UAV.linear.lateral_roll * p_controller, -1);
sisotool(tf_da2phi);

%% 4阶完整横航向小扰动方程优化设计控制律
% 在Control law design框中
% 断开 Kphi,输入为phic，输出为phi
[num_phi,den_phi]=linmod('hand_linear_analyse');
tf_phi_loop = tf(num_phi,den_phi);
tf_phi_loop = minreal(tf_phi_loop);
sisotool(tf_phi_loop);


