% ���������
% ÿһ�ڶ�Ӧһ����·
% ���"hand_linear_analyse.slx"ʹ��
% �ڸ�slx�ļ��У�Ҫ�Ѵ����ķ��ӷ�ĸ��״̬�ռ��ABCD���󻻳��Լ��Ķ���ѧģ��

%% ���ݺ������q��·

% �������浽q�Ĵ����Ĳο�ֵ��
% UAV.linear.q_sp_de.transfer = 
%      -33.9 s - 173.8
%   ---------------------
%   s^2 + 10.62 s + 79.51

sisotool(UAV.linear.q_sp_de.transfer);  

% q_controller = tf([0.5,0.2],[1,0]);	% q_controller(s) = 0.5+0.2/s = Kq*(1+0.4/s); % Kq = 0.5, KqI = 0.2

%% ���ݺ�����Ƹ�����theta��·
% ����inΪqc�����Ϊtheta
q_controller= 0.5;  % q��·����ƽ��
tf_de2theta = tf(1,[1,0])*feedback(UAV.linear.q_sp_de.transfer * q_controller, -1);
sisotool(tf_de2theta);

%% 4����������С�Ŷ������Ż���ƿ�����
% ��Control law design����
% �Ͽ� Ktheta,����Ϊthetac�����Ϊtheta
[num_theta,den_theta]=linmod('hand_linear_analyse');
tf_theta_loop = tf(num_theta,den_theta);
tf_theta_loop = minreal(tf_theta_loop);
sisotool(tf_theta_loop);

%% �ẽ��
%% ƫ�����ٶ�r
% �ӷ���浽r�Ĵ����Ĳο�ֵ��
% UAV.linear.r_dutch_dr.transfer = 
%     -2.877 s + 1.277
%   ---------------------
%   s^2 + 2.468 s + 27.99
  
tf_dr2r = UAV.linear.r_dutch_dr.transfer;
sisotool(tf_dr2r); 

%% ��ת���ٶ�p

% �Ӹ���浽p�Ĵ����Ĳο�ֵ��
% UAV.linear.lateral_roll = 
%    -21.65
%   ---------
%   s + 17.54
  
sisotool(UAV.linear.lateral_roll); 

%% ��ת��phi
p_controller= 0.8;  % p��·����ƽ��
tf_da2phi = tf(1,[1,0])*feedback(UAV.linear.lateral_roll * p_controller, -1);
sisotool(tf_da2phi);

%% 4�������ẽ��С�Ŷ������Ż���ƿ�����
% ��Control law design����
% �Ͽ� Kphi,����Ϊphic�����Ϊphi
[num_phi,den_phi]=linmod('hand_linear_analyse');
tf_phi_loop = tf(num_phi,den_phi);
tf_phi_loop = minreal(tf_phi_loop);
sisotool(tf_phi_loop);


