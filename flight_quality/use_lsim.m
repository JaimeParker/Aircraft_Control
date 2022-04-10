clear; clc;

% P467 example on Book PSDC
A = [-0.0453   0.0363  0       -0.1859;
     -0.3717  -2.0354  0.9723   0;
      0.3398  -7.0301 -2.9767   0;
      0        0       1        0];
  
B = [0; -0.1609; -11.8674; 0];

[x, y] = eig(A);
lambda = diag(y);

C1 = [1 0 0 0];
C2 = [0 1 0 0];
C3 = [0 0 1 0];
C4 = [0 0 0 1];

D = 0;

delta_alpha = 5;
x_0 = [0; delta_alpha; 0; 0];
x_1 = [16; delta_alpha; 0; 0];
x_2 = [0; delta_alpha; 0; -6];
x_3 = [0; 5 / 57.3; 0; 0];

t = 0:0.01:199.99;
u = (0.1*ones(1, length(t)));

sys1 = ss(A, B, C1, D); % u
sys2 = ss(A, B, C2, D); % delta_alpha
sys3 = ss(A, B, C3, D); % q
sys4 = ss(A, B, C4, D); % delta_theta

figure(1);
subplot(2,2,1);
lsim(sys1, u, t, x_1);
y = lsim(sys1, u, t, x_1);
xlabel("Time(s)"), ylabel("\Delta speed (m/s)"), grid on;
axis([0 200 -15 20 ]);

subplot(2,2,2);
lsim(sys4, u, t, x_0);
xlabel("Time(s)"), ylabel("\Delta \theta (degree)"), grid on;
axis([0 200 -6 8]);

subplot(2,2,3);
lsim(sys2, u, t, x_0);
xlabel("Time(s)"), ylabel("\Delta \alpha (degree)"), grid on;
axis([0 20 -2 6]);

subplot(2,2,4);
lsim(sys3, u, t, x_0);
xlabel("Time(s)"), ylabel("\Delta q (degree/s)"), grid on;
axis([0 20 -6 2]);


figure(2);
subplot(3, 1, 1);
temp = y';
plot(t, temp);
xlabel("Time(s)"), ylabel("\Delta speed (m/s)"), grid on;
axis([0 200 -15 20]);

subplot(3, 1, 2);
y1 = lsim(sys4, u, t, x_2);
temp1 = y1';
plot(t, temp1);
xlabel("Time(s)"), ylabel("\Delta \theta (degree)"), grid on;
axis([0 200 -6 8]);

subplot(3, 1, 3);
y2 = lsim(sys2, u, t, x_3);
temp2 = y2';
plot(t, temp2 * 57.3);
xlabel("Time(s)"), ylabel("\Delta \alpha (degree)"), grid on;
axis([0 20 -2 6]);



