close all, clear all, clc;

[s_array, s_hat_array, y_array]=Kalman();   % 计算理论值、观测值、无约束Kalman滤波估计值
s_con_array = C_Kalman(s_hat_array);        % 计算有约束Kalman滤波估计值

% 画图
t = 0.1 : 0.1 : 300;
% 汽车理论轨迹
figure(1);
plot(s_array(1, :), s_array(2, :));
title('理论轨迹');
xlabel('x_e (m)');
ylabel('x_n (m)');

% 汽车理论速度-时间
figure(2);
subplot(1,2,1);
plot(t, s_array(3, :));
xlabel('t/s');
ylabel('v_e (m/s)');
subplot(1,2,2);
plot(t, s_array(4, :));
xlabel('t/s');
ylabel('v_n (m/s)');

% 汽车理论位移-时间
figure(3);
subplot(1,2,1);
plot(t, s_array(1, :));
xlabel('t/s');
ylabel('x_e (m)');
subplot(1,2,2);
plot(t, s_array(2, :));
xlabel('t/s');
ylabel('x_n (m)');

% 观测值-时间
figure(4);
subplot(1,2,1);
plot(t, y_array(1, :));
xlabel('t/s');
ylabel('y_1 (m)');
subplot(1,2,2);
plot(t, y_array(2, :));
xlabel('t/s');
ylabel('y_2 (m)');

% 无约束Kalman滤波估计值位移-时间
figure(5);
subplot(1,2,1);
plot(t, s_array(1, :), t, s_hat_array(1, :), 'r-');
xlabel('t/s');
ylabel('x hat_e (m)');
subplot(1,2,2);
plot(t, s_array(2, :), t, s_hat_array(2, :), 'r-');
xlabel('t/s');
ylabel('x hat_n (m)');

% 无约束Kalman滤波估计轨迹
figure(6);
plot(s_array(1, :), s_array(2, :), s_hat_array(1, :), s_hat_array(2, :), 'r-');
title('无约束Kalman滤波估计轨迹');
xlabel('x hat_e (m)');
ylabel('x hat_n (m)');

% 无约束Kalman估计汽车位置误差-时间
figure(7);
subplot(1,2,1);
plot(t, s_array(1,:)-s_hat_array(1,:));
xlabel('t/s');
ylabel('error_e (m)');
subplot(1,2,2);
plot(t, s_array(2,:)-s_hat_array(2,:));
xlabel('t/s');
ylabel('error_n (m)');

% 约束Kalman滤波估计值位移-时间
figure(8);
subplot(1,2,1);
plot(t, s_array(1, :), t, s_con_array(1, :), 'r-');
xlabel('t/s');
ylabel('x con_e (m)');
subplot(1,2,2);
plot(t, s_array(2, :), t, s_con_array(2, :), 'r-');
xlabel('t/s');
ylabel('x con_n (m)');

% 约束Kalman滤波估计位置
figure(9);
plot(s_array(1, :), s_array(2, :), s_con_array(1, :), s_con_array(2, :), 'r-');
title('约束Kalman滤波估计轨迹');
xlabel('x con_e (m)');
ylabel('x con_n (m)');

% 约束Kalman估计汽车位置误差-时间
figure(10);
subplot(1,2,1);
plot(t, s_array(1,:)-s_con_array(1,:));
xlabel('t/s');
ylabel('error_e (m)');
subplot(1,2,2);
plot(t, s_array(2,:)-s_con_array(2,:));
xlabel('t/s');
ylabel('error_n (m)');

% 误差比较
figure(11);
subplot(2,2,1);
plot(t, s_array(1,:)-s_con_array(1,:), t, s_array(1,:)-s_hat_array(1,:), 'r-');
title("位移误差（东）-时间");
xlabel('t/s');
ylabel('x error_e (m)');
subplot(2,2,2);
plot(t, s_array(2,:)-s_con_array(2,:), t, s_array(2,:)-s_hat_array(2,:), 'r-');
title("位移误差（南）-时间");
xlabel('t/s');
ylabel('x error_n (m)');
subplot(2,2,3);
plot(t, s_array(3,:)-s_con_array(3,:), t, s_array(3,:)-s_hat_array(3,:), 'r-');
title("速度误差（东）-时间");
xlabel('t/s');
ylabel('v error_e (m)');
subplot(2,2,4);
plot(t, s_array(4,:)-s_con_array(4,:), t, s_array(4,:)-s_hat_array(4,:), 'r-');
title("速度误差（南）-时间");
xlabel('t/s');
ylabel('v error_n (m)');

figure(12);
err_hat = sqrt((s_array(1,:)-s_hat_array(1,:)).^2+(s_array(2,:)-s_hat_array(2,:)).^2);
err_con = sqrt((s_array(1,:)-s_con_array(1,:)).^2+(s_array(2,:)-s_con_array(2,:)).^2);
plot(t, err_con, t, err_hat, 'r-');
title("偏离理论位置距离");
xlabel('t/s');
ylabel('x error (m)');