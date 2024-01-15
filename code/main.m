close all, clear all, clc;

[s_array, s_hat_array, y_array]=Kalman();   % ��������ֵ���۲�ֵ����Լ��Kalman�˲�����ֵ
s_con_array = C_Kalman(s_hat_array);        % ������Լ��Kalman�˲�����ֵ

% ��ͼ
t = 0.1 : 0.1 : 300;
% �������۹켣
figure(1);
plot(s_array(1, :), s_array(2, :));
title('���۹켣');
xlabel('x_e (m)');
ylabel('x_n (m)');

% ���������ٶ�-ʱ��
figure(2);
subplot(1,2,1);
plot(t, s_array(3, :));
xlabel('t/s');
ylabel('v_e (m/s)');
subplot(1,2,2);
plot(t, s_array(4, :));
xlabel('t/s');
ylabel('v_n (m/s)');

% ��������λ��-ʱ��
figure(3);
subplot(1,2,1);
plot(t, s_array(1, :));
xlabel('t/s');
ylabel('x_e (m)');
subplot(1,2,2);
plot(t, s_array(2, :));
xlabel('t/s');
ylabel('x_n (m)');

% �۲�ֵ-ʱ��
figure(4);
subplot(1,2,1);
plot(t, y_array(1, :));
xlabel('t/s');
ylabel('y_1 (m)');
subplot(1,2,2);
plot(t, y_array(2, :));
xlabel('t/s');
ylabel('y_2 (m)');

% ��Լ��Kalman�˲�����ֵλ��-ʱ��
figure(5);
subplot(1,2,1);
plot(t, s_array(1, :), t, s_hat_array(1, :), 'r-');
xlabel('t/s');
ylabel('x hat_e (m)');
subplot(1,2,2);
plot(t, s_array(2, :), t, s_hat_array(2, :), 'r-');
xlabel('t/s');
ylabel('x hat_n (m)');

% ��Լ��Kalman�˲����ƹ켣
figure(6);
plot(s_array(1, :), s_array(2, :), s_hat_array(1, :), s_hat_array(2, :), 'r-');
title('��Լ��Kalman�˲����ƹ켣');
xlabel('x hat_e (m)');
ylabel('x hat_n (m)');

% ��Լ��Kalman��������λ�����-ʱ��
figure(7);
subplot(1,2,1);
plot(t, s_array(1,:)-s_hat_array(1,:));
xlabel('t/s');
ylabel('error_e (m)');
subplot(1,2,2);
plot(t, s_array(2,:)-s_hat_array(2,:));
xlabel('t/s');
ylabel('error_n (m)');

% Լ��Kalman�˲�����ֵλ��-ʱ��
figure(8);
subplot(1,2,1);
plot(t, s_array(1, :), t, s_con_array(1, :), 'r-');
xlabel('t/s');
ylabel('x con_e (m)');
subplot(1,2,2);
plot(t, s_array(2, :), t, s_con_array(2, :), 'r-');
xlabel('t/s');
ylabel('x con_n (m)');

% Լ��Kalman�˲�����λ��
figure(9);
plot(s_array(1, :), s_array(2, :), s_con_array(1, :), s_con_array(2, :), 'r-');
title('Լ��Kalman�˲����ƹ켣');
xlabel('x con_e (m)');
ylabel('x con_n (m)');

% Լ��Kalman��������λ�����-ʱ��
figure(10);
subplot(1,2,1);
plot(t, s_array(1,:)-s_con_array(1,:));
xlabel('t/s');
ylabel('error_e (m)');
subplot(1,2,2);
plot(t, s_array(2,:)-s_con_array(2,:));
xlabel('t/s');
ylabel('error_n (m)');

% ���Ƚ�
figure(11);
subplot(2,2,1);
plot(t, s_array(1,:)-s_con_array(1,:), t, s_array(1,:)-s_hat_array(1,:), 'r-');
title("λ��������-ʱ��");
xlabel('t/s');
ylabel('x error_e (m)');
subplot(2,2,2);
plot(t, s_array(2,:)-s_con_array(2,:), t, s_array(2,:)-s_hat_array(2,:), 'r-');
title("λ�����ϣ�-ʱ��");
xlabel('t/s');
ylabel('x error_n (m)');
subplot(2,2,3);
plot(t, s_array(3,:)-s_con_array(3,:), t, s_array(3,:)-s_hat_array(3,:), 'r-');
title("�ٶ�������-ʱ��");
xlabel('t/s');
ylabel('v error_e (m)');
subplot(2,2,4);
plot(t, s_array(4,:)-s_con_array(4,:), t, s_array(4,:)-s_hat_array(4,:), 'r-');
title("�ٶ����ϣ�-ʱ��");
xlabel('t/s');
ylabel('v error_n (m)');

figure(12);
err_hat = sqrt((s_array(1,:)-s_hat_array(1,:)).^2+(s_array(2,:)-s_hat_array(2,:)).^2);
err_con = sqrt((s_array(1,:)-s_con_array(1,:)).^2+(s_array(2,:)-s_con_array(2,:)).^2);
plot(t, err_con, t, err_hat, 'r-');
title("ƫ������λ�þ���");
xlabel('t/s');
ylabel('x error (m)');