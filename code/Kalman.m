function [s_array, s_hat_array, y_array]=Kalman()

tf = 300; % final time (s)
T = 0.1; % time step (s)
Qu = diag([0.01, 0.01]); % ��������Э����
Qw = diag([900, 900]); % �۲�����Э����
theta = pi / 3; % �����from east
tanth = tan(theta);
sinth = sin(theta);
costh = cos(theta);
% ��ʼ״̬
a0 = [costh;sinth];
s0 = [0;0;0;0];
m0 = zeros(4);
% ����ϵͳ
A = [1, 0 ,T, 0; 0, 1, 0, T;
    0, 0, 1, 0; 0, 0, 0, 1];
B = [0.5*T^2, 0; 0, 0.5*T^2;
    T, 0; 0, T];
% ����ֵ
s = s0;
s_hat = s0;
s_array = [];
s_hat_array = [];
y_array = [];
a_flag = 1;
cnt = 0;
m = m0; % MSE����

for t = T:T:tf
    if cnt == 100
        a_flag = -1 * a_flag;   %���ٶȷ���
        cnt = 1;
    else
        cnt = cnt + 1;
    end
    a = a_flag * a0;
    a_n = a + normrnd(0, 0.1, [2,1]);
    s = A*s + B*a;  %����ֵ
    s_array = [s_array ,s];
    
    y11 = ((s(1)+5000)^2 + s(2)^2)^0.5;
    y21 = ((s(1)-5000)^2 + s(2)^2)^0.5;
    y = [y11; y21];
    MeasErr = normrnd(0, 10, [2,1]);    %�۲����
    y = y + MeasErr;    %�۲�ֵ
    y_array = [y_array, y];
    
    m_pre = A*m*A' + B*Qu*B';   % M[n|n-1]
    s_pre = A*s_hat + B*a;      % s_hat[n|n-1]
    h11 = (s_pre(1)+5000) * ((s_pre(1)+5000)^2 + s_pre(2)^2)^(-1/2);
    h21 = (s_pre(1)-5000) * ((s_pre(1)-5000)^2 + s_pre(2)^2)^(-1/2);
    h12 = s_pre(2) * ((s_pre(1)+5000)^2 + s_pre(2)^2)^(-1/2);
    h22 = s_pre(2) * ((s_pre(1)-5000)^2 + s_pre(2)^2)^(-1/2);
    h_jocabi = [h11, h12, 0, 0; h21, h22, 0, 0];                % H[n] jocabi
    k = m_pre * h_jocabi' *inv(Qw + h_jocabi*m_pre*h_jocabi');  % K[n]
    
    hs = [((s_pre(1)+5000)^2 + s_pre(2)^2)^0.5; ((s_pre(1)-5000)^2 + s_pre(2)^2)^0.5];  % h(s_hat[n|n-1])
    s_hat = s_pre + k*(y - hs); % s_hat[n|n]
    m = (eye(4) - k*h_jocabi) * m_pre; % M[n|n]
    s_hat_array = [s_hat_array ,s_hat];
end

end