function s_con_array = C_Kalman(s_hat_array)

tf = 300; % final time (s)
T = 0.1; % time step (s)
theta = pi / 3; % �����from east
tanth = tan(theta);
D = [-tanth, 1, 0, 0; 0, 0, -tanth, 1]; % Լ��
d = [0; 0];
s_con_array = [];
cnt = 1;
for t = T:T:tf
    s_con = s_hat_array(:,cnt) - D'*inv(D*D') * (D*s_hat_array(:,cnt)-d);   %Լ��Kalman����ֵ
    s_con_array = [s_con_array ,s_con];
    cnt = cnt+1;
end

end