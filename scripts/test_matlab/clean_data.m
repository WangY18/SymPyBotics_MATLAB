%% 
A_write = [];
for n_exp = 1:5
    tab = readtable("..\..\data\Exp_20250612\output"+string(n_exp)+"\output.csv");
    
    Ts = 1e-3;
    q_command = [tab.q_command1,tab.q_command2,tab.q_command3,tab.q_command4,tab.q_command5,tab.q_command6,tab.q_command7];
    q_predict = [tab.q_predict1,tab.q_predict2,tab.q_predict3,tab.q_predict4,tab.q_predict5,tab.q_predict6,tab.q_predict7];
    q_real = [tab.q_real1,tab.q_real2,tab.q_real3,tab.q_real4,tab.q_real5,tab.q_real6,tab.q_real7];
    dq_real = [tab.dq_real1,tab.dq_real2,tab.dq_real3,tab.dq_real4,tab.dq_real5,tab.dq_real6,tab.dq_real7];
    dq_gradient = gradient_uniformgrid(q_real')'/Ts;
    d2q_real = gradient_uniformgrid(dq_real')'/Ts;
    dq_command = gradient_uniformgrid(q_command')'/Ts;
    d2q_command = gradient_uniformgrid(dq_command')'/Ts;
    d3q_command = gradient_uniformgrid(d2q_command')'/Ts;
    torque_real = [tab.torque_real1,tab.torque_real2,tab.torque_real3,tab.torque_real4,tab.torque_real5,tab.torque_real6,tab.torque_real7];


    Fs = 1000;        % 采样频率 (Hz)
    Fc = 10;         % 截止频率 (Hz)
    q_filter = zeros(size(q_real));
    torque_filter = zeros(size(torque_real));
    order = 4;
    Wn = Fc / (Fs/2);              % 归一化截止频率 (0~1)
    [b, a] = butter(order, Wn, 'low');
    for i = 1:7
        q_filter(:,i) = filtfilt(b, a, q_real(:,i)-q_command(:,i)) + q_command(:,i);         % 零相位滤波（推荐）
        torque_filter(:,i) = filtfilt(b, a, torque_real(:,i));
    end
    dq_filter = gradient_uniformgrid(q_filter')'/Ts;
    d2q_filter = gradient_uniformgrid(dq_filter')'/Ts;

    A_here = [q_filter,dq_filter,d2q_filter,torque_filter];
    A_write = [A_write;A_here(5000:59800,:)];
end
%% 
tab = array2table(A_write,'VariableNames',...
    {'q1','q2','q3','q4','q5','q6','q7','dq1','dq2','dq3','dq4','dq5','dq6','dq7',...
     'ddq1','ddq2','ddq3','ddq4','ddq5','ddq6','ddq7','torque1','torque2','torque3','torque4','torque5','torque6','torque7'});
writetable(tab,'..\data\Exp_20250612\feedback_filter.csv');
%% 
figure
for i = 1:7
    subplot(4,2,i)
    hold on
    plot((1:size(q_command,1))*Ts,q_command(:,i),'.-')
    plot((1:size(q_predict,1))*Ts,q_predict(:,i),'.-')
    plot((1:size(q_real,1))*Ts,q_real(:,i),'.-')

    % plot((1:size(dq_real,1))*Ts,dq_real(:,i))
    % plot((1:size(dq_gradient,1))*Ts,dq_gradient(:,i),'--')

    % plot((1:size(d2q_real,1))*Ts,d2q_real(:,i),'r.')
    % plot((1:size(d2q_filter,1))*Ts,d2q_filter(:,i),'k')
    % plot((1:size(d2q_command,1))*Ts,d2q_command(:,i),'b','LineWidth',2)

    % plot((1:size(dq_filter,1))*Ts,dq_filter(:,i),'k')
    % plot((1:size(dq_command,1))*Ts,dq_command(:,i),'b','LineWidth',2)

    % plot((1:size(torque_real,1))*Ts,torque_real(:,i),'k')
    % plot((1:size(torque_filter,1))*Ts,torque_filter(:,i),'b','LineWidth',2)
    hold off
    % xlim([5,59.8])
end
legend(["q_command","q_predict","q_real"])
%% 
figure
i = 7;
plot((1:size(dq_real,1))*Ts,dq_real(:,i),'.')
hold on
plot((1:size(dq_gradient,1))*Ts,dq_gradient(:,i),'.')
hold off
%% About 0.1~0.6 deg of tracking error for each axis
figure
plot((1:size(q_real,1))*Ts,(q_real-q_command)*180/pi)
legend()
% i = 7;
% plot((1:size(q_real,1))*Ts,q_real(:,i),'.')
% hold on
% plot((1:size(q_command,1))*Ts,q_command(:,i),'.')
% hold off
%% 
i = 7;
Y = fft(q_real(:,i)-q_command(:,i));
L = length(Y);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
plot(1000*(0:(L/2))/L,P1)
ylim([0,9e-6])
%% 
Fs = 1000;        % 采样频率 (Hz)
Fc = 10;         % 截止频率 (Hz)
q_filter = zeros(size(q_real));
torque_filter = zeros(size(torque_real));
order = 4;
Wn = Fc / (Fs/2);              % 归一化截止频率 (0~1)
[b, a] = butter(order, Wn, 'low');
for i = 1:7
    q_filter(:,i) = filtfilt(b, a, q_real(:,i)-q_command(:,i)) + q_command(:,i);         % 零相位滤波（推荐）
    torque_filter(:,i) = filtfilt(b, a, torque_real(:,i));
end

%% 可视化原始与滤波后信号
t = (0:length(x)-1)/Fs;
i = 2;
figure
plot(t, q_real(:,i), 'b.-', 'DisplayName', '原始信号'); hold on;
plot(t, q_filter(:,i), 'r', 'LineWidth', 1.5, 'DisplayName', '低通滤波后');
xlabel('时间 (s)')
ylabel('信号幅值')
legend
grid on
title('低通滤波效果对比')
%% 
dq_filter = gradient_uniformgrid(q_filter')'/Ts;
d2q_filter = gradient_uniformgrid(dq_filter')'/Ts;
d3q_filter = gradient_uniformgrid(d2q_filter')'/Ts;
%% 
plot(d2q_filter(5000:59800,:))