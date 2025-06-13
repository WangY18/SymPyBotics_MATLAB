%% 
tab = readtable('..\data\Exp_20250612\feedback_filter.csv');
%% 
torque = get_Torque(tab{:,1:7},tab{:,8:14},tab{:,15:21});
%% 
torque_ori = [];
for n_exp = 1:5
    tab_ = readtable("..\data\Exp_20250612\output"+string(n_exp)+"\output.csv");
    torque_real = [tab_.torque_real1,tab_.torque_real2,tab_.torque_real3,tab_.torque_real4,tab_.torque_real5,tab_.torque_real6,tab_.torque_real7];
    torque_ori = [torque_ori;torque_real(5000:59800,:)];
end
%% 
figure
for i = 1:7
    subplot(4,2,i)
    hold on
    plot(tab{:,21+i},'b','LineWidth',1)
    % plot(torque_ori(:,i),'b','LineWidth',1)
    plot(torque(:,i),'r','LineWidth',1)
    hold off
    title("Joint "+string(i),"FontSize",20)
    ax = gca;
    ax.FontSize = 20;
end
lg = legend(["Measured Torque","Fitted Torque"],"FontSize",20,'Position',[0.5853    0.1772    0.0690    0.0339]);
%% 
figure
for i = 1:7
    subplot(4,2,i)
    yyaxis left
    % plot(torque(:,i)-tab{:,21+i},'LineWidth',1)
    plot(torque(:,i)-torque_ori(:,i),'LineWidth',1)
    yyaxis right
    plot(tab{:,7+i},'LineWidth',1)
    hold off
    title("Joint "+string(i))
end