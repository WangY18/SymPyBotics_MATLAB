%% 
addpath("dyn_model_panda\")
%% 
tab = readtable('..\data\Exp_20250612\feedback_filter.csv');
%% 
torque_baseline = zeros(size(tab,1),7);
for i=1:size(tab,1)
    if mod(i,1000)==0
        [i,size(tab,1)]
    end
    q = tab{i,1:7}';
    dq = tab{i,8:14}';
    ddq = tab{i,15:21}';
    
    g = get_GravityVector(q);
    c = get_CoriolisVector(q,dq);
    M = get_MassMatrix(q);
    tauf = get_FrictionTorque(dq);
    
    torque_baseline(i,:) = M*ddq + c + g + tauf;
    
    % equivalently, you could use (decomment) the following two lines:
%     Cmat = get_CoriolisMatrix(q,dq);
%     TAU(:,i) = M*ddq + Cmat*dq + g + tauf;
end
%% 
save("data\baseline.mat","torque_baseline")
%% 
figure
for i = 1:7
    subplot(4,2,i)
    hold on
    plot(tab{:,21+i},'b','LineWidth',1)
    % plot(torque_ori(:,i),'b','LineWidth',1)
    plot(torque_baseline(:,i),'r','LineWidth',1)
    hold off
    title("Joint "+string(i))
end
lg = legend(["Fitted Torque","Measured Torque"],'Position',[0.5853    0.1772    0.0690    0.0339]);