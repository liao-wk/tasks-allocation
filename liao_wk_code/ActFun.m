function [bestt,best_value_fit] = ActFun(iii)

%%  ���ó�ʼ������ʹ֮���пɶ��ԺͲ������Ӽ򵥷���  %%%%%%%%%%%%%%%
% �����������������
W = load('weight.txt'); % ������Ȩ��
W = W'+1; % Ҫ��Ȩ�ر������0
duration = load('duration.txt');
duration = duration'; % ����ĳ���ʱ��
lats = load('X.txt');
lons = load('Y.txt');
tasks_num = size(W,1); % ���������
st = load('Estime.txt');
le = load('letime.txt');
% ����������ͳһΪһ������
tasks = zeros(tasks_num,10);
tasks(:,1) = (1:tasks_num)'; % ��1������ı��
tasks(:,2) = W; % ��2�������Ȩ��
tasks(:,3) = lats'; % ������γ��
tasks(:,4) = lons'; % �����ľ���
tasks(:,5) = st'; % ������Ԥ�ȹ涨��ʼִ��ʱ��
tasks(:,6) = le'; % ������Ԥ�ȹ涨����ִ��ʱ��
tasks(:,7) = duration; % ����ĳ����۲�ʱ��
% ���ǲ���
sats.start = load('Stime.txt'); % �����ܹ۲⵽�������Ŀ�ʼʱ��
sats.end = load('Etime.txt'); % �����ܹ۲⵽������������ʱ��
sats_num = size(sats.start,2)/16; %���ǵ�����
% ���˻�����
base1 = [27/180*pi, 95/180*pi];% 2�����˻��Ļ��ص�γ�Ⱥ;���,ת���ɻ��ȵ���ʽ
base2 = [29/180*pi, 105/180*pi];
% base3 = [26/180*pi, 105/180*pi];
V_UAV = 1440; % ���˻��ķ����ٶ�400m/s,1440km/h��
D_UAV = 1000; % ���˻��������о���2000km
UAVs_N1 = 4; % ����1��4�����˻�
UAVs_N2 = 3; % ����2��3�����˻�
uavs_num = 7; % 7�����˻�
r_num = sats_num+UAVs_N1+UAVs_N2; % ��Դ������
R_remain = zeros(r_num,3); % ��Դ��ʣ����Դ���������ػ��������۲����ʱ�䡢����о���
R_remain(:,1) = 140; % ���ػ�������140��
R_remain(:,2) = 3000; % ������ʱ��3000s
R_remain(1:sats_num,3) = 0; % ����û�к���Լ��
R_remain(sats_num+1:end,3) = 2*D_UAV; % ���˻�����󺽳�Ϊ2��������о���
subplan = zeros(r_num,2); % �ӹ滮����
subplan(:,1) = (1:r_num)';
subplan(:,2) = [1;2;1;2;1;2;1;2;3;3;3;3;4;4;4];
subplan_num = 4;
clear lats;
clear lons;
clear st;
clear le;
tasks_alloc_funs = task_alloc_funs1; % �����ű���ֵ
tasks_alloc_funs2 = task_alloc_funs2; % �����ű���ֵ
tasks_alloc_funs3 = task_alloc_funs3; % �����ű���ֵ
%% �õ���ʼ�Ĵ��ھ���������������µĹ۲���� 
[sats.Windows,sats.sats_opp] = tasks_alloc_funs.Windows_Satsopp(sats,duration,...
    tasks(:,5:6));
% ���������������µ�����ȣ���ֵԽ�ߣ��۲����ԽС���������Դ�������Խ��
sats.tasks_value = tasks_alloc_funs.opp_devide_weight(sats_num,...
    sats.sats_opp,W);
%% ���������������µ���Чʱ�䴰����Щ������ĳ�������������ɸ�ʱ�䴰����Ԫ�����ʽ
sats.TW_tasks = tasks_alloc_funs.TW_tasks_sats(sats,tasks(:,5:6));
[sats.Stime,sats.Etime] = tasks_alloc_funs.Window_sats(sats,tasks(:,5:6),tasks(:,7));


%% ��������������Чʱ�䴰��������ĳ�����˻���ֻ��1��ʱ�䴰���Ծ������ʽ
% ��ͬ�Ļ��أ������Ӧ����Чʱ�䴰��һ��
uavs.TW_uavs1 = tasks_alloc_funs2.TW_tasks_uavs(tasks,base1,V_UAV,D_UAV);
uavs.TW_uavs2 = tasks_alloc_funs2.TW_tasks_uavs(tasks,base2,V_UAV,D_UAV);
%% �������������˻��Ĺ۲���ᣬ�������˻��Ļ���λ�úͺ��٣��������ж��Լ�ʱ�䴰�����жϣ�
uavs.opp1 = tasks_alloc_funs2.Opportunity(tasks,UAVs_N1,uavs.TW_uavs1);
uavs.opp2 = tasks_alloc_funs2.Opportunity(tasks,UAVs_N2,uavs.TW_uavs2);
uavs.opp = [uavs.opp1,uavs.opp2]; % ���񡪡����˻� �۲���������
% �õ����������˻��µ�����ȣ���ֵԽ�ߣ��۲����ԽС���������Դ�������Խ��
uavs.tasks_value = tasks_alloc_funs.opp_devide_weight(UAVs_N1+UAVs_N2,...
    uavs.opp,W);
%% �������������������Դ�µĹ۲�����ܺ��Լ������ 
tasks_Opportunity = [sats.sats_opp,uavs.opp]; 
tasks(:,8) = tasks_alloc_funs3.Opportunity_Sum(tasks_Opportunity); % ����Ĺ۲�����ܺ�
tasks(:,9) = tasks(:,8)./W; 



%% �����ͻ��
%% ���ǵĳ�ͻ�ȼ��㣬ʱ�䴰����н��棬��Ϊ��ͻ
Conflict_tasks.sats = zeros(tasks_num,sats_num);
for i = 1:tasks_num
    for j = 1:sats_num
        if ~isempty(sats.TW_tasks{i,j}) % �������i������j�ϵ�ʱ�䴰Ϊ��
            Conflict_tasks.sats(i,j) = tasks_alloc_funs.Conflict_value_sats(sats,i,j,W);
        else % �������i����Դj��û�й۲����
            Conflict_tasks.sats(i,j) = inf;
        end
    end
end 
clear i; clear j
% ���������������ϵ�ƽ����ͻ��
Conflict_tasks.sats_ave = tasks_alloc_funs3.Conflict_average(sats_num,sats.sats_opp,Conflict_tasks.sats);
%% ���˻��ĳ�ͻ�ȼ��㣬������Чʱ�䴰�ͳ���ʱ�䡢׼��ʱ������ͻ�����Դ�С 
% ע�⣬���˻��Ĺ۲����Ϊ0����Ӧ��ʱ�䴰Ϊ0����
Conflict_tasks.uavs1 = tasks_alloc_funs2.Conflict_value_uavs(uavs.opp1,uavs.TW_uavs1,tasks,base1,V_UAV,D_UAV);
Conflict_tasks.uavs2 = tasks_alloc_funs2.Conflict_value_uavs(uavs.opp2,uavs.TW_uavs2,tasks,base2,V_UAV,D_UAV);
Conflict_tasks.uavs = [Conflict_tasks.uavs1,Conflict_tasks.uavs2];
% ����ƽ����ͻ��
Conflict_tasks.uavs_ave = tasks_alloc_funs3.Conflict_average(UAVs_N1+UAVs_N2,uavs.opp,Conflict_tasks.uavs);
Conflict_tasks.average = [Conflict_tasks.sats_ave,Conflict_tasks.uavs_ave];



%% ��������Դ��ʣ������ֻ��������Ⱥͳ�ͻ��
%               utility = alph*value + beta*conflict                 %
% ������������tasks_value���й�һ������ͬһ������ĸ���ͻ��Ҳ���й�һ��
 %                         ��ʼ������                                    %
% Solution = tasks_alloc_funs3.Solution_generate(tasks,Conflict_tasks,R_remain,subplan);
solu2 = tasks_alloc_funs3.Solution_generate_average(tasks,Conflict_tasks,R_remain,subplan,sats,uavs,base1,base2,V_UAV,D_UAV);
%% ���ÿ���ӹ滮���ĵİ���������
T = zeros(subplan_num,1);
for i = 1:subplan_num
    for j = 1:tasks_num
        T(i,1) = T(i,1) + solu2(i,j);
    end
end
T_schedule = zeros(subplan_num,tasks_num); %���ѵ��ȵ����񼯣���������
T_unschedule = zeros(subplan_num,tasks_num);%��δ���ȵ����񼯣���������
best_so_far_fit2 = 0; 
% ��õ�ǰ���Ž������ֵ
for i = 1:subplan_num
    if i == 1
        [profit,T_schedule(i,:),T_unschedule(i,:)] = sats_func(solu2(i,:),sats,tasks,subplan,i);
    elseif i == 2
        [profit,T_schedule(i,:),T_unschedule(i,:)] = sats_func(solu2(i,:),sats,tasks,subplan,i);
    elseif i == 3
        [profit,T_schedule(i,:),T_unschedule(i,:)] = uavs_func(solu2(i,:),tasks,base1,UAVs_N1,D_UAV,V_UAV,R_remain(1,2),R_remain(1,1),tasks(:,7));
    else
        [profit,T_schedule(i,:),T_unschedule(i,:)] = uavs_func(solu2(i,:),tasks,base2,UAVs_N2,D_UAV,V_UAV,R_remain(1,2),R_remain(1,1),tasks(:,7));
    end
    best_so_far_fit2 = best_so_far_fit2 + profit;
end




%% ����㷨��ʵ�ֵ��� 
%% ����ģ���˻��㷨�Ż�����������ֵ 
tasks_Opportunity_subplan = zeros(subplan_num,tasks_num);
tasks_Opportunity = [sats.sats_opp,uavs.opp];
for i = 1:tasks_num
    for j = 1:r_num
        if tasks_Opportunity(i,j) ~= 0
            tasks_Opportunity_subplan(subplan(j,2),i) = ...
                tasks_Opportunity_subplan(subplan(j,2),i)+tasks_Opportunity(i,j);
        end
    end
end
% ������ɳ�ʼ��
solu = zeros(subplan_num,tasks_num);
for i = 1:tasks_num
    p_subs = randperm(subplan_num);
    while tasks_Opportunity_subplan(p_subs(1),i) == 0
        p_subs = randperm(subplan_num);
    end
    solu(p_subs(1),i) = 1;
end


T = 1000; % ��ʼ�¶�Ϊ1000��
tasks_num = size(tasks,1);
r_num = size(subplan,1);
subplan_num= max(subplan(:,2));
% Tabu1 = zeros(tasks_num,tasks_num); % ���ɱ�1
Tabu3 = zeros(subplan_num,tasks_num); % ���ɱ�3
K = 0.89; % �¶�˥������
L = 30; % ����Ʒ�������
% TabuL1 = 3; % ���ɱ�1�ĳ���
TabuL2 = 3; % ���ɱ�2�ĳ���
TabuL3 = 3; % ���ɱ�3�ĳ���
% ���ӹ滮�����ѵ��ȷ�����δ���ȷ����������Ʊ��룩;��Ӧ����������ֵ
% ��ȡT_unschedule,subplan_num�У�tasks_num�У�Ԫ��Ϊ1��Ϊ����Ӧ��subplanδ��������
T_schedule = zeros(subplan_num,tasks_num); %���ѵ��ȵ����񼯣���������
T_unschedule = zeros(subplan_num,tasks_num);%��δ���ȵ����񼯣���������
best_so_far = solu; % ��ǰ���Ž�
best_so_far_fit = 0; 
% ��õ�ǰ���Ž������ֵ
for i = 1:subplan_num
    if i == 1
        [profit,T_schedule(i,:),T_unschedule(i,:)] = sats_func(solu(i,:),sats,tasks,subplan,i);
    elseif i == 2
        [profit,T_schedule(i,:),T_unschedule(i,:)] = sats_func(solu(i,:),sats,tasks,subplan,i);
    elseif i == 3
        [profit,T_schedule(i,:),T_unschedule(i,:)] = uavs_func(solu(i,:),tasks,base1,UAVs_N1,D_UAV,V_UAV,R_remain(1,2),R_remain(1,1),tasks(:,7));
    else
        [profit,T_schedule(i,:),T_unschedule(i,:)] = uavs_func(solu(i,:),tasks,base2,UAVs_N2,D_UAV,V_UAV,R_remain(1,2),R_remain(1,1),tasks(:,7));
    end
    best_so_far_fit = best_so_far_fit + profit;
end
tasks_alloc_funs4 = task_alloc_funs4; % �����ű���ֵ
Tabu2 = TabuL2*T_unschedule; % ���ɱ�2��ֵ
best_value = [best_so_far_fit];
best_solution = best_so_far;
best_far_fit = best_so_far_fit;
while T > 700
    for p = 1:L
        %% ������ 
        % ��ȡtasks_Opportunity_subplan���������ӹ滮�����ϵĹ۲����
        R_subplan = zeros(subplan_num,2); % �ӹ滮���ĵ�ʣ����Դ����󿪹ػ������������۲�ʱ�䣩
        for m = 1:r_num
            agent_code = subplan(m,2); % ��������
            R_subplan(agent_code,1) = R_subplan(agent_code,1)+R_remain(1,1);
            R_subplan(agent_code,2) = R_subplan(agent_code,2)+R_remain(1,2);
        end
        clear agent_code;
        %% ����ṹ����
%         [solu,task1,task2] = tasks_alloc_funs4.Neighborhood_structure_cross(T_schedule,Tabu1,tasks_Opportunity_subplan,tasks);
        %% ����ṹɾ���ѵ��ȵ�����
        [T_schedule,T_unschedule,p_sub,task3] =  tasks_alloc_funs4.Neighborhood_structure_delete(T_schedule,...
    T_unschedule,tasks,subplan,tasks_Opportunity,sats,uavs,base1,base2,V_UAV,D_UAV,Tabu3);
        % �ѵ��ȵ������ڸ��ӹ滮���������ĵ���Դ
        for j = 1:subplan_num
            for i = 1:tasks_num
                R_subplan(j,1) = R_subplan(j,1)-T_schedule(j,i); % ���ػ�����
                R_subplan(j,2) = R_subplan(j,2)-T_schedule(j,i)*tasks(i,7); % �����۲�ʱ��
            end
        end
        
        %% ��δ���ȵ���������С��ͻ��(���ѵ�������)��ԭ����뵽��Ӧ���ӹ滮����
        solu = tasks_alloc_funs4.Neighborhood_structure_insertion(solu,T_unschedule,R_subplan,...
                Conflict_tasks,subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV,Tabu2);
        
    %% ���½��ɱ�&&�˻����
    current_fit = 0; % ��ǰ�����Ӧ��
    for i = 1:subplan_num
        if i == 1
            [profit,T_schedule(i,:),T_unschedule(i,:)] = sats_func(solu(i,:),sats,tasks,subplan,i);
        elseif i == 2
            [profit,T_schedule(i,:),T_unschedule(i,:)] = sats_func(solu(i,:),sats,tasks,subplan,i);
        elseif i == 3
            [profit,T_schedule(i,:),T_unschedule(i,:)] = uavs_func(solu(i,:),tasks,base1,UAVs_N1,D_UAV,V_UAV,R_remain(1,2),R_remain(1,1),tasks(:,7));
        else
            [profit,T_schedule(i,:),T_unschedule(i,:)] = uavs_func(solu(i,:),tasks,base2,UAVs_N2,D_UAV,V_UAV,R_remain(1,2),R_remain(1,1),tasks(:,7));
        end
        current_fit = current_fit + profit;
    end

    if current_fit > best_so_far_fit 
        best_so_far_fit = current_fit;
        best_so_far = solu;
        best_far_fit = [best_far_fit,best_so_far_fit];
    else
        delta = abs(best_so_far_fit-current_fit);
        if exp(-1*delta/T) > rand
            best_so_far_fit = current_fit;
            best_so_far = solu;
            best_far_fit = [best_far_fit,best_so_far_fit];
        end
    end
    if best_value(end) < best_so_far_fit
        best_value(end+1) = best_so_far_fit;
        best_solution = solu;
    end
%     % ���ɱ�1�ĸ���
%     for m = 1:tasks_num
%         for n = 1:tasks_num
%             if Tabu1(m,n) ~= 0
%                 Tabu1(m,n) = Tabu1(m,n)-1;
%                 Tabu1(n,m) = Tabu1(n,m)-1;
%             end
%         end
%     end
%     Tabu1(task1,task2) = TabuL1; % ��ӽ��ɶ���
%     Tabu1(task2,task1) = TabuL1; % ��ӽ��ɶ���
    % ���ɱ�2�ĸ���
     for m = 1:subplan_num
        for n = 1:tasks_num
            if T_unschedule(m,n) ~= 0 
                Tabu2(m,n) = TabuL2;
            else
                if Tabu2(m,n) ~= 0
                    Tabu2(m,n) = Tabu2(m,n)-1;
                end
            end
        end
     end
     % ���ɱ�3�ĸ���
     for m = 1:subplan_num
        for n = 1:tasks_num
            if Tabu3(m,n) ~= 0
                Tabu3(m,n) = Tabu3(m,n)-1;
            end
        end
     end
     Tabu3(p_sub,task3) = TabuL3; % ��ӽ��ɶ���
    end
    T = T*K; % �¶�˥��
end

len = size(best_far_fit,2);
best_value_list = best_far_fit(1);
for i = 1:len
    if best_far_fit(i) > best_value_list(end)
        best_value = best_far_fit(i);
    else
        best_value = best_value_list(end);
    end
    best_value_list(end+1) = best_value;
end
figure(1)
plot(best_value_list);
hold on;
xlabel("x/��������");
ylabel("y/��Ӧ��");
title(["��ʷ�Ż�����ֵ��",num2str(best_value_list(end))]);
filename = ['myfig',num2str(iii*10+1),'.jpg'];
saveas(gcf,filename);
close;
figure(2)
plot(best_far_fit);
xlabel('x/��������');
ylabel('y/��Ӧ��');
title(['�Ż�����ֵ��',num2str(best_value(end))]);
filename = ['myfig',num2str(iii*10+2),'.jpg'];
saveas(gcf,filename);
close;

bestt = best_solution(1,:);
for i = 2:subplan_num
	bestt = [bestt,best_solution(i,:)];
end
best_value_fit = best_value(end);



