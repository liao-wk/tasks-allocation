function [bestt,best_value_fit] = ActFun(iii)

%%  设置初始参数，使之具有可读性和操作更加简单方便  %%%%%%%%%%%%%%%
% 随机生成任务点的属性
W = load('weight.txt'); % 任务点的权重
W = W'+1; % 要求权重必须大于0
duration = load('duration.txt');
duration = duration'; % 任务的持续时间
lats = load('X.txt');
lons = load('Y.txt');
tasks_num = size(W,1); % 任务的数量
st = load('Estime.txt');
le = load('letime.txt');
% 任务点的属性统一为一个矩阵
tasks = zeros(tasks_num,10);
tasks(:,1) = (1:tasks_num)'; % 第1列任务的编号
tasks(:,2) = W; % 第2列任务的权重
tasks(:,3) = lats'; % 任务点的纬度
tasks(:,4) = lons'; % 任务点的经度
tasks(:,5) = st'; % 任务点的预先规定开始执行时间
tasks(:,6) = le'; % 任务点的预先规定最晚执行时间
tasks(:,7) = duration; % 任务的持续观测时间
% 卫星部分
sats.start = load('Stime.txt'); % 卫星能观测到这个任务的开始时刻
sats.end = load('Etime.txt'); % 卫星能观测到这个任务的最晚时刻
sats_num = size(sats.start,2)/16; %卫星的数量
% 无人机部分
base1 = [27/180*pi, 95/180*pi];% 2个无人机的基地的纬度和经度,转换成弧度的形式
base2 = [29/180*pi, 105/180*pi];
% base3 = [26/180*pi, 105/180*pi];
V_UAV = 1440; % 无人机的飞行速度400m/s,1440km/h。
D_UAV = 1000; % 无人机的最大飞行距离2000km
UAVs_N1 = 4; % 基地1有4架无人机
UAVs_N2 = 3; % 基地2有3架无人机
uavs_num = 7; % 7架无人机
r_num = sats_num+UAVs_N1+UAVs_N2; % 资源的总数
R_remain = zeros(r_num,3); % 资源的剩余资源数量：开关机次数、观测持续时间、最大航行距离
R_remain(:,1) = 140; % 开关机最大次数140次
R_remain(:,2) = 3000; % 最大持续时间3000s
R_remain(1:sats_num,3) = 0; % 卫星没有航程约束
R_remain(sats_num+1:end,3) = 2*D_UAV; % 无人机的最大航程为2倍的最大航行距离
subplan = zeros(r_num,2); % 子规划中心
subplan(:,1) = (1:r_num)';
subplan(:,2) = [1;2;1;2;1;2;1;2;3;3;3;3;4;4;4];
subplan_num = 4;
clear lats;
clear lons;
clear st;
clear le;
tasks_alloc_funs = task_alloc_funs1; % 函数脚本赋值
tasks_alloc_funs2 = task_alloc_funs2; % 函数脚本赋值
tasks_alloc_funs3 = task_alloc_funs3; % 函数脚本赋值
%% 得到初始的窗口矩阵和任务在卫星下的观测机会 
[sats.Windows,sats.sats_opp] = tasks_alloc_funs.Windows_Satsopp(sats,duration,...
    tasks(:,5:6));
% 计算任务在卫星下的需求度，价值越高，观测机会越小的任务对资源的需求度越高
sats.tasks_value = tasks_alloc_funs.opp_devide_weight(sats_num,...
    sats.sats_opp,W);
%% 计算任务在卫星下的有效时间窗，有些任务在某颗卫星下有若干个时间窗，以元组的形式
sats.TW_tasks = tasks_alloc_funs.TW_tasks_sats(sats,tasks(:,5:6));
[sats.Stime,sats.Etime] = tasks_alloc_funs.Window_sats(sats,tasks(:,5:6),tasks(:,7));


%% 计算各个任务的有效时间窗，任务在某架无人机下只有1个时间窗，以矩阵的形式
% 不同的基地，任务对应的有效时间窗不一样
uavs.TW_uavs1 = tasks_alloc_funs2.TW_tasks_uavs(tasks,base1,V_UAV,D_UAV);
uavs.TW_uavs2 = tasks_alloc_funs2.TW_tasks_uavs(tasks,base2,V_UAV,D_UAV);
%% 计算任务在无人机的观测机会，根据无人机的基地位置和航速（最大距离判断以及时间窗长度判断）
uavs.opp1 = tasks_alloc_funs2.Opportunity(tasks,UAVs_N1,uavs.TW_uavs1);
uavs.opp2 = tasks_alloc_funs2.Opportunity(tasks,UAVs_N2,uavs.TW_uavs2);
uavs.opp = [uavs.opp1,uavs.opp2]; % 任务——无人机 观测机会的整合
% 得到任务在无人机下的需求度，价值越高，观测机会越小的任务对资源的需求度越高
uavs.tasks_value = tasks_alloc_funs.opp_devide_weight(UAVs_N1+UAVs_N2,...
    uavs.opp,W);
%% 计算各个任务在所有资源下的观测机会总和以及需求度 
tasks_Opportunity = [sats.sats_opp,uavs.opp]; 
tasks(:,8) = tasks_alloc_funs3.Opportunity_Sum(tasks_Opportunity); % 任务的观测机会总和
tasks(:,9) = tasks(:,8)./W; 



%% 计算冲突度
%% 卫星的冲突度计算，时间窗如果有交叉，视为冲突
Conflict_tasks.sats = zeros(tasks_num,sats_num);
for i = 1:tasks_num
    for j = 1:sats_num
        if ~isempty(sats.TW_tasks{i,j}) % 如果任务i在卫星j上的时间窗为真
            Conflict_tasks.sats(i,j) = tasks_alloc_funs.Conflict_value_sats(sats,i,j,W);
        else % 如果任务i在资源j上没有观测机会
            Conflict_tasks.sats(i,j) = inf;
        end
    end
end 
clear i; clear j
% 计算任务在卫星上的平均冲突度
Conflict_tasks.sats_ave = tasks_alloc_funs3.Conflict_average(sats_num,sats.sats_opp,Conflict_tasks.sats);
%% 无人机的冲突度计算，依据有效时间窗和持续时间、准备时间计算冲突可能性大小 
% 注意，无人机的观测机会为0，对应的时间窗为0向量
Conflict_tasks.uavs1 = tasks_alloc_funs2.Conflict_value_uavs(uavs.opp1,uavs.TW_uavs1,tasks,base1,V_UAV,D_UAV);
Conflict_tasks.uavs2 = tasks_alloc_funs2.Conflict_value_uavs(uavs.opp2,uavs.TW_uavs2,tasks,base2,V_UAV,D_UAV);
Conflict_tasks.uavs = [Conflict_tasks.uavs1,Conflict_tasks.uavs2];
% 计算平均冲突度
Conflict_tasks.uavs_ave = tasks_alloc_funs3.Conflict_average(UAVs_N1+UAVs_N2,uavs.opp,Conflict_tasks.uavs);
Conflict_tasks.average = [Conflict_tasks.sats_ave,Conflict_tasks.uavs_ave];



%% 不考虑资源的剩余量，只考虑需求度和冲突度
%               utility = alph*value + beta*conflict                 %
% 对任务的需求度tasks_value进行归一化，对同一个任务的各冲突度也进行归一化
 %                         初始解的求解                                    %
% Solution = tasks_alloc_funs3.Solution_generate(tasks,Conflict_tasks,R_remain,subplan);
solu2 = tasks_alloc_funs3.Solution_generate_average(tasks,Conflict_tasks,R_remain,subplan,sats,uavs,base1,base2,V_UAV,D_UAV);
%% 检测每个子规划中心的安排任务数
T = zeros(subplan_num,1);
for i = 1:subplan_num
    for j = 1:tasks_num
        T(i,1) = T(i,1) + solu2(i,j);
    end
end
T_schedule = zeros(subplan_num,tasks_num); %　已调度的任务集，０１编码
T_unschedule = zeros(subplan_num,tasks_num);%　未调度的任务集，０１编码
best_so_far_fit2 = 0; 
% 获得当前最优解的收益值
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




%% 设计算法，实现迭代 
%% 禁忌模拟退火算法优化任务总收益值 
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
% 随机生成初始解
solu = zeros(subplan_num,tasks_num);
for i = 1:tasks_num
    p_subs = randperm(subplan_num);
    while tasks_Opportunity_subplan(p_subs(1),i) == 0
        p_subs = randperm(subplan_num);
    end
    solu(p_subs(1),i) = 1;
end


T = 1000; % 初始温度为1000度
tasks_num = size(tasks,1);
r_num = size(subplan,1);
subplan_num= max(subplan(:,2));
% Tabu1 = zeros(tasks_num,tasks_num); % 禁忌表1
Tabu3 = zeros(subplan_num,tasks_num); % 禁忌表3
K = 0.89; % 温度衰减参数
L = 30; % 马尔科夫链长度
% TabuL1 = 3; % 禁忌表1的长度
TabuL2 = 3; % 禁忌表2的长度
TabuL3 = 3; % 禁忌表3的长度
% 各子规划中心已调度方案和未调度方案（二进制编码）;对应的任务收益值
% 获取T_unschedule,subplan_num行，tasks_num列，元素为1的为在相应的subplan未调度任务
T_schedule = zeros(subplan_num,tasks_num); %　已调度的任务集，０１编码
T_unschedule = zeros(subplan_num,tasks_num);%　未调度的任务集，０１编码
best_so_far = solu; % 当前最优解
best_so_far_fit = 0; 
% 获得当前最优解的收益值
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
tasks_alloc_funs4 = task_alloc_funs4; % 函数脚本赋值
Tabu2 = TabuL2*T_unschedule; % 禁忌表2赋值
best_value = [best_so_far_fit];
best_solution = best_so_far;
best_far_fit = best_so_far_fit;
while T > 700
    for p = 1:L
        %% 邻域构造 
        % 获取tasks_Opportunity_subplan，任务在子规划中心上的观测机会
        R_subplan = zeros(subplan_num,2); % 子规划中心的剩余资源（最大开关机次数、持续观测时间）
        for m = 1:r_num
            agent_code = subplan(m,2); % 智能体编号
            R_subplan(agent_code,1) = R_subplan(agent_code,1)+R_remain(1,1);
            R_subplan(agent_code,2) = R_subplan(agent_code,2)+R_remain(1,2);
        end
        clear agent_code;
        %% 邻域结构交换
%         [solu,task1,task2] = tasks_alloc_funs4.Neighborhood_structure_cross(T_schedule,Tabu1,tasks_Opportunity_subplan,tasks);
        %% 邻域结构删除已调度的任务
        [T_schedule,T_unschedule,p_sub,task3] =  tasks_alloc_funs4.Neighborhood_structure_delete(T_schedule,...
    T_unschedule,tasks,subplan,tasks_Opportunity,sats,uavs,base1,base2,V_UAV,D_UAV,Tabu3);
        % 已调度的任务在各子规划中心下消耗的资源
        for j = 1:subplan_num
            for i = 1:tasks_num
                R_subplan(j,1) = R_subplan(j,1)-T_schedule(j,i); % 开关机次数
                R_subplan(j,2) = R_subplan(j,2)-T_schedule(j,i)*tasks(i,7); % 持续观测时间
            end
        end
        
        %% 将未调度的任务按照最小冲突度(与已调度任务)的原则插入到对应的子规划中心
        solu = tasks_alloc_funs4.Neighborhood_structure_insertion(solu,T_unschedule,R_subplan,...
                Conflict_tasks,subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV,Tabu2);
        
    %% 更新禁忌表&&退火策略
    current_fit = 0; % 当前解的适应度
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
%     % 禁忌表1的更新
%     for m = 1:tasks_num
%         for n = 1:tasks_num
%             if Tabu1(m,n) ~= 0
%                 Tabu1(m,n) = Tabu1(m,n)-1;
%                 Tabu1(n,m) = Tabu1(n,m)-1;
%             end
%         end
%     end
%     Tabu1(task1,task2) = TabuL1; % 添加禁忌对象
%     Tabu1(task2,task1) = TabuL1; % 添加禁忌对象
    % 禁忌表2的更新
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
     % 禁忌表3的更新
     for m = 1:subplan_num
        for n = 1:tasks_num
            if Tabu3(m,n) ~= 0
                Tabu3(m,n) = Tabu3(m,n)-1;
            end
        end
     end
     Tabu3(p_sub,task3) = TabuL3; % 添加禁忌对象
    end
    T = T*K; % 温度衰减
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
xlabel("x/迭代次数");
ylabel("y/适应度");
title(["历史优化收益值：",num2str(best_value_list(end))]);
filename = ['myfig',num2str(iii*10+1),'.jpg'];
saveas(gcf,filename);
close;
figure(2)
plot(best_far_fit);
xlabel('x/迭代次数');
ylabel('y/适应度');
title(['优化收益值：',num2str(best_value(end))]);
filename = ['myfig',num2str(iii*10+2),'.jpg'];
saveas(gcf,filename);
close;

bestt = best_solution(1,:);
for i = 2:subplan_num
	bestt = [bestt,best_solution(i,:)];
end
best_value_fit = best_value(end);



