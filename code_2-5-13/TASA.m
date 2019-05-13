function   TASA(Solu)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
%% 禁忌模拟退火算法优化任务总收益值 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T = 1000; % 初始温度为1000度
tasks_num = size(tasks,1);
r_num = size(subplan,1);
subplan_num= max(subplan(:,2));
Tabu1 = zeros(tasks_num,tasks_num); % 禁忌表1
Tabu2 = zeros(subplan_num,tasks_num); % 禁忌表2
K = 0.89; % 温度衰减参数
L = 20; % 马尔科夫链长度
TabuL = 3; % 禁忌表的长度
Pro_Tabu2 = 0.5; % 禁忌表2的禁忌概率
SO = zeros(L,tasks_num*sunplan_num); % 邻域解解集
% 各子规划中心已调度方案和未调度方案（二进制编码）;对应的任务收益值
% T1_schedule,T1_unschedule;T2_schedule,T2_unschedule;
% T3_schedule,T3_unschedule;T4_schedule,T4_unschedule。

% 获取T_Schedule,subplan_num行，tasks_num列，元素为1的为在相应的subplan已调度任务

% 获取T_unschedule,subplan_num行，tasks_num列，元素为1的为在相应的subplan未调度任务

best_so_far = Solu; % 当前最优解
best_so_far_fit =  profit1 + profit2 + profit3 + profit4; % 当前最优解对应的任务收益值
tasks_alloc_funs4 = task_alloc_funs4; % 函数脚本赋值
R_subplan = zeros(subplan_num,2); % 子规划中心的剩余资源（最大开关机次数、持续观测时间）
best_value = best_so_far_fit;
best_solution = best_so_far;
while T > 0.01
    for p = 1:L
        SO = zeros(L,tasks_num);
        SO_FIT = zeros(L,1);
        SO_cross = zeros(L,2);
        %% 邻域构造 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 获取tasks_Opportunity_subplan，任务在子规划中心上的观测机会
        tasks_Opportunity_subplan = zeros(subplan_num,tasks_num);
        tasks_Opportunity = [sats.sats_opp,uavs.opp]; 
        for i = 1:tasks_num
            for j = 1:r_num
               if tasks_Opportunity(i,j) ~= 0
                   tasks_Opportunity_subplan(subplan(j),2) = ...
                       tasks_Opportunity_subplan(subplan(j),2)+tasks_Opportunity(i,j);
               end
            end
        end
        
        [solu,task1,task2] = tasks_alloc_funs4.Neighborhood_structure_cross(T_schedule,tasks_Opportunity_subplan);
        % 已调度的任务在各子规划中心下消耗的资源
        for j = 1:subplan_num
            for i = 1:tasks_num
                R_subplan(j,1) = R_subplan(j,1)-solu(j,i); % 开关机次数
                R_subplan(j,2) = R_subplan(j,2)-solu(j,i)*tasks(i,7); % 持续观测时间
            end
        end
        
        %% 将未调度的任务按照最小冲突度(与已调度任务)的原则插入到对应的子规划中心
        solu = tasks_alloc_funs4.Neighborhood_structure_insertion(solu,T_unschedule,R_subplan,...
                Conflict_tasks,subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV);
        
        %% 根据solu得到SOLU %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        SOLU = zeros(subplan_num*tasks_num,1);
        for j = 1:subplan_num
            SOLU((j-1)*tasks_num:j*tasks_num,1) = solu(j,:);
        end
        
        SO(P,:) = SOLU;
        SO_FIT(P) = FUN(SOLU);
        SO_cross(p,:) = [task1,task2];
    end
    %% 藐视准则判断&&更新禁忌表 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 按适应度从大到小排序
    [SO_FIT,index] = sort(SO_FIT,'descend');
    SO = SO(index,:);
    SO_cross = SO_cross(index,:);
    %                  藐视准则                           %
    if SO_FIT(1) > best_so_far_fit
        best_so_far_fit = SO_FIT(1);
        best_so_far = SO(1);
        %                   更新禁忌表                       %     
        [Tabu1,Tabu2] = tasks_alloc_funs4.Re_Tabu(Tabu1,Tabu2,TabuL,1);
    else
        for i = 1:L
            if Tabu1(task1,task2)==0 && probabilty_Tabu2 < Pro_Tabu2
                best_so_far_fit = SO_FIT(i);
                best_so_far = SO(i);
                [Tabu1,Tabu2] = tasks_alloc_funs4.Re_Tabu(Tabu1,Tabu2,TabuL,i);
                break
            end
        end
    end
    
    %% 退火策略，接受更好解，以一定概率接受劣解 %%%%%%%%%%%%%%%%%%%%%%%%
    if best_so_far_fit < best_value 
        if exp(-1*abs(best_so_far_fit-best_so_far_fit_old)) > rand
            best_value = best_so_far_fit;
            best_solution = best_so_far;
            T = K*T;
        end
    else
        best_value = best_so_far_fit;
        best_solution = best_so_far;
        T = K*T;
    end 
end
end

%% 绘制图形 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
plot(best_value)
hold on;
xlabel("x");
ylabel("y");
title(["优化收益值：",num2str(best_value)]);
end
