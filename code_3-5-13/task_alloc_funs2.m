function funs = task_alloc_funs2
%用于无人机的数据处理
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
funs.distance_value = @distance_value; % 计算两个点之间的距离（经纬度）
funs.TW_tasks_uavs = @TW_tasks_uavs; % 计算无人机观测下的有效时间窗
funs.Opportunity = @Opportunity; % 计算无人机的观测机会
funs.Conflict_value_uavs = @Conflict_value_uavs; % 计算无人机的冲突度
funs.Conflict_Caculate = @Conflict_Caculate; %计算单个任务在无人机下的冲突度
funs.Conflict_Caculate_schedule = @Conflict_Caculate_schedule; %单个任务在无人机下与已调度任务的冲突度
funs.Probability_conflict_type1 = @Probability_conflict_type1; % 第一类交叉的冲突度计算
funs.Probability_conflict_type2 = @Probability_conflict_type2; % 第二类交叉的冲突度计算
funs.fit_addition = @fit_addition; % 计算附减的适应度
end
%% 子函数1，用来计算两个点之间的距离（经纬度）%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function distance = distance_value(t_pos1,t_pos2)
r = 6371;
distance = 2*r*asin(sqrt(sin(0.5*(t_pos1(1)-t_pos2(1))).^2+cos(t_pos1(1))*...
    cos(t_pos2(1))*sin(0.5*(t_pos1(2)-t_pos2(2))).^2));
end
%% 子函数2，计算无人机观测下的有效时间窗 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function TW = TW_tasks_uavs(tasks,base,V_UAV,D_UAV)
% 1.考虑最大航程约束；2.考虑任务的规定观测最早时间和最晚时间以及持续时间
tasks_num = size(tasks,1); % 任务的数量
t_pos = tasks(:,3:4); % 任务的位置点
SE_tasks = tasks(:,5:6); % 任务规定观测最早时间和最晚时间
duration = tasks(:,7); % 任务所需的持续观测时间
TW = zeros(tasks_num,2);
for i = 1:tasks_num
    % 计算基地与任务位置点之间的距离
    task_position = t_pos(i,:);
    distance = distance_value(base,task_position);
    % 最大距离判断，如果超过最大距离，时间窗为0
    if distance >= D_UAV
        TW(i,1) = 0;
        TW(i,2) = 0;
    else % 如果目标点在最大航程范围内
        tw_start = distance/V_UAV*3600; % 转换成单位s
        TW(i,1) = max([tw_start,SE_tasks(i,1)]);
        TW(i,2) = SE_tasks(i,2);
    % 如果更新的时间窗的开始时间加上持续观测时间大于结束时间，那么令为0向量。
        if  TW(i,2) - TW(i,1) < duration(i)    
            TW(i,1) = 0;
            TW(i,2) = 0;
        end
    end
end
end
%% 子函数3，计算任务在无人机下的观测机会 %%%%%%%%%%%%%%%%%%%%%%%%%%
function opp = Opportunity(tasks,UAVs_N,TW_uavs)
tasks_num = size(tasks,1); 
opp = zeros(tasks_num,UAVs_N); % 无人机基地对任务的观测机会
for i = 1:tasks_num
    if TW_uavs(i,1) > 0 || TW_uavs(i,2) > 0
        opp(i,:) = 1;
    else
        opp(i,:) = 0;
    end
end
end
%% 子函数4，计算任务在无人机下的冲突度 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Conflicts = Conflict_value_uavs(UAVs_opp,TW_uavs,tasks,base,V_UAV,D_UAV)
tasks_num = size(UAVs_opp,1); 
UAVs_N = size(UAVs_opp,2);
Conflicts = zeros(tasks_num,UAVs_N); % 各任务在各无人机下的冲突度矩阵
for i = 1:tasks_num
    if UAVs_opp(i,1) ~= 0
         % 任务i在无人机j的冲突度
         Conflicts(i,:) = Conflict_Caculate(TW_uavs,tasks,base,V_UAV,D_UAV,i);
    else % 如果任务i在无人机j上没有观测机会
        Conflicts(i,:) = inf; 
    end
end
end
%% 子函数5，计算单个任务在无人机下的冲突度 %%%%%%%%%%%%%%%%%%%%%%%%%%%
function conflict = Conflict_Caculate(TW_uavs,tasks,base,V_UAV,D_UAV,i)
conflict = 0;
t_pos = tasks(:,3:4);
duration = tasks(:,7);
W = tasks(:,2);
tasks_num = size(tasks,1);
distance3 = distance_value(base,t_pos(i,:)); % 任务i与基地之间的距离
tw1 = TW_uavs(i,:); % 任务i的有效时间窗
for p = 1:tasks_num
    if p == i || TW_uavs(p,1)+TW_uavs(p,2) == 0 % 时间窗为0向量即无观测机会
        continue
    else
        tw2 = TW_uavs(p,:); % 任务p的有效时间窗
        distance1 = distance_value(t_pos(p,:),t_pos(i,:)); % 任务p与任务i之间的距离
        time1 = distance1/V_UAV*3600; % 任务p与任务i之间的飞行时间
        distance2 = distance_value(base,t_pos(p,:)); % 任务p与基地之间的距离
        distance = distance1+distance2+distance3;
        % 最大距离判断（必定冲突）
        if distance > 2*D_UAV
            conflict = conflict + W(p);
        %  时间需求判断，（必定冲突判断）
        %  先执行完任务p,如果任务i的开始执行时间大于任务p的完成时间
        elseif TW_uavs(i,1) >= TW_uavs(p,2)
            if TW_uavs(p,1)+time1+duration(p) > TW_uavs(i,2)
                conflict = conflict + W(p);
                % （可能冲突判断），不相交的两个时间窗会存在任务冲突的情况
                % （可能冲突判断），相交的两个时间窗会存在任务冲突的情况
            elseif  TW_uavs(p,2)+time1+duration(p) > TW_uavs(i,1) &&...
                    TW_uavs(p,1)+time1+duration(p) <= TW_uavs(i,2)
                dur1 = duration(i);
                dur2 = duration(p)+time1;
                tw1(2) = tw1(2) - duration(i); % 任务i的最晚执行时刻
                tw2(2) = tw2(2) - duration(p); % 任务p的最晚执行时刻
                prob = Probability_conflict_type1(tw1,tw2,dur1,dur2);
                conflict = conflict + W(p)*prob;
            end
            %  时间需求判断，（必定冲突判断），先执行完任务i,如果任务p的开始执行时间大于任务i的完成时间
        elseif TW_uavs(p,1) >= TW_uavs(i,2)
            if TW_uavs(i,1)+time1+duration(i) > TW_uavs(p,2)
                conflict = conflict + W(p);
            % （可能冲突判断），不相交的两个时间窗会存在任务冲突的情况
            elseif  TW_uavs(i,2)+time1+duration(i) > TW_uavs(p,1) &&...
                    TW_uavs(i,1)+time1+duration(i) <= TW_uavs(p,2)
                dur1 = duration(i)+time1;
                dur2 = duration(p);
                tw1(2) = tw1(2) - duration(i); % 任务i的最晚执行时刻
                tw2(2) = tw2(2) - duration(p); % 任务p的最晚执行时刻
                prob = Probability_conflict_type1(tw1,tw2,dur1,dur2);
                conflict = conflict + W(p)*prob;
            end
        %  可能冲突判断，根据两个任务的交叉时间窗计算冲突可能性
        else
           dur1 = duration(i)+time1; 
           dur2 = duration(p)+time1;
           tw1(2) = tw1(2) - duration(i); % 任务i的最晚执行时刻
           tw2(2) = tw2(2) - duration(p); % 任务p的最晚执行时刻
           prob = Probability_conflict_type2(tw1,tw2,dur1,dur2);
           conflict = conflict + W(p)*prob;
        end 
    end
end
end
%% 子函数6，计算单个任务在无人机下与已调度任务的冲突度 %%%%%%%%%%%%%%%%%%
function conflict = Conflict_Caculate_schedule(TW_uavs,tasks,base,V_UAV,D_UAV,i,j,Schedule)
conflict = 0;
t_pos = tasks(:,3:4);
duration = tasks(:,7);
W = tasks(:,2);
tasks_num = size(tasks,1);
distance3 = distance_value(base,t_pos(i,:)); % 任务i与基地之间的距离
tw1 = TW_uavs(i,:); % 任务i的有效时间窗
for p = 1:tasks_num
    if p == i || TW_uavs(p,1)+TW_uavs(p,2) == 0 || Schedule(p,j) == 0 % 时间窗为0向量即无观测机会
        continue
    else
        tw2 = TW_uavs(p,:); % 任务p的有效时间窗
        distance1 = distance_value(t_pos(p,:),t_pos(i,:)); % 任务p与任务i之间的距离
        time1 = distance1/V_UAV*3600; % 任务p与任务i之间的飞行时间
        distance2 = distance_value(base,t_pos(p,:)); % 任务p与基地之间的距离
        distance = distance1+distance2+distance3;
        % 最大距离判断（必定冲突）
        if distance > 2*D_UAV
            conflict = conflict + W(p);
        %  时间需求判断，（必定冲突判断）
        %  先执行完任务p,如果任务i的开始执行时间大于任务p的完成时间
        elseif TW_uavs(i,1) >= TW_uavs(p,2)
            if TW_uavs(p,1)+time1+duration(p) > TW_uavs(i,2)
                conflict = conflict + W(p);
                % （可能冲突判断），不相交的两个时间窗会存在任务冲突的情况
                % （可能冲突判断），相交的两个时间窗会存在任务冲突的情况
            elseif  TW_uavs(p,2)+time1+duration(p) > TW_uavs(i,1) &&...
                    TW_uavs(p,1)+time1+duration(p) <= TW_uavs(i,2)
                dur1 = duration(i);
                dur2 = duration(p)+time1;
                tw1(2) = tw1(2) - duration(i); % 任务i的最晚执行时刻
                tw2(2) = tw2(2) - duration(p); % 任务p的最晚执行时刻
                prob = Probability_conflict_type1(tw1,tw2,dur1,dur2);
                conflict = conflict + W(p)*prob;
            end
            %  时间需求判断，（必定冲突判断），先执行完任务i,如果任务p的开始执行时间大于任务i的完成时间
        elseif TW_uavs(p,1) >= TW_uavs(i,2)
            if TW_uavs(i,1)+time1+duration(i) > TW_uavs(p,2)
                conflict = conflict + W(p);
            % （可能冲突判断），不相交的两个时间窗会存在任务冲突的情况
            elseif  TW_uavs(i,2)+time1+duration(i) > TW_uavs(p,1) &&...
                    TW_uavs(i,1)+time1+duration(i) <= TW_uavs(p,2)
                dur1 = duration(i)+time1;
                dur2 = duration(p);
                tw1(2) = tw1(2) - duration(i); % 任务i的最晚执行时刻
                tw2(2) = tw2(2) - duration(p); % 任务p的最晚执行时刻
                prob = Probability_conflict_type1(tw1,tw2,dur1,dur2);
                conflict = conflict + W(p)*prob;
            end
        %  可能冲突判断，根据两个任务的交叉时间窗计算冲突可能性
        else
           dur1 = duration(i)+time1; 
           dur2 = duration(p)+time1;
           tw1(2) = tw1(2) - duration(i); % 任务i的最晚执行时刻
           tw2(2) = tw2(2) - duration(p); % 任务p的最晚执行时刻
           prob = Probability_conflict_type2(tw1,tw2,dur1,dur2);
           conflict = conflict + W(p)*prob;
        end 
    end
end
end
%% 子函数7，不交叉的两个时间窗由于持续时间和准备时间的原因而可能导致冲突的可能性计算
% tw代表任务的有效可执行时间窗，考虑持续时间的影响。
function prob = Probability_conflict_type1(tw1,tw2,dur1,dur2)
S0 = abs(tw1(2)-tw1(1))*abs(tw2(2)-tw2(1)); % 矩形的面积
delta1= tw1(2)-tw2(2);
delta2 = tw1(1)-tw2(1);
if delta1*delta2 > 0 
    if delta1 > 0
        T12 = tw1(2);
        T11 = tw1(1);
        T22 = tw2(2);
        T21 = tw2(1);
        d1 = dur1;
        d2 = dur2;
    else
        T12 = tw2(2);
        T11 = tw2(1);
        T22 = tw1(2);
        T21 = tw1(1);
        d1 = dur2;
        d2 = dur1;
    end
    if T22 + d2 <= T11 % 约束直线在矩形区域外上侧
        prob = 0;
    else % 约束直线在矩形区域相交
        if T22 + d2 < T12 && T11 - d2 > T21 % 约束直线与左上边界成三角形
            S1 = 0.5*(T22+d2-T11)*(T22+d2-T11);
        elseif T22 + d2 < T12 && T11 - d2 <= T21 % 约束直线与上下边界成三角形 
            S1 = 0.5*(2*T22-T11+2*d2-T12)*(T12-T11);
        elseif T22 + d2 >= T12 % 约束直线在矩形区域外下侧，包含整个矩形
            S1 = S0;
        else % 约束直线与右下边界成三角形
            S1 = S0 - 0.5*(T12-d2-T21)*(T12-d2-T21);
        end
        prob = (S0-S1)/S0;
    end
end
end
%% 子函数8，在不考虑准备时间和持续时间的情况下，两个交叉的时间窗，计算冲突度
function prob = Probability_conflict_type2(tw1,tw2,dur1,dur2)
delta1= tw1(2)-tw2(2);
delta2 = tw1(1)-tw2(1);
S0 = abs(tw1(2)-tw1(1))*abs(tw2(2)-tw2(1)); % 矩形的面积
% 接下来求阴影部分的面积
% 如果小于0，说明有一个时间窗在另一个时间窗内,否则两个时间窗重叠一部分
if delta1*delta2 < 0
    if delta1 > 0 
        T12 = tw2(2);
        T11 = tw2(1);
        T22 = tw1(2);
        T21 = tw1(1);
        d1 = dur1;
        d2 = dur2;
    else
        T12 = tw1(2);
        T11 = tw1(1);
        T22 = tw2(2);
        T21 = tw2(1);
        d1 = dur2; 
        d2 = dur1;
    end
    if T21+d2 > T11 && T21+d2 < T12 % 形成三角形，不超过边界
        S2 = 0.5*(T12-T21-d2)*(T12-d2-T21);
    elseif T21+d2 <= T11
        S2 = 0.5*(T11-2*d2-2*T21+T12)*(T12-T11);
    elseif T21+d2 > T11 && T21+d2 >= T12 % 超过边界
        S2 = 0;
    end
    if T22-d1 < T12  && T11+d1 < T22 % 形成三角形，不超过边界。
        S1 = 0.5*(T22-T11-d1)*(T22-d1-T11);
    elseif T22-d1 >= T12  % 形成直角梯形
        S1 = 0.5*(2*T22-T12-T11-2*d1)*(T12-T11);
    elseif T22-d1 < T12 && T11+d1 >= T22
        S1 = 0;
    end
else
    if delta1 > 0
        T12 = tw1(2);
        T11 = tw1(1);
        T22 = tw2(2);
        T21 = tw2(1);
        d1 = dur1;
        d2 = dur2;
    else
        T12 = tw2(2);
        T11 = tw2(1);
        T22 = tw1(2);
        T21 = tw1(1);
        d1 = dur2;
        d2 = dur1;
    end
    if T11 + d1 < T22
        S1 = 0.5*(T22-T11-d1)*(T22-d1-T11);
    else
        S1 = 0;
    end
    if T11-d2 > T21
        S2 = 0.5*(T12-T11)*(T22-2*T21+T11-d2);
    else
        if d2+T22 < T12
            S2 = 0.5*(2*T12-2*d2-T22-T21)*(T22-T21);
        elseif d2+T22 >= T12 && T21+d2 < T12
            S2 = 0.5*(T12-d2-T21)*(T12-d2-T21);
        elseif d2+T22 >= T12 && T21+d2 >= T12
            S2 = 0;
        end
    end
end
prob = (S0-S1-S2)/S0; % 求两个时间窗之间的冲突发生概率
end
%% 子函数9，计算附加的适应度 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function fit = fit_addition(number,N_open_close)
if number > N_open_close
    fit = -20000000*(number-N_open_close);
else
    fit = 0;
end
end






