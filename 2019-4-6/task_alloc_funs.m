function funs = task_alloc_funs
%UNTITLED6 此处显示有关此函数的摘要
%   此处显示详细说明
funs.distance_value = @distance_value;
funs.time_windows = @time_windows; % 计算单个任务在单个卫星上的有效时间窗
funs.TW_tasks = @TW_tasks; % 计算各个在各个卫星上的有效时间窗
funs.Windows_Sats_opp = @Windows_Sats_opp; % 计算Windows矩阵和任务的观测机会矩阵
funs.Conflict_degree1 = @Conflict_degree1; % 计算卫星观测下任务的冲突度大小
funs.Conflict_degree2 = @Conflict_degree2; % 计算无人机观测下任务的冲突度大小
funs.Judge_Dis_TW = @Judge_Dis_TW; % 无人机的最大距离和时间窗判断
funs.Probability_tasks = @Probability_tasks; % 计算两个交叉的时间窗的冲突概率
funs.TW_tasks2 = @TW_tasks2; % 计算各个任务在各架无人机上的有效时间窗
funs.UAVs_opp = @UAVs_opp; % 计算各个任务在各家无人机上的观测机会
end
%% 子函数1，根据经纬度求任务点到基地的距离
function distance = distance_value(base,t_pos)
r = 6371;
distance = 2*r*asin(sqrt(sin(0.5*(base(1)-t_pos(1))).^2+cos(base(1))*...
    cos(t_pos(1))*sin(0.5*(base(2)-t_pos(2))).^2));
end
%% 子函数2，计算任务的时间窗口
function tw = time_windows(SE_tasks,SE_sats,i,j)
tw = [max([SE_tasks(i,1),SE_sats.start(i,j)]),min([SE_tasks(i,2),...
    SE_sats.end(i,j)])];
end
%% 子函数3，生成任务的卫星时间窗矩阵
function TW_tasks = TW_tasks(sats_opp,SE_sats,SE_tasks)
size_W = size(SE_sats.Windows); %Windows矩阵的大小
sats_num = size_W(2)/16; %卫星的个数
tasks_num = size_W(1); % 任务的个数
size_col = 2; % tw时间窗的列数,一般只有开始时间和结束时间
% 给TW_tasks赋初值
TW_tasks = cell(tasks_num,sats_num);
% 卫星观测的机会矩阵中最大的值,作为tw的最大维数
size_max = max(max(sats_opp))+1;
% 一个任务在一颗卫星个观测下可能有多个时间窗，因此要提前赋一个向量空间
% 先排任务，再排卫星
for i = 1:tasks_num
    for p = 1:sats_num  % 卫星的编号
        q = 1; % 任务的有效时间窗矩阵的索引
        % 对某一任务在某卫星上的时间窗tw赋一个向量空间
        tw = zeros(size_max,size_col);
        for j = 16*(p-1)+1:16*(p-1)+16
            % 如果在一颗卫星的观测下（16列），存在Windows中的元素大于0，
            %  根据i,j位置找到有效时间窗
            if SE_sats.Windows(i,j) > 0
                tw(q, 1:end) = time_windows(SE_tasks,SE_sats,i,j);
                q = q+1;
            end
        end
        % 在执行完一颗卫星的周期（16列）后，删除tw的零行。
        tw(q:end,:) = [];
        TW_tasks{i,p} = tw;
    end
end
end
%% 子函数4，如果卫星在某个任务的观测时间为有效时间，则用卫星
% 观测的结束时间减去开始时间
function [Windows,sats_opp] = Windows_Sats_opp(SE_sats,duration,SE_tasks)
Windows = SE_sats.end - SE_sats.start; % 预先分配内存
size_t = size(SE_sats.start); %
tasks_num = size_t(1); % 任务的个数
size_W = size(Windows); % 时间窗口矩阵的大小
if tasks_num ~= size_W(1)
    disp("卫星遗漏了部分任务的观测结果数据")
end
% 判断卫星是否在有效时间段内观测到任务，如果不是，令相应的窗口长度为0,
% 如果是，令卫星观测的结束时间减去开始时间
% 根据改变的窗口矩阵，确定任务在卫星下的观测机会有多少次
sats_opp = zeros(tasks_num,size_W(2)/16); % 任务i被卫星观测的机会，1表示可以被观测，否则为0
for i = 1:tasks_num  % 任务的个数
    for j = 1:size_W(2)  % 全部卫星的环绕次数，1个卫星绕地16次
        % 如果卫星的观测时间是有效观测时间，跳过，否则令它为0
        if SE_sats.end(i,j) >= duration(i)+SE_tasks(i,1) &&...
                SE_sats.start(i,j) <= SE_tasks(i,2)-duration(i)
            Windows(i,j) = SE_sats.end(i,j) - SE_sats.start(i,j);
        else
            Windows(i,j) = 0;
        end
    end
    indexes = find(Windows(i,1:end) > 0);
    % 一个时间窗为一个观测机会
    size_indexes = size(indexes);
    for k = 1:size_indexes(2)
        if sats_opp(i,floor(indexes(k)/16)+1) == 0
            sats_opp(i,floor(indexes(k)/16)+1) = 1;
        else
            sats_opp(i,floor(indexes(k)/16)+1) = sats_opp(i,floor(indexes(k)/16)+1)+1;
        end
    end
end
end
%% 子函数5，计算卫星观测下冲突度的大小，并将与任务i必然冲突和可能冲突的任务q记录下来
%   暂时先不将必然冲突记录下来
function Conflict = Conflict_degree1(i,j,SE_sats,W,tasks_num,duration)
m = size(SE_sats.TW_tasks{i,j},1); % 计算任务i在卫星j上有m个时间窗
Conflict = zeros(m,1);
for p = 1:m % p代表任务i在卫星j上的第p个时间窗
    tw_obj = SE_sats.TW_tasks{i,j}(p,:); % 表示任务i在卫星j上的第p个时间窗
    for q = 1:tasks_num
        prob = 0; % 初始化任务i与任务q的冲突概率
        % 如果其他任务不存在时间窗或其他任务为任务i
        if q == i || isempty(SE_sats.TW_tasks{q,j})
            continue
        else
            tw_tasks = SE_sats.TW_tasks{q,j}; % 时间窗可能有多个
            k = size(tw_tasks,1); % 得到任务q的时间窗的行数
            for x = 1:k
                tw = tw_tasks(x,:); % 比较的任务的第x个时间窗
                % 任务i与任务q的冲突概率计算，再用冲突概率乘以任务q的价值得到冲突度
                if tw(2)-tw(1)+tw_obj(2)-tw_obj(1)-(max([tw(2),tw_obj(2)])-...
                        min([tw(1),tw_obj(1)])) > 0
                    prob = Probability_tasks(tw,tw_obj,duration,i,q);
                    %  冲突度累加
                    Conflict(p,1) = Conflict(p,1)+prob*W(q)/k;
                end
            end  
        end
    end
end
% 取任务i在卫星j上所有有效时间窗的最小冲突度
Conflict = min(Conflict);
end
%% 子函数6，计算无人机观测下任务的冲突度大小
function Conflict = Conflict_degree2(tasks_num,UAVs_N,UAVs_opp,D_UAV,base,t_pos,V_UAV,SE_tasks,duration,W)
Conflict = zeros(tasks_num,UAVs_N);
% 最大距离冲突判断,如果不满足，进行最大时间窗判断
for i = 1:tasks_num
    if UAVs_opp(i,1) ~= 0
        % 任务i在无人机j的冲突度
        Conflict(i,:) = Judge_Dis_TW(i,tasks_num,UAVs_opp,base,D_UAV,t_pos,V_UAV,SE_tasks,duration,W); 
    end
end
end
%% 子函数7，最大距离和时间窗判断,SE_tasks表示任务在无人机下的有效时间窗
function conflict = Judge_Dis_TW(i,tasks_num,UAVs_opp,base,D_UAV,t_pos,V_UAV,SE_tasks,duration,W)
conflict = 0;
distance3 = distance_value(base,t_pos(i,:)); % 任务i与基地之间的距离
for p = 1:tasks_num
    if p == i || UAVs_opp(p,1) == 0
        continue
    else
        distance1 = distance_value(t_pos(p,:),t_pos(i,:)); % 任务p与任务i之间的距离
        time1 = distance1/V_UAV*3600; % 任务p与任务i之间的飞行时间
        distance2 = distance_value(base,t_pos(p,:)); % 任务p与基地之间的距离
        distance = distance1+distance2+distance3;
        % 最大距离判断（必定冲突）
        if distance > 2*D_UAV
            conflict = conflict + W(p);
        %  时间需求判断，（必定冲突判断）
        %  先执行完任务p,如果任务i的开始执行时间大于任务p的完成时间
        elseif SE_tasks(i,1) >= SE_tasks(p,2)
            if SE_tasks(p,1)+time1+duration(p) > SE_tasks(i,2)
                conflict = conflict + W(p);
            end
        %  时间需求判断，（必定冲突判断），先执行完任务i,如果任务p的开始执行时间大于任务i的完成时间
        elseif SE_tasks(p,1) >= SE_tasks(i,2)
            if SE_tasks(i,1)+time1+duration(i) > SE_tasks(p,2)
                conflict = conflict + W(p);
            end
        %  可能冲突判断，根据两个任务的时间窗
        else 
            prob = Probability_tasks(SE_tasks(i,:),SE_tasks(p,:),duration,i,p);
            conflict = conflict + W(p)*prob;
        end
    end
end
end
%% 子函数8，计算两个交叉的时间窗冲突的概率
function prob = Probability_tasks(tw,tw_obj,duration,i,q)
delta1= tw(2)-tw_obj(2);
delta2 = tw(1)-tw_obj(1);
S0 = abs(tw(2)-tw(1))*abs(tw_obj(2)-tw_obj(1)); % 矩形的面积
% 接下来求阴影部分的面积
% 如果小于0，说明有一个时间窗在另一个时间窗内,否则两个时间窗重叠一部分
if delta1*delta2 < 0
    if delta1 > 0 
        T12 = tw_obj(2);
        T11 = tw_obj(1);
        T22 = tw(2);
        T21 = tw(1);
        d1 = duration(i);
        d2 = duration(q);
    else
        T12 = tw(2);
        T11 = tw(1);
        T22 = tw_obj(2);
        T21 = tw_obj(1);
        d1 = duration(q); 
        d2 = duration(i);
    end
    if T21+d2 > T11
        S2 = 0.5*(T12-T21-d2)*(T12-d2-T21);
    else
        S2 = 0.5*(T11-2*d2-2*T21+T12)*(T12-T11);
    end
    if T22-d1 < T12
        S1 = 0.5*(T22-T11-d1)*(T22-d1-T11);
    else
        S1 = 0.5*(2*T22-T12-T11-2*d1)*(T12-T11);
    end
else
    if delta1 > 0
        T12 = tw(2);
        T11 = tw(1);
        T22 = tw_obj(2);
        T21 = tw_obj(1);
        d1 = duration(i);
        d2 = duration(q);
        
    else
        T12 = tw_obj(2);
        T11 = tw_obj(1);
        T22 = tw(2);
        T21 = tw(1);
        d1 = duration(q);
        d2 = duration(i);
    end
    S1 = 0.5*(T22-T11-d1)*(T22-d1-T11);
    if T11-d2 > T21
            S2 = 0.5*(T12-T11)*(T22-2*T21+T11-d2);
        else
            if d2+T22 < T12
                S2 = 0.5*(2*T12-2*d2-T22-T21)*(T22-T21);
            else
                S2 = 0.5*(T12-d2-T21)*(T12-d2-T21);
            end
    end
end
prob = (S0-S1-S2)/S0; % 求任务i与任务q之间的冲突发生概率
end
%% 子函数9，计算各个在各架无人机上的有效时间窗,只比较任务的开始时间和无人机从基地飞到任务所用的时间
function TW = TW_tasks2(SE_tasks,tasks_num,UAVs_num,base,V_UAV,t_pos)
TW = zeros(tasks_num,2);
for i = 1:tasks_num
    for j = 1:UAVs_num
        % 计算基地与任务位置点之间的距离
        task_position = t_pos(i,:);
        distance = distance_value(base,task_position);
        tw_start = distance/V_UAV*3600; % 转换成单位s
        TW(i,1) = max([tw_start,SE_tasks(i,1)]);
        TW(i,2) = SE_tasks(i,2);
        % 如果更新的时间窗的开始时间大于结束时间，那么令为0向量。
        if TW(i,1) > TW(i,2)
            TW(i,1) = 0;
            TW(i,2) = 0;
        end
    end
end
end
%% 子函数10，计算各个任务在各架无人机上的观测机会
function opp = UAVs_opp(tasks_num,UAVs_N,base,t_pos,TW_tasks,D_UAV)
opp = zeros(tasks_num,UAVs_N); % 无人机基地对任务的观测机会
for i = 1:tasks_num
    dist_base = distance_value(base,t_pos(i,:));
    % 最大巡航距离判断 和 时间窗长度判断
    if dist_base < D_UAV && TW_tasks(i,2)-TW_tasks(i,1) ~= 0  
        opp(i,:) = 1;
    else
        opp(i,:) = 0;
    end
end
end



