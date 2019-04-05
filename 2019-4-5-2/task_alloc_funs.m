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
funs.Conflict_degree1_2 = @Conflict_degree1_2; % 计算卫星观测下任务的冲突度大小
end
%% 子函数1，根据经纬度求任务点到基地的距离
function distance = distance_value(base1,t_pos)
r = 6371;
distance = 2*r*asin(sqrt(sin(0.5*(base1(1)-t_pos(1))).^2+cos(base1(1))*...
    cos(t_pos(1))*sin(0.5*(base1(2)-t_pos(2))).^2));
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
%% 子函数5，计算卫星观测下冲突度的大小
function Conflict = Conflict_degree1(i,j,SE_sats,W,tasks_num)
m = size(SE_sats.TW_tasks{i,j},1); % 计算任务i在卫星j上有m个时间窗
Conflict = zeros(m,1);
for p = 1:m % p代表任务i在卫星j上的第p个时间窗
    tw_obj = SE_sats.TW_tasks{i,j}(p,:); % 表示任务i在卫星j上的第p个时间窗
    TWValue_length = W(i)*(tw_obj(2)-tw_obj(1)); % 计算时间窗的长度乘以对应的价值，作为冲突度的分母
    for q = 1:tasks_num
        cro_deg = 0; % 初始化任务i与任务q的交叉度为0
        % 如果其他任务不存在时间窗或其他任务为任务i
        if q == i || isempty(SE_sats.TW_tasks{q,j})
            continue
        else
            tw_tasks = SE_sats.TW_tasks{q,j}; % 时间窗可能有多个
            k = size(tw_tasks,1); % 得到任务q的时间窗的行数
            for x = 1:k
                tw = tw_tasks(x,:); % 比较的任务的第x个时间窗
                % 计算时间窗的长度乘以对应的价值
                TWValue_length = TWValue_length+W(q)*(tw(2)-tw(1))/k;
                % 交叉度cro_deg计算，如果有交叉，求任务i第p个时间窗与任务q的交叉度大小
                if tw(2)-tw(1)+tw_obj(2)-tw_obj(1)-(max([tw(2),tw_obj(2)])-...
                        min([tw(1),tw_obj(1)])) > 0
                    cro_deg = cro_deg+tw(2)-tw(1)+tw_obj(2)-tw_obj(1)-...
                        (max([tw(2),tw_obj(2)])-min([tw(1),tw_obj(1)])) ;
                end
            end
            % 对任务q的交叉度取平均值，计算完交叉度后，计算任务i在卫星j上的第p个时间窗的冲突度的分子项
            Conflict(p,1) = Conflict(p,1)+cro_deg*W(q)/k;
        end
    end
    Conflict(p,1) = Conflict(p,1)/TWValue_length; % 计算冲突度
end
% 取任务i在卫星j上的最小冲突度
Conflict = min(Conflict);
end
%% 子函数6，计算无人机观测下任务的冲突度大小
function Conflict = Conflict_degree2(tasks_num,UAVs_N,UAVs_opp,D_UAV,base,t_pos,V_UAV,SE_tasks,duration,W)
Conflict = zeros(tasks_num,UAVs_N(1));
% 最大距离冲突判断,如果不满足，进行最大时间窗判断
for i = 1:tasks_num
    for j = 1:UAVs_N
        if UAVs_opp(i,j) ~= 0
        % 任务i在无人机j的原冲突度
        Conflict(i,j) = Judge_Dis_TW(i,j,tasks_num,UAVs_opp,base,D_UAV,t_pos,V_UAV,SE_tasks,duration,W); 
        end
    end
end
for j = 1:UAVs_N
    max_Conflict = max(Conflict(:,j)); % 在无人机j观测下的最大原冲突度
    if max_Conflict ~= 0
        Conflict(:,j) = Conflict(:,j)/max_Conflict; % 无人机观测下的冲突度
    end
end
end
%% 子函数7，最大距离和时间窗判断
function conflict = Judge_Dis_TW(i,j,tasks_num,UAVs_opp,base,D_UAV,t_pos,V_UAV,SE_tasks,duration,W)
conflict = 0;
distance3 = distance_value(base,t_pos(i,:)); % 任务i与基地之间的距离
for p = 1:tasks_num
    if p == i || UAVs_opp(p,j) == 0
        continue
    else
        distance1 = distance_value(t_pos(p,:),t_pos(i,:)); % 任务p与任务i之间的距离
        time1 = distance1/V_UAV*3600; % 任务p与任务i之间的飞行时间
        distance2 = distance_value(base,t_pos(p,:)); % 任务p与基地之间的距离
        distance = distance1+distance2+distance3;
        if distance > D_UAV
            conflict = conflict + W(p);
        elseif time1+SE_tasks(p,2)+duration(i) > SE_tasks(i,1)
            conflict = conflict + W(p);
        elseif time1+SE_tasks(1,1)+duration(i) > SE_tasks(p,2)
            conflict = conflict + W(p);
        end
    end
end
end
%% 子函数8，计算卫星观测下冲突度的大小
function Conflict = Conflict_degree1_2(i,j,SE_sats,W,tasks_num)
m = size(SE_sats.TW_tasks{i,j},1); % 计算任务i在卫星j上有m个时间窗
Conflict = zeros(m,1);
for p = 1:m % p代表任务i在卫星j上的第p个时间窗
    tw_obj = SE_sats.TW_tasks{i,j}(p,:); % 表示任务i在卫星j上的第p个时间窗
    for q = 1:tasks_num
        % 如果其他任务不存在时间窗或其他任务为任务i
        if q == i || isempty(SE_sats.TW_tasks{q,j})
            continue
        else
            tw_tasks = SE_sats.TW_tasks{q,j}; % 时间窗可能有多个
            k = size(tw_tasks,1); % 得到任务q的时间窗的行数
            for x = 1:k
                tw = tw_tasks(x,:); % 比较的任务的第x个时间窗
                % 计算时间窗的长度乘以对应的价值
                % 交叉度cro_deg计算，如果有交叉，求任务i第p个时间窗与任务q的交叉度大小
                if tw(2)-tw(1)+tw_obj(2)-tw_obj(1)-(max([tw(2),tw_obj(2)])-...
                        min([tw(1),tw_obj(1)])) > 0
                    Conflict(p,1) = Conflict(p,1) + W(q)/k; % ||有交叉就加上交叉的权值||
                end
            end
        end
    end
end
% 取任务i在卫星j上的最小冲突度
Conflict = min(Conflict);
end




