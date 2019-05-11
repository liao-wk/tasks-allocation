function funs = task_alloc_funs1
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
funs.Windows_Satsopp = @Windows_Satsopp; % 获取任务在卫星下的时间窗和观测机会
funs.opp_devide_weight = @opp_devide_weight; % 获取任务在观测资源下的需求度
funs.TW_tasks_sats = @TW_tasks_sats; % 获取任务在卫星下的有效时间窗
funs.Conflict_value_sats = @Conflict_value_sats; % 获取任务在资源下的冲突度大小
funs.Conflict_value_sats_schedule = @Conflict_value_sats_schedule; % 获取任务在资源下的与已调度任务冲突度大小
funs.Window_sats = @Window_sats; % 计算任务在各轨道上的最早开始时间、最晚开始时间
end
%% 子函数1，获取任务在卫星下的时间窗长度和观测机会 %%%%%%%%%%%%%%%%%%%%%%%%
function [Windows,sats_opp] = Windows_Satsopp(sats,duration,SE_tasks)
Windows = sats.end - sats.start; % 预先分配内存
pathway_num = size(Windows,2); % 卫星的轨道数量
sats_num = pathway_num/16; % 卫星的个数
tasks_num = size(Windows,1); % 任务的个数
sats_opp = zeros(tasks_num,sats_num); % 任务i被卫星观测的机会，1表示可以被观测，否则为0
for i = 1:tasks_num  % 任务的个数
    for j = 1:pathway_num  % 全部卫星轨道de圈数
        % 如果卫星的观测时间是有效观测时间，跳过，否则令它为0
        if sats.end(i,j) >= duration(i)+SE_tasks(i,1) &&...
                sats.start(i,j) <= SE_tasks(i,2)-duration(i)
            Windows(i,j) = sats.end(i,j) - sats.start(i,j);
        else % 如果不满足有效观测的条件
            Windows(i,j) = 0;
        end
    end
    indexes = find(Windows(i,1:end) > 0);
    % 一个时间窗为一个观测机会
    opp_num = size(indexes,2);
    for k = 1:opp_num
        if sats_opp(i,floor(indexes(k)/16)+1) == 0
            sats_opp(i,floor(indexes(k)/16)+1) = 1;
        else
            sats_opp(i,floor(indexes(k)/16)+1) = sats_opp(i,floor(indexes(k)/16)+1)+1;
        end
    end
end
end
%% 子函数2，获取任务在观测资源下的需求度 %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function tasks_value = opp_devide_weight(resource_num,opp,W)
tasks_num = size(W,1);
tasks_value = zeros(tasks_num,resource_num);
for j = 1:resource_num
    for i = 1:tasks_num
        if opp(i,j) ~= 0
            tasks_value(i,j) = W(i)/opp(i,j); % 任务i的权重除以在资源j下的机会
        else
            tasks_value(i,j) = 0; % 如果任务在资源下没有观测机会，那么需求度为0.
        end
    end
end
end
%% 子函数3，取任务在卫星下的有效时间窗，以元组的形式存储 %%%%%%%%%%%%%%%%%%%%%%
function TW_tasks = TW_tasks_sats(sats,SE_tasks)
% 定义一个函数,根据两个时间窗确定有效时间窗
f_tw = @(tw1,tw2) [max([tw1(1),tw2(1)]),min([tw1(2),tw2(2)])]; 

sats_num = size(sats.Windows,2)/16; 
tasks_num = size(sats.Windows,1); 
m = 2; % 时间窗向量的列数
TW_tasks = cell(tasks_num,sats_num); % 给TW_tasks赋初值
k = max(max(sats.sats_opp))+1; % 卫星观测机会中最大数作为时间窗向量的维数，赋初值用处
% 一个任务在一颗卫星个观测下可能有多个时间窗，因此要提前赋一个向量空间
for i = 1:tasks_num
    for p = 1:sats_num  % 卫星的编号
        q = 1; % 任务的有效时间窗矩阵的索引
        % 对某一任务在某卫星上的时间窗tw赋一个向量空间
        tw = zeros(m,k);
        for j = 16*(p-1)+1:16*(p-1)+16
            % 如果在一颗卫星的观测下（16列），存在Windows中的元素大于0，
            %  根据i,j位置找到有效时间窗
            if sats.Windows(i,j) > 0
                % 先获得任务规定完成时间，再获得卫星观测下任务的时间窗口
                tw1 = SE_tasks(i,:);
                tw2 = [sats.start(i,j),sats.end(i,j)];
                tw(q, 1:end) = f_tw(tw1,tw2);
                q = q+1;
            end
        end
        % 在执行完一颗卫星的周期（16列）后，删除tw的零行。
        tw(q:end,:) = [];
        TW_tasks{i,p} = tw;
    end
end
end
%% 子函数4，获取任务在资源下的冲突度大小 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Conflict = Conflict_value_sats(sats,i,j,W)
tasks_num = size(sats.TW_tasks,1);
m = size(sats.TW_tasks{i,j},1); % 计算任务i在卫星j上有m个时间窗
Conflict = zeros(m,1);
for p = 1:m % p代表任务i在卫星j上的第p个时间窗
    tw1 = sats.TW_tasks{i,j}(p,:); % 表示任务i在卫星j上的第p个时间窗
    for q = 1:tasks_num
        % 如果其他任务不存在时间窗或其他任务为任务i
        if q == i || isempty(sats.TW_tasks{q,j})
            continue
        else
            tw_tasks = sats.TW_tasks{q,j}; % 时间窗可能有多个
            k = size(tw_tasks,1); % 得到任务q的时间窗的行数
            for x = 1:k
                tw2 = tw_tasks(x,:); % 比较的任务的第x个时间窗
                % 有效时间窗交叉判断
                if tw2(2)-tw2(1)+tw1(2)-tw1(1)-(max([tw2(2),tw1(2)])-...
                        min([tw2(1),tw1(1)])) > 0
                    Conflict(p,1) = Conflict(p,1) + W(q)/k; %  冲突度累加
                end
            end
        end  
    end
end
end
%% 子函数5，获取任务在观测资源下与已调度任务的冲突度大小 %%%%%%%%%%%%%%%%
function Conflict = Conflict_value_sats_schedule(sats,i,j,W,Schedule)
tasks_num = size(sats.TW_tasks,1);
m = size(sats.TW_tasks{i,j},1); % 计算任务i在卫星j上有m个时间窗
Conflict = zeros(m,1);
for p = 1:m % p代表任务i在卫星j上的第p个时间窗
    tw1 = sats.TW_tasks{i,j}(p,:); % 表示任务i在卫星j上的第p个时间窗
    for q = 1:tasks_num
        % 如果其他任务不存在时间窗或其他任务为任务i或在资源j下不是已调度的任务
        if q == i || isempty(sats.TW_tasks{q,j}) || Schedule(q,j) == 0
            continue
        else
            tw_tasks = sats.TW_tasks{q,j}; % 时间窗可能有多个
            k = size(tw_tasks,1); % 得到任务q的时间窗的行数
            for x = 1:k
                tw2 = tw_tasks(x,:); % 比较的任务的第x个时间窗
                % 有效时间窗交叉判断
                if tw2(2)-tw2(1)+tw1(2)-tw1(1)-(max([tw2(2),tw1(2)])-...
                        min([tw2(1),tw1(1)])) > 0
                    Conflict(p,1) = Conflict(p,1) + W(q)/k; %  冲突度累加
                end
            end
        end  
    end
end
end
%% 子函数6，计算任务在各轨道上的最早开始时间、最晚开始时间
function [Stime,Etime] = Window_sats(sats,SE_tasks,duration)
tasks_num = size(SE_tasks,1);
pathways_num = size(sats.start,2); % 轨道数量
Stime = zeros(tasks_num,pathways_num);
Etime = zeros(tasks_num,pathways_num);
for i = 1:tasks_num
    for j = 1:pathways_num
        if sats.end(i,j) >= duration(i)+SE_tasks(i,1) &&...
                sats.start(i,j) <= SE_tasks(i,2)-duration(i)
            Stime(i,j) = max([sats.start(i,j),SE_tasks(i,1)]);
            Etime(i,j) = min([sats.end(i,j),SE_tasks(i,2)]);
        end
    end
end

end


