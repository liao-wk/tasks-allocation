function funs = task_alloc_funs3
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
funs.Opportunity_Sum = @Opportunity_Sum; % 计算各个任务在所有资源下的观测机会
funs.Conflict_average = @Conflict_average; % 计算平均冲突度
funs.Solution_generate = @Solution_generate; % 生成初始解
funs.Solution_generate_average = @Solution_generate_average; % 按照平均最小冲突度原则生成初始解
funs.Conflict_Caculate = @Conflict_Caculate; % 计算任务与已调度的任务的冲突度
end
%% 子函数1，计算各个任务在所有资源下的观测机会 %%%%%%%%%%%%%%%%%%%%%%%%%
function tasks_opp = Opportunity_Sum(tasks_Opportunity)
tasks_num = size(tasks_Opportunity,1);
tasks_opp = zeros(tasks_num,1); % 观测机会总和
for i = 1:tasks_num
    tasks_opp(i,1) = sum(tasks_Opportunity(i,:));
end
end
%% 子函数2，计算任务在不同的观测资源下的平均冲突度
function Conflict_ave = Conflict_average(resource_num,opp,Conflict)
tasks_num = size(opp,1);
Conflict_ave = zeros(tasks_num,resource_num);
% 计算机会矩阵中大于0的每一列的元素个数
for j = 1:resource_num
    A = opp(opp(:,j) > 0); % 机会矩阵中大于0的元素向量
    num = size(A,1)-1; % 该向量的行数减一
    if num == -1 % 即该卫星对所有的任务没有一个观测机会
        Conflict_ave(:,j) = inf;
    else
        Conflict_ave(:,j) = Conflict(:,j)/num; % 在资源r下的各个任务的平均冲突度
    end
end
end
%% 子函数3，生成初始解 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Solu = Solution_generate(tasks,Conflict_tasks,R_remain,subplan)
% 按需求度进行从大到小的排序
[Q,IX] = sort(tasks(:,9),'descend');
tasks2 = tasks(IX,:);
% 存在某些任务的总的观测机会为0,因此需要删除掉。
% 计算subplan的数目
% 根据subplan矩阵计算subplan数目
subplan_num = max(subplan(:,2));
r_num = size(subplan,1);
Solu = [];
tasks_num = size(tasks,1);
solu = zeros(subplan_num,tasks_num); % 解向量的矩阵形式
duration = tasks(:,7);
I = find(tasks(:,8)==0); % 行向量
nn = size(I,1);
if nn ~= 0
    disp("存在任务总的观测机会为0");
    disp(I);
end
for i = 1:tasks_num
    k = tasks2(i,1); % 任务的索引
    R_use = find(Conflict_tasks.average(k,:) < inf); % 找出任务的可用资源,行向量
    C_Tab = Conflict_tasks.average(k,R_use); % 任务在可用资源下的冲突度
    RC = [R_use;C_Tab]; % 可选资源的编号和冲突度的合成
    [A,index2] = sort(C_Tab);
    RC = RC(:,index2); % 按冲突度进行从小到大排序，第1行是对应的资源，第2行是对应的冲突度
    p = 1; % 在RC当中可选资源的索引（序号）
    Re = RC(p,1); % 初始选用的可选资源编号
    R_remain2 = R_remain;
    R_remain2(Re,1) = R_remain2(Re,1) - 1; % 剩余开关机次数减去1
    R_remain2(Re,2) = R_remain2(Re,1) - duration(k); % 剩余观测时间减去该任务的持续时间
    while 1
        if R_remain2(Re,1) >= 0 &&  R_remain2(Re,2) >= 0
            for j = 1:r_num
                if subplan(j,1) == Re
                    suplan_code = subplan(j,2);
                    break
                end
            end
            solu(suplan_code,k) = 1; % 分配任务到对应的资源上
            R_remain(Re,1) = R_remain(Re,1) - 1; % 剩余开关机次数减去1
            R_remain(Re,2) = R_remain(Re,1) - duration(k); % 剩余观测时间减去该任务的持续时间
            break
        else
            Re = Rc(p,1);
            p = p+1;
        end
    end   
end
% TODO将SOLU转换成行向量
% for i = 1:subplan_num
%     Solu = [Solu;solu(i,:)'];
% end
Solu = solu;
% TODO初始解的修复，因为无人机存在最大航程约束，所以要将可行解进行修复。
end
%% 子函数4，按照最小平均冲突度原则分配任务到各子规划中心 %%%%%%%%%%%%%%%%
function Solu = Solution_generate_average(tasks,Conflict_tasks,R_remain,subplan,sats,uavs,base1,base2,V_UAV,D_UAV)
% 按需求度进行从大到小的排序
[Q,IX] = sort(tasks(:,9),'descend');
tasks2 = tasks(IX,:);
% 存在某些任务的总的观测机会为0,因此需要删除掉。
% 计算subplan的数目
% 根据subplan矩阵计算subplan数目
subplan_num = max(subplan(:,2));
r_num = size(subplan,1);
tasks_num = size(tasks,1);
solu = zeros(subplan_num,tasks_num); % 解向量的矩阵形式
duration = tasks(:,7);
I = find(tasks(:,8)==0); % 行向量
nn = size(I,1);
if nn ~= 0
    disp("存在任务总的观测机会为0");
    disp(I);
end
% 每分配一个任务要计算一下下一个任务在各个资源下的冲突度，以此计算任务在各子规划中心
% 的平均最小冲突度，然后按照平均最小冲突度的原则分配任务
for i = 1:tasks_num
    clear RC;
    clear subplan_list;
    k = tasks2(i,1); % 任务的索引
    R_use = find(Conflict_tasks.average(k,:) < inf); % 找出任务的可用资源,行向量
    C_Tab1 = Conflict_tasks.average(k,R_use); % 任务在可用资源下的可能冲突度
    C_Tab2 = Conflict_Caculate(subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV,solu,k); % 任务与已调度的任务在可用资源下的冲突度
    C_Tab = 0*C_Tab1 + 1*C_Tab2; % 计算平均最小冲突度%%%%%############%%
    RC(1:2,:) = [R_use;C_Tab]; % 可选资源的编号和冲突度的合成
    RC_num = size(RC,2);
    % 在RCde基础上添加一行，为对应的子规划中心编号
    for p = 1:RC_num
        for q = 1:r_num
            if RC(1,p) == subplan(q,1)
                RC(3,p) = subplan(q,2);
                break
            end     
        end
    end
    
    % 计算各子规划中心的平均冲突度
    subplan_list2 = unique(RC(3,:)); % 子规划中心无重复的升序列
    subplan_list_num = size(subplan_list2,2); % 可用的子规划中心数量
    subplan_list(1,:) = subplan_list2;
    subplan_list(2,:) = zeros(1,subplan_list_num);
    % 计算各子规划中心的平均冲突度的具体步骤
    for m = 1:subplan_list_num
        num = 0;
        for n = 1:RC_num
            if RC(3,n) == subplan_list(1,m) &&R_remain(RC(1,n),1)>=1&&...
                    R_remain(RC(1,n),2)>=duration(k)
                subplan_list(2,m) = subplan_list(2,m)+RC(2,n);
                num = num + 1;
            end
        end
        if num ~= 0
            subplan_list(2,m) = subplan_list(2,m)/num;
        else
            subplan_list(2,m) = inf; % 如果子规划中心无具有剩余资源的观测资源
        end
    end
    
    % 按平均冲突度进行从小到大排序，第1行是对应的子规划中心，第2行是对应的平均冲突度。
    [A,index2] = sort(subplan_list(2,:));
    subplan_list = subplan_list(:,index2); % 同上注释
    
    % 将任务k分配到第p个子规划中心，该子规划中心的具有最小冲突度的可用资源消耗剩余资源
    p = 1;
    while p <= subplan_list_num
        p_sub = subplan_list(p,1);
        % 第p_sub个子规划中心的具有最小冲突度的可用资源
        c = inf; % 第p个子规划中心的可用资源的最小冲突度
        c_code = 0; % 第p个子规划中心的最小冲突度的可用资源的编号
        for j = 1:RC_num
            Re = RC(1,j);
            if RC(3,j) == p_sub && R_remain(Re,1)>=1 && R_remain(Re,2)>=duration(k)
                if RC(2,j) < c
                    c = RC(2,j);
                    c_code = j;
                end
            end
        end
        if c_code ~= 0
            RE = RC(1,c_code);
            R_remain(RE,1) = R_remain(RE,1) - 1;
            R_remain(RE,1) = R_remain(RE,1) - duration(k);
            solu(p_sub,k) = 1; % 分配任务到对应的子规划中心上
            break
        else
            p = p+1;
        end
    end
    
    if p == subplan_list_num + 1 % 如果各可选子规划中心的资源都已经消耗完毕
        p_sub = subplan_list(1,1);
        solu(p_sub,k) = inf; % 分配任务到冲突度最小的子规划中心上，并做标记
        disp("selected subplaners have been consumed!")
    end
    
end
Solu = solu;
end
%% 子函数6，计算任务与已调度的任务的冲突度 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Conflict = Conflict_Caculate(subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV,solu,k)
tasks_alloc_funs = task_alloc_funs1; % 函数句柄
tasks_alloc_funs2 = task_alloc_funs2; % 函数句柄
W = tasks(:,2);
r_num = size(subplan,1);
Conflict = zeros(r_num,1);
tasks_Opportunity = [sats.sats_opp,uavs.opp]; 
% 根据solu确定各个观测资源的调度情况
tasks_num = size(solu,2);
Schedule = zeros(tasks_num,r_num);
subplan_num = size(solu,1);
for i = 1:subplan_num
    index_tasks = find(solu(i,:)==1); % 每一个子规划中心已调度任务的编号
    index_resouce = find(subplan(:,2)==i); % 每一个子规划中心的观测资源的编号
    index_resouce_num = size(index_resouce,1);
    for j = 1:index_resouce_num
        Schedule(index_tasks,index_resouce(j)) = 1;
        % 对任务在该资源下没有观测机会的任务刷新为0
        Schedule(index_tasks,tasks_Opportunity(index_tasks,:)==0) = 0;
    end
end
% 根据各个观测资源的调度情况计算预调度任务k的冲突度
for j = 1:r_num
    if subplan(j,2) == 1 || subplan(j,2) == 2 % 如果观测资源是卫星
        if ~isempty(sats.TW_tasks{k,j}) % 如果任务i在卫星j上的时间窗为真
            Conflict(j) = tasks_alloc_funs.Conflict_value_sats_schedule(sats,k,j,W,Schedule);
        else
            Conflict(j) = inf;
        end
    elseif  subplan(j,2) == 3 % 如果观测资源属于无人机基地1
        if uavs.opp1(k,1) ~= 0
            Conflict(j) = tasks_alloc_funs2.Conflict_Caculate_schedule...
                (uavs.TW_uavs1,tasks,base1,V_UAV,D_UAV,k,j,Schedule);
        else
            Conflict(j) = inf;
        end
    else % 如果观测资源属于无人机基地2
        if uavs.opp2(k,1) ~= 0
            Conflict(j) = tasks_alloc_funs2.Conflict_Caculate_schedule...
                (uavs.TW_uavs2,tasks,base2,V_UAV,D_UAV,k,j,Schedule);
        else
            Conflict(j) = inf;
        end
    end
end
% 删除预调度任务k在不存在观测机会的观测资源下的冲突度
Conflict(Conflict(:,1)==inf) = [];
Conflict = Conflict';
end







