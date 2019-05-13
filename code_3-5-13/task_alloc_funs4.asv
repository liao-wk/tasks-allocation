function  funs = task_alloc_funs4
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
funs.Neighborhood_structure_cross = @Neighborhood_structure_cross; % 邻域结构交换
funs.Neighborhood_structure_insertion = @Neighborhood_structure_insertion; % 邻域结构插入
funs.Neighborhood_structure_delete_Conflict = @Neighborhood_structure_delete_Conflict; % 邻域结构删除
funs.Neighborhood_structure_delete_Need = @Neighborhood_structure_delete_Need; % 邻域结构删除
end
%% 子函数1，邻域结构交换任务 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [solu,task1,task2] = Neighborhood_structure_cross(T_schedule,Tabu1,tasks_Opportunity_subplan,tasks)
% 任意两个子规划中心的已调度方案中的两个任务进行交换
subplan_num = size(T_schedule,1);
pp = randperm(subplan_num);
sub1 = pp(1); % 选择的子规划中心1
sub2 = pp(2); % 选择的子规划中心2
index1 = find(T_schedule(sub1,:)>0);
index2 = find(T_schedule(sub1,:)>0);
index1_num = size(index1,2);
index2_num = size(index2,2);
ppp1 = randperm(index1_num);
ppp2 = randperm(index2_num);
task1 = index1(ppp1(1)); % 子规划中心1的任务编号
task2 = index2(ppp2(1)); % 子规划中心2的任务编号
% 禁忌策略Tabu1
while Tabu1(task1,task2) ~= 0
    % task1在子规划中心2中有观测机会，则退出
    while ~tasks_Opportunity_subplan(sub2,task1)
        ppp1 = randperm(index1_num);
        task1 = index1(ppp1(1));
    end
    % task2 同理
    while ~tasks_Opportunity_subplan(sub1,task1)
        ppp2 = randperm(index2_num);
        task2 = index1(ppp2(1));
    end
end
% task1和task2进行交换
solu = T_schedule;
solu(sub1,task1) = 0;
solu(sub2,task1) = 1;
solu(sub1,task2) = 1;
solu(sub2,task2) = 0;
% 同时在两个子规划中心删去最小需求度的任务，删去的任务放在其它可执行的子规划中心
index_sub1 = find(solu(sub1,:)>0); % 子规划中心sub1中分配到的任务
index_sub2 = find(solu(sub2,:)>0); % 子规划中心sub2中分配到的任务
[C,index_task1] = min(tasks(index_sub1',9));
[D,index_task2] = min(tasks(index_sub2',9));
solu(sub1,index_task1) = 0;
solu(sub2,index_task2) = 0;
% 将删除的任务插入到其它可执行的子规划中心上
pp = randperm(subplan_num);
p1 = pp(1);
pp = randperm(subplan_num);
p2 = pp(1);
while ~tasks_Opportunity_subplan(sub1,index_task1)&&p1~=sub1
    pp = randperm(subplan_num);
    p1 = pp(1);
    solu(p1,index_task1) = 1;
end
while ~tasks_Opportunity_subplan(sub2,index_task2)&&p2~=sub2
    pp = randperm(subplan_num);
    p2= pp(1);
    solu(p2,index_task2) = 1;
end
%% 删除每个子规划中心平均冲突度最大的任务，选择其它子规划中心插入
%  计算每个子规划中心已分配任务的平均冲突度
W = tasks(:,2);
% index1是子规划中心sub1已调度的所有任务编号
Schedule_sub1 = index1; % 在子规划中心sub1已调度的任务集
index1_num = size(index1,2); % 在子规划中心sub1已调度的任务数量
% 判断sub1属于哪种观测资源
if sub1 == 1 || sub2 == 2
    % 计算各个任务在子规划中心的冲突度
    sub1_num = 0;
    clear sub_R;
    sub_R = []; % 子规划中心sub1所属的资源编号
    for k = 1:subplan_num
        if subplan(k,2) == sub1
            sub1_num = sub1_num+1; % 子规划中心sub1的观测资源数量
            sub_R = [sub_R,subplan(k,2)];% 子规划中心sub1所属的资源编号
        end
    end
    % 求Schedule,它是根据Schedule_sub1和tasks_Opportunity得到的
    Schedule = zeros(tasks_num,r_num);
    for i = 1:index1_num
        for j = 1:r_num
            if tasks_Oppotunity(Schedule_sub1(i),j) ~= 0 && subplan(j,2) == sub1
                Schedule(Schedule_sub1(i),j) = 1;
            end
        end
    end
    Conflict = zeros(index1_num,sub1_num); % 第2列是对应的冲突度
    Conflict(:,1) = index1; % 第1列为子规划中心sub1已调度的任务编号
    for i = 1:index1_num
        for j = 1:sub_num
            R_code = sub_R(j); % 资源的编号
            Conflict(i,2) = Conflict_value_sats_schedule(sats,Schedule_sub1(i),R_code,W,Schedule);
        end
    end
    
elseif sub1 == 3 ||sub1 == 4
 % 计算各个任务在子规划中心的冲突度
    sub1_num = 0;
    clear sub_R;
    sub_R = []; % 子规划中心sub1所属的资源编号
    for k = 1:subplan_num
        if subplan(k,2) == sub1
            sub1_num = sub1_num+1; % 子规划中心sub1的观测资源数量
            sub_R = [sub_R,subplan(k,2)];% 子规划中心sub1所属的资源编号
        end
    end
    % 求Schedule,它是根据Schedule_sub1和tasks_Opportunity得到的
    Schedule = zeros(tasks_num,r_num);
    for i = 1:index1_num
        for j = 1:r_num
            if tasks_Oppotunity(Schedule_sub1(i),j) ~= 0 && subplan(j,2) == sub1
                Schedule(Schedule_sub1(i),j) = 1;
            end
        end
    end
    Conflict = zeros(index1_num,sub1_num); % 第2列是对应的冲突度
    Conflict(:,1) = index1; % 第1列为子规划中心sub1已调度的任务编号
    for i = 1:index1_num
        for j = 1:sub_num
            R_code = sub_R(j); % 资源的编号
            Conflict(i,2) =Conflict_Caculate_schedule(TW_uavs,tasks,base,V_UAV,D_UAV,Schedule_sub1(i),R_code,Schedule);
        end
    end
    
end
end
%% 子函数2，邻域结构插入未完成任务 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function solu = Neighborhood_structure_insertion(solu,T_unschedule,R_subplan,...
    Conflict_tasks,subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV,Tabu2)
tasks_alloc_funs3 = task_alloc_funs3; % 函数脚本赋值

tasks_num = size(T_unschedule,2);
subplan_num = size(T_unschedule,1);
r_num = size(subplan,1);
duration = tasks(:,7);
%% 对未调度任务按照需求度进行从大到小的排序，再按照互不干扰度分配
UnSchedule = zeros(tasks_num,2);
q = 1;
for i = 1:subplan_num
    for j = 1:tasks_num
        if T_unschedule(i,j) == 1
            UnSchedule(q,1) = j;
            UnSchedule(q,2) = tasks(j,9);
            q = q+1;
        end
    end
end
UnSchedule(q:end,:) = []; % 删除多余
[A,index3] = sort(UnSchedule(:,2),'descend');
UnSchedule = UnSchedule(index3,:);% 按需求度从大到小排序得到未调度任务的UnSchedule
UnSchedule_tasks = UnSchedule(:,1);

%    在引入禁忌策略前提下，按最小冲突度原则将UnSchedule中的任务分配给各子规划中心
%    在分配的时候考虑各子规划中心资源的限制，资源的消耗以子规划中心为单位

% 按需求度从大到小将未调度任务集UnSchedule中的任务进行分配
UnSchedule_num = size(UnSchedule,1);
for i = 1:UnSchedule_num
    clear RC;
    clear subplan_list;
    k = UnSchedule_tasks(i,1); % 需求度最高的任务的编号
    R_use = find(Conflict_tasks.average(k,:) < inf); % 找出任务的可用资源,行向量
    C_Tab = tasks_alloc_funs3.Conflict_Caculate(subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV,solu,k); % 任务与已调度的任务在可用资源下的冲突度
    RC(1:2,:) = [R_use;C_Tab]; % 可选资源的编号和冲突度的合成
    RC_num = size(RC,2); % 任务k的具有观测机会的观测资源数量
    
    % 在RC的基础上添加一行，RC第3行为对应的子规划中心编号
    for q = 1:RC_num
        for qq = 1:r_num
            if RC(1,q) == subplan(qq,1)
                RC(3,q) = subplan(qq,2);
                break
            end
        end
    end
    
    %% 计算各子规划中心的互不干扰度，无剩余资源的子规划中心的互不干扰度为0
    subplan_list2 = unique(RC(3,:)); % 子规划中心编号无重复的升序列
    subplan_list_num = size(subplan_list2,2); % 可用的子规划中心数量
    subplan_list(1,:) = subplan_list2; % subplan_list第1行是子规划中心的编号
    subplan_list(2,:) = zeros(1,subplan_list_num); % subplan_list第2行是任务k在对应的子规划中心的互不干扰度
    % 计算各子规划中心的平均冲突度的具体步骤 $$ 互不干扰度 $$
    for m = 1:subplan_list_num % 逐个遍历任务k的具有观测机会的子规划中心
        % 同一子规划中心总的冲突度，先求冲突度再求互不干扰度
        for n = 1:RC_num
            if RC(3,n) == subplan_list(1,m)
                subplan_list(2,m) = subplan_list(2,m)+RC(2,n);
            end
        end
    end
    % 计算任务k在各个具有观测机会的子规划中心的互不干扰度
    subplan_list(2,:) = max(subplan_list(2,:))-subplan_list(2,:);
    for m = 1:subplan_list_num
        subplan_code = subplan_list(1,m); % 子规划中心编号
        if R_subplan(subplan_code,1)<1 && R_subplan(subplan_code,2)< duration(k)
            subplan_list(2,m) = 0; % 如果子规划中心无具有剩余资源的观测资源,互不干扰度为0
        end
    end
    
    
    %% 按互不干扰度进行从大到小排序，第1行是对应的子规划中心，第2行是对应的互不干扰度。
    [B,index2] = sort(subplan_list(2,:),'descend');
    subplan_list = subplan_list(:,index2); % 同上注释
    % 将任务k分配到第d个子规划中心，该子规划中心可用资源消耗剩余资源
    d = 1;
    while d <= subplan_list_num
        p_sub = subplan_list(1,d); % subplan_list的第1行是子规划中心编号
        % 引入禁忌策略Tabu2,满足资源消耗。
        if R_subplan(p_sub,1)>=1 && R_subplan(p_sub,2)>=duration(k) && Tabu2(p_sub,k)==0
            solu(p_sub,k) = 1; % 将未完成的任务插入到子规划中心上
            R_subplan(p_sub,1) = R_subplan(p_sub,1)-1; % 开关机次数
            R_subplan(p_sub,2) = R_subplan(p_sub,2)-duration(k); % 持续观测时间
            break
        else
            d = d+1;
        end
    end
    if d == subplan_list_num+1
        solu(1,k) = 1; % 将未完成的任务插入到子规划中心上
    end
end
end
%% 子函数3，邻域结构删除，即从任意一个子规划中心已调度的任务集中选择一个互不干扰度最小的任务%%%%%%%%%%%%%%%%%%
% 进行删除，然后添加到这个子规划中心的未调度任务中
function [T_schedule,T_unschedule,p_sub,t] = Neighborhood_structure_delete_Conflict(T_schedule,...
    T_unschedule,tasks,subplan,tasks_Opportunity,sats,uavs,base1,base2,V_UAV,D_UAV,Tabu1)
tasks_alloc_funs = task_alloc_funs1; % 函数脚本赋值
tasks_alloc_funs2 = task_alloc_funs2; % 函数脚本赋值
% 首先选择任意一个子规划中心p_sub
tasks_num = size(T_schedule,2);
r_num = size(subplan,1);
W = tasks(:,2);
subplan_num = max(subplan(:,2)); % 子规划中心的数量
p_sub = ceil(rand*subplan_num); % 任意选择的子规划中心的编号
%% 从已调度任务集中选择互不干扰度最小的任务，进行删除
% 先得到该子规划中心已调度的任务集（非01编码）
P_Schedule = find(T_schedule(p_sub,:)==1); % 行向量
P_Schedule_num = size(P_Schedule,2); % 已调度任务的数量
P_Conflict = zeros(P_Schedule_num,2); % 任务的冲突度，第1列是任务编号，第2列是冲突度
P_Conflict(:,1) = P_Schedule; % 第1列是任务编号
% 将子规划中心p_sub拆开成对应的观测资源，任务在观测资源下的调度矩阵
% 调度矩阵是tasks_num行，r_num列，已调度的任务且可用的观测资源属于p_sub，对应的元素为0
Schedule = zeros(tasks_num,r_num);
for i = 1:P_Schedule_num
    for j =1:r_num
        t = P_Schedule(i); % 任务t
        if tasks_Opportunity(t,j)>0 && subplan(j,2)==p_sub
            Schedule(t,j) = 1; % 任务在属于p_sub的可用观测资源下的调度矩阵
        end
    end
end

for i = 1:P_Schedule_num
    %% 计算任务t在子规划中心p_sub上的冲突度
    t = P_Schedule(i); % 任务t
    % 首先判断子规划中心p_sub属于哪种观测资源
    if p_sub == 1||p_sub == 2
        for j = 1:r_num
            % 判断观测资源是否属于p_sub且任务t在该资源下可用
            if subplan(j,2) == p_sub && tasks_Opportunity(t,j)>0
                P_Conflict(i,2)=P_Conflict(i,2)+tasks_alloc_funs.Conflict_value_sats_schedule(sats,t,j,W,Schedule);
            end
        end
        
        
    elseif p_sub == 3||p_sub == 4
        if p_sub == 3
            base = base1; % p_sub的基地
            TW_uavs = uavs.TW_uavs1; % p_sub的有效时间窗
        else
            base = base2;
            TW_uavs = uavs.TW_uavs2;
        end
        
        for j = 1:r_num
            % 判断观测资源是否属于p_sub且任务t在该资源下可用
            if subplan(j,2) == p_sub && tasks_Opportunity(t,j)>0
                P_Conflict(i,2)=P_Conflict(i,2)+tasks_alloc_funs2.Conflict_Caculate_schedule(TW_uavs,tasks,base,V_UAV,D_UAV,t,j,Schedule);
            end
        end
        
    end
end
%% p_sub已调度任务集的互不干扰度，等于任务中的最大冲突度减冲突度矩阵，互不干扰度越小任务执行情况越差
P_unConflict = P_Conflict; % p_sub已调度任务集的互不干扰度矩阵，第1列已调度任务编号，第2列互不干扰度
P_unConflict(:,2) = max(P_Conflict(:,2)) - P_Conflict(:,2);


% 对P_unConflict按照冲突度从小到大排序 
[A,index1] = sort(P_unConflict(:,2));
P_unConflict = P_unConflict(index1,:);
% 删除互不干扰度最小的任务，插入到T_unschedule，禁忌策略 
i = 1;
t = P_unConflict(i,1); % 任务t
while Tabu1(p_sub,t) ~= 0 % 如果被禁忌,重新生成
    i = i+1;
    if i == P_Schedule_num+1
        t = P_unConflict(1,1); % 任务t
        break
    end
    t = P_unConflict(i,1); % 任务t
end
T_unschedule(p_sub,t) = 1;
T_schedule(p_sub,t) = 0;

end
%% 子函数4，按照最小需求度删除子规划中心的任务
function [T_schedule,T_unschedule,p_sub,t2] = Neighborhood_structure_delete_Need(T_schedule,...
    T_unschedule,tasks,subplan,Tabu1)
% 首先选择任意一个子规划中心p_sub
subplan_num = max(subplan(:,2)); % 子规划中心的数量
p_sub = ceil(rand*subplan_num); % 任意选择的子规划中心的编号
%% 从已调度任务集中选择平均冲突度最大的任务，进行删除
% 先得到该子规划中心已调度的任务集（非01编码）
P_Schedule = find(T_schedule(p_sub,:)==1); % 行向量
P_Schedule_num = size(P_Schedule,2); % 已调度任务的数量
%% 从p_sub的已调度任务集P_Schedule中删除需求度最小的任务
Need = zeros(P_Schedule_num,2); % 第1列是已调度任务编号，第2列对应的需求度
Need(:,1) = P_Schedule';
Need(:,2) = tasks(P_Schedule',9);
% 按需求度对已调度的任务进行从小到大的排序
[A,index2] = sort(Need(:,2));
Need = Need(index2,:);
% 禁忌策略，并在T_schedule删除t2到T_unschedule中
i = 1;
t2 = Need(i,1); % 任务t2
while Tabu1(p_sub,t2) ~= 0 % 禁忌策略，直到任务t2不禁忌
    i = i+1;
    if i == P_Schedule_num+1
        t2 = Need(1,1); % 任务t2
        break
    end
    t2 = Need(i,1); % 任务t2
end
T_unschedule(p_sub,t2) = 1;
T_schedule(p_sub,t2) = 0;

end


