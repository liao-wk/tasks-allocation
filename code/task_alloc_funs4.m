function  funs = task_alloc_funs4
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
funs.Neighborhood_structure_cross = @Neighborhood_structure_cross; % 邻域结构交换
funs.Neighborhood_structure_insertion = @Neighborhood_structure_insertion; % 邻域结构插入
funs.Re_Tabu = @Re_Tabu; % 更新禁忌表
end
%% 子函数1，邻域结构交换任务 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [solu,task1,task2] = Neighborhood_structure_cross(T_schedule,Tabu1,tasks_Opportunity_subplan)
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
    while ~tasks_Opportunity_subplan(task1,sub2)
        ppp1 = randperm(index1_num);
        task1 = index1(ppp1(1));
    end
    % task2 同理
    while ~tasks_Opportunity_subplan(task2,sub1)
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
end
%% 子函数2，邻域结构插入未完成任务 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function solu = Neighborhood_structure_insertion(solu,T_unschedule,R_subplan,...
    Conflict_tasks,subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV,Tabu2)
tasks_alloc_funs3 = task_alloc_funs3; % 函数脚本赋值

tasks_num = size(T_unschedule,2);
subplan_num = size(T_unschedule,1);
r_num = size(subplan,1);
duration = tasks(:,7);
% 对未调度任务按照需求度进行排序，再按照冲突度分配
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
UnSchedule = UnSchedule(index3,:);
UnSchedule_tasks = UnSchedule(:,1);

%    在引入禁忌策略前提下，按最小冲突度原则将UnSchedule中的任务分配给各子规划中心
%    在分配的时候考虑各子规划中心资源的限制，资源的消耗以子规划中心为单位

% 按需求度排序得到未调度任务的UnSchedule
UnSchedule_num = size(UnSchedule,1);
for i = 1:UnSchedule_num
    clear RC;
    clear subplan_list;
    k = UnSchedule_tasks(i,1); % 需求度最高的任务的编号
    R_use = find(Conflict_tasks.average(k,:) < inf); % 找出任务的可用资源,行向量
    C_Tab = tasks_alloc_funs3.Conflict_Caculate(subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV,solu,k); % 任务与已调度的任务在可用资源下的冲突度
    RC(1:2,:) = [R_use;C_Tab]; % 可选资源的编号和冲突度的合成
    RC_num = size(RC,2);
    % 在RC的基础上添加一行，为对应的子规划中心编号
    for q = 1:RC_num
        for qq = 1:r_num
            if RC(1,q) == subplan(qq,1)
                RC(3,q) = subplan(qq,2);
                break
            end
        end
    end
    
    % 计算各子规划中心的平均冲突度，无剩余资源的子规划中心的冲突度为inf
    subplan_list2 = unique(RC(3,:)); % 子规划中心编号无重复的升序列
    subplan_list_num = size(subplan_list2,2); % 可用的子规划中心数量
    subplan_list(1,:) = subplan_list2;
    subplan_list(2,:) = zeros(1,subplan_list_num);
    % 计算各子规划中心的平均冲突度的具体步骤
    for m = 1:subplan_list_num
        num = 0;
        for n = 1:RC_num
            if RC(3,n) == subplan_list(1,m)
                subplan_list(2,m) = subplan_list(2,m)+RC(2,n);
                num = num+1;
            end
        end
        subplan_code = subplan_list(1,m); % 子规划中心编号
        if R_subplan(subplan_code,1)<1 && R_subplan(subplan_code,2)...
                < duration(k)
            subplan_list(2,m) = inf; % 如果子规划中心无具有剩余资源的观测资源
        else
            subplan_list(2,m) = subplan_list(2,m)/num; % 计算可用的子规划中心的平均冲突度
        end
    end
    
    % 按平均冲突度进行从小到大排序，第1行是对应的子规划中心，第2行是对应的平均冲突度。
    [B,index2] = sort(subplan_list(2,:));
    subplan_list = subplan_list(:,index2); % 同上注释
    % 将任务k分配到第d个子规划中心，该子规划中心可用资源消耗剩余资源
    d = 1;
    while d <= subplan_list_num
        p_sub = subplan_list(1,d); % subplan_list的第一行是子规划中心编号
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
%% 子函数3，更新禁忌表 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Tabu1,Tabu2] = Re_Tabu(Tabu1,Tabu2,TabuL,i)
tasks_num = size(Tabu1,1);
subplan_num = size(Tabu2,1);
for m = 1:tasks_num
    for n = 1:tasks_num
        task1 = SO_cross(i,1);
        task2 = SO_cross(i,2);
        if Tabu1(task1,task2) ~= 0
            Tabu1(task1,task2) = Tabu1(task1,task2)-1;
            Tabu1(task2,task1) = Tabu1(task2,task1)-1;
        else
            Tabu1(task1,task2) = TabuL;
            Tabu1(task2,task1) = TabuL;
        end
    end
end
for m = 1:subplan_num
    for n = 1:tasks_num
        pass
    end
end
end



