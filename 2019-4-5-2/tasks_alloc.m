% 设置参数,任务分配到各子规划中心
base1 = [27/180*pi, 95/180*pi];% 2个无人机的基地的纬度和经度,转换成弧度的形式
base2 = [29/180*pi, 105/180*pi];
% base3 = [26/180*pi, 105/180*pi];
V_UAV = 3000; % 无人机的飞行速度3000km/h
D_UAV = 2000; % 无人机的最大飞行距离2000km
Resources = struct('s1',140,'s2',140,'s3',140,'s4',140,'s5',140,'s6',140,...
's7',140,'s8',140,'u1',180,'u2',180,'u3',180,'u4',180,'u5',180,'u6',180,...
'u7',180); %一共8颗卫星，7架无人机
SATS_N = [5, 3]; % 5+3颗卫星
UAVs_N = [4,3]; % 4+3架无人机
% uavs_num = UAVs_N(1)+UAVs_N(2); % 无人机的数量
tasks_alloc_funs = task_alloc_funs; % 函数脚本赋值
%% 任务点的经纬度，t_pos
% 任务的权重（价值）
W = load("weight.txt");
W = W';
lats = load("X.txt");
lons = load("Y.txt");
size_t = size(lats); % 任务纬度的大小，后面用来表示任务的个数
tasks_num = size_t(2); % 任务的数量
t_pos = zeros(tasks_num,2);
t_pos(1:end, 1) = lats';
t_pos(1:end, 2) = lons';
% 任务的硬性时间窗，SE_tasks(1)代表任务的最早开始时间，SE_tasks(2)代表任务的最晚结束时间。
SE_tasks = zeros(tasks_num, 2);
st = load("Estime.txt");
SE_tasks(1:end,1) = st';
le = load("letime.txt");
SE_tasks(1:end,2) = le';
% 任务的持续观察需要时间
duration = load("duration.txt");
duration = duration';
%% 得到任务在相应卫星下的观测机会。
% 卫星能观察到各个任务的开始时间和结束时间,能观测到，但不一定有效。
% 根据可见时间窗；判断任务是否被卫星观测到，每16个数据表示1颗卫星
% 因此需要删除掉无效的观测机会，SE_sats_start表示卫星观测到任务的开始时间Stime
%  SE_sats_end表示卫星观测到任务的结束时间Etime
SE_sats.start = load("Stime.txt");
SE_sats.end = load("Etime.txt");
size_start = size(SE_sats.start);
sats_num = size_start(2)/16; %卫星的数量
% 得到初始的窗口矩阵
[SE_sats.Windows,sats_opp] = tasks_alloc_funs.Windows_Sats_opp(SE_sats,duration,SE_tasks);
sats_opp1 = sats_opp(1:end,1:SATS_N(1)); % 子规划中心1对任务的观测机会
sats_opp2 = sats_opp(1:end,SATS_N(1)+1:SATS_N(1)+SATS_N(2)); % 子规划中心2对任务的观测机会
%% 计算各个任务的有效时间窗，有些任务在某颗卫星下有若干个时间窗
SE_sats.TW_tasks = tasks_alloc_funs.TW_tasks(sats_opp,SE_sats,SE_tasks);
%% 根据无人机的基地位置和航速，判断任务能否被无人机观测到
UAVs_opp1 = zeros(tasks_num,UAVs_N(1)); % 无人机基地1(子规划中心3)对任务的观测机会
UAVs_opp2 = zeros(tasks_num,UAVs_N(2)); % 无人机基地2(子规划中心4)对任务的观测机会
for i = 1:tasks_num
    dist_b1 = tasks_alloc_funs.distance_value(base1,t_pos(i,1:2));
    dist_b2 = tasks_alloc_funs.distance_value(base2,t_pos(i,1:2));
    if dist_b1 < D_UAV  % 最大巡航距离判断
        UAVs_opp1(i,1:UAVs_N(1)) = 1;
    else
        UAVs_opp1(i,1:UAVs_N(1)) = 0;
    end
    if dist_b2 < D_UAV
        UAVs_opp2(i,1:UAVs_N(2)) = 1;
    else
        UAVs_opp2(i,1:UAVs_N(2)) = 0;
    end
end
%% 冲突度计算，卫星冲突度计算和无人机冲突度计算
%% 卫星冲突度Conflict计算，时间窗上是否发生冲突,如果发生了冲突，冲突度多大
Conflict_tasks.sats = zeros(tasks_num,sats_num);
for i = 1:tasks_num
    for j = 1:sats_num
        if ~isempty(SE_sats.TW_tasks{i,j}) % 如果任务i在卫星j上的时间窗为真
            Conflict_tasks.sats(i,j) = tasks_alloc_funs.Conflict_degree1(i,j,SE_sats,W,tasks_num); 
        else
            Conflict_tasks.sats(i,j) = 0;
        end
    end
end
for j = 1:sats_num
    max_conflict = max(Conflict_tasks.sats(:,j)); % 每颗卫星的最大冲突度
    if max_conflict ~= 0
    Conflict_tasks.sats(:,j) = Conflict_tasks.sats(:,j)/max_conflict;
    end
end
%% 卫星冲突度Conflict计算de第二种方法，近似于老师的那种，判断有效时间窗是否有交叉
Conflict_tasks.sats2 = zeros(tasks_num,sats_num);
for i = 1:tasks_num
    for j = 1:sats_num
        if ~isempty(SE_sats.TW_tasks{i,j}) % 如果任务i在卫星j上的时间窗为真
            Conflict_tasks.sats2(i,j) = tasks_alloc_funs.Conflict_degree1_2(i,j,SE_sats,W,tasks_num); 
        else
            Conflict_tasks.sats2(i,j) = 0;
        end
    end
end
for j = 1:sats_num
    max_conflict = max(Conflict_tasks.sats2(:,j)); % 每颗卫星的最大冲突度
    if max_conflict ~= 0
    Conflict_tasks.sats2(:,j) = Conflict_tasks.sats2(:,j)/max_conflict;
    end
end
%% 无人机冲突度Conflict计算，有两种冲突，一种是最大距离冲突，另一种是时间窗冲突
Conflict_tasks.uav1 = tasks_alloc_funs.Conflict_degree2(tasks_num,UAVs_N(1),...
    UAVs_opp1,D_UAV,base1,t_pos,V_UAV,SE_tasks,duration,W);
Conflict_tasks.uav2 = tasks_alloc_funs.Conflict_degree2(tasks_num,UAVs_N(2),...
    UAVs_opp2,D_UAV,base2,t_pos,V_UAV,SE_tasks,duration,W);
%% 资源消耗度的计算
%% 任务在卫星观测下的资源消耗度











