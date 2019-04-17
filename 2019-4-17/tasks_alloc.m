% 设置参数,任务分配到各子规划中心
base1 = [27/180*pi, 95/180*pi];% 2个无人机的基地的纬度和经度,转换成弧度的形式
base2 = [29/180*pi, 105/180*pi];
% base3 = [26/180*pi, 105/180*pi];
V_UAV = 1440; % 无人机的飞行速度400m/s,1440km/h。
D_UAV = 2000; % 无人机的最大飞行距离2000km
Resources = struct('s1',140,'s2',140,'s3',140,'s4',140,'s5',140,'s6',140,...
's7',140,'s8',140,'u1',180,'u2',180,'u3',180,'u4',180,'u5',180,'u6',180,...
'u7',180); %一共8颗卫星，7架无人机
SATS_N = [5, 3]; % 5+3颗卫星
UAVs_N = [4,3]; % 4+3架无人机
% uavs_num = UAVs_N(1)+UAVs_N(2); % 无人机的数量
tasks_alloc_funs = task_alloc_funs; % 函数脚本赋值
tasks_alloc_funs2 = task_alloc_funs2; % 函数脚本赋值
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
%  还需要考虑通过上面的计算得到的有效时间窗的长度是否大于观测该任务的持续时间
SE_sats.start = load("Stime.txt");
SE_sats.end = load("Etime.txt");
size_start = size(SE_sats.start);
sats_num = size_start(2)/16; %卫星的数量
% 得到初始的窗口矩阵和任务在卫星下的观测机会
[SE_sats.Windows,SATs.sats_opp] = tasks_alloc_funs.Windows_Sats_opp(SE_sats,duration,SE_tasks);
% 得到任务在卫星下的需求度，价值越高，观测机会越小的任务对资源的需求度越高
SATs.sats_oppweight = tasks_alloc_funs2.Opp_Weight(tasks_num,sats_num,...
    SATs.sats_opp,W);
SATs.sats_opp1 = SATs.sats_opp(1:end,1:SATS_N(1)); % 子规划中心1对任务的观测机会
SATs.sats_opp2 = SATs.sats_opp(1:end,SATS_N(1)+1:SATS_N(1)+SATS_N(2)); % 子规划中心2对任务的观测机会
%% 计算各个任务的有效时间窗，有些任务在某颗卫星下有若干个时间窗，以元组的形式
SE_sats.TW_tasks = tasks_alloc_funs.TW_tasks(SATs.sats_opp,SE_sats,SE_tasks);
%% 计算各个任务的有效时间窗，任务在某架无人机下只有1个时间窗，以矩阵的形式
% 不同的基地，任务对应的有效时间窗不一样
UAVs.TW_tasks_1 = tasks_alloc_funs.TW_tasks2(SE_tasks,tasks_num,UAVs_N(1),base1,V_UAV,t_pos);
UAVs.TW_tasks_2 = tasks_alloc_funs.TW_tasks2(SE_tasks,tasks_num,UAVs_N(2),base2,V_UAV,t_pos);
%% 计算任务在无人机的观测机会，根据无人机的基地位置和航速（最大距离判断以及时间窗长度判断）
UAVs.opp1 = tasks_alloc_funs.UAVs_opp(tasks_num,UAVs_N(1),base1,t_pos,UAVs.TW_tasks_1,D_UAV);
UAVs.opp2 = tasks_alloc_funs.UAVs_opp(tasks_num,UAVs_N(2),base2,t_pos,UAVs.TW_tasks_2,D_UAV);
% 得到任务在无人机下的需求度，价值越高，观测机会越小的任务对资源的需求度越高
UAVs.opp1weight = tasks_alloc_funs2.Opp_Weight(tasks_num,UAVs_N(1),...
    UAVs.opp1,W);
UAVs.opp2weight = tasks_alloc_funs2.Opp_Weight(tasks_num,UAVs_N(2),...
    UAVs.opp2,W);
%% 冲突度计算，卫星冲突度计算和无人机冲突度计算
%% 卫星冲突度Conflict计算，时间窗上是否发生冲突,由于卫星的可见时间窗口很小，卫星观测
%   地面点目标需要持续时间，因此认为两个任务的有效时间窗交叉了即必定发生冲突。
%   如果两个时间窗没有发生交叉，计算发生冲突的概率。
Conflict_tasks.sats = zeros(tasks_num,sats_num);
% 在meige卫星观测下，每个任务的冲突任务
SATs.conflict_withtasks = cell(tasks_num,sats_num); 
for i = 1:tasks_num
    for j = 1:sats_num
        if ~isempty(SE_sats.TW_tasks{i,j}) % 如果任务i在卫星j上的时间窗为真
            [Conflict_tasks.sats(i,j),SATs.conflict_withtasks{i,j}] = ...
       tasks_alloc_funs.Conflict_degree1(i,j,SE_sats,W,tasks_num,duration); 
        else
            Conflict_tasks.sats(i,j) = 0;
        end
    end
end
% 计算卫星的平均冲突度
Conflict_tasks.sats_ave = tasks_alloc_funs2.Conflict_average(tasks_num,...
    sats_num,SATs.sats_opp,Conflict_tasks.sats);
% 将观测机会为0的位置上的元素替换成inf
Conflict_tasks.sats(SATs.sats_opp == 0) = inf; 
Conflict_tasks.sats_ave(SATs.sats_opp == 0) = inf; 
%% 无人机冲突度Conflict计算，有两种冲突，一种是最大距离冲突，另一种是时间窗冲突
% 计算基地1的无人机的冲突度和各个任务与其他任务的冲突程度。
[Conflict_tasks.uav1,UAVs.conflict_withtasks1] = tasks_alloc_funs.Conflict_degree2(tasks_num,UAVs_N(1),...
    UAVs.opp1,D_UAV,base1,t_pos,V_UAV,UAVs.TW_tasks_1,duration,W);
% 计算基地1的无人机的平均冲突度
Conflict_tasks.uav1_ave = tasks_alloc_funs2.Conflict_average(tasks_num,...
   UAVs_N(1),UAVs.opp1,Conflict_tasks.uav1);
% 计算基地2的无人机的冲突度和各个任务与其他任务的冲突程度。
[Conflict_tasks.uav2,UAVs.conflict_withtasks2] = tasks_alloc_funs.Conflict_degree2(tasks_num,UAVs_N(2),...
    UAVs.opp2,D_UAV,base2,t_pos,V_UAV,UAVs.TW_tasks_2,duration,W);
% 计算基地2的无人机的平均冲突度
Conflict_tasks.uav2_ave = tasks_alloc_funs2.Conflict_average(tasks_num,...
   UAVs_N(2),UAVs.opp2,Conflict_tasks.uav2);
% 将观测机会为0的位置上的元素替换成inf
Conflict_tasks.uav1(UAVs.opp1 == 0) = inf; 
Conflict_tasks.uav1_ave(UAVs.opp1 == 0) = inf; 
Conflict_tasks.uav2(UAVs.opp2 == 0) = inf; 
Conflict_tasks.uav2_ave(UAVs.opp2 == 0) = inf; 
%% 资源消耗度的计算
%% 任务在卫星观测下的资源消耗度
Consumers.sats = tasks_alloc_funs.Consumer_Resource(tasks_num,sats_num,...
    SATs.sats_opp,0,base1,t_pos,duration,3000,1,140,D_UAV,0.5,0.5,0);
Consumers.uavs1 = tasks_alloc_funs.Consumer_Resource(tasks_num,UAVs_N(1),...
    UAVs.opp1,1,base1,t_pos,duration,3000,1,140,D_UAV,1/3,1/3,1/3);
Consumers.uavs2 = tasks_alloc_funs.Consumer_Resource(tasks_num,UAVs_N(2),...
    UAVs.opp2,1,base2,t_pos,duration,3000,1,140,D_UAV,1/3,1/3,1/3);








