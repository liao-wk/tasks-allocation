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
%% 任务点的经纬度，t_pos
% 任务的权重（价值）
W = load("weight.txt");
W = W';
lats = load("X.txt");
lons = load("Y.txt");
size_t = size(lats); % 任务纬度的大小，后面用来表示任务的个数
t_pos = zeros(size_t(2),2);
t_pos(1:end, 1) = lats';
t_pos(1:end, 2) = lons';
% 任务的硬性时间窗
TW_tasks = zeros(size_t(2), 2);
st = load("Estime.txt");
TW_tasks(1:end,1) = st';
le = load("letime.txt");
TW_tasks(1:end,2) = le';
%% 根据可见时间窗；判断任务是否被卫星观测到，每16个数据表示1颗卫星
% 任务点的可见时间窗
Windows = load("window.txt");
size_W = size(Windows); % 时间窗口矩阵的大小
if size_t(2) ~= size_W(1)
    disp("卫星遗漏了部分任务的观测结果数据")
end
sats_opp = zeros(size_t(2),size_W(2)/16); % 任务i被卫星观测的机会，1表示可以被观测，否则为0
for i = 1:size_W(1)
    indexes = find(Windows(i,1:end) > 0);
    sats_opp(i,floor(indexes/16)+1) = 1;
end
sats_opp1 = sats_opp(1:end,1:SATS_N(1)); % 子规划中心1对任务的观测机会
sats_opp2 = sats_opp(1:end,SATS_N(1)+1:SATS_N(1)+SATS_N(2)); % 子规划中心2对任务的观测机会
%% 根据无人机的基地位置和航速，判断任务能否被无人机观测到
UAVs_opp1 = zeros(size_t(2),UAVs_N(1)); % 无人机基地1(子规划中心3)对任务的观测机会
UAVs_opp2 = zeros(size_t(2),UAVs_N(2)); % 无人机基地2(子规划中心4)对任务的观测机会
tasks_alloc_funs = task_alloc_funs;
for i = 1:size_t(2)
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
%% 卫星冲突度计算，时间窗上是否发生冲突



