% ���ò���,������䵽���ӹ滮����
base1 = [27/180*pi, 95/180*pi];% 2�����˻��Ļ��ص�γ�Ⱥ;���,ת���ɻ��ȵ���ʽ
base2 = [29/180*pi, 105/180*pi];
% base3 = [26/180*pi, 105/180*pi];
V_UAV = 3000; % ���˻��ķ����ٶ�3000km/h
D_UAV = 2000; % ���˻��������о���2000km
Resources = struct('s1',140,'s2',140,'s3',140,'s4',140,'s5',140,'s6',140,...
's7',140,'s8',140,'u1',180,'u2',180,'u3',180,'u4',180,'u5',180,'u6',180,...
'u7',180); %һ��8�����ǣ�7�����˻�
SATS_N = [5, 3]; % 5+3������
UAVs_N = [4,3]; % 4+3�����˻�
% uavs_num = UAVs_N(1)+UAVs_N(2); % ���˻�������
tasks_alloc_funs = task_alloc_funs; % �����ű���ֵ
%% �����ľ�γ�ȣ�t_pos
% �����Ȩ�أ���ֵ��
W = load("weight.txt");
W = W';
lats = load("X.txt");
lons = load("Y.txt");
size_t = size(lats); % ����γ�ȵĴ�С������������ʾ����ĸ���
tasks_num = size_t(2); % ���������
t_pos = zeros(tasks_num,2);
t_pos(1:end, 1) = lats';
t_pos(1:end, 2) = lons';
% �����Ӳ��ʱ�䴰��SE_tasks(1)������������翪ʼʱ�䣬SE_tasks(2)����������������ʱ�䡣
SE_tasks = zeros(tasks_num, 2);
st = load("Estime.txt");
SE_tasks(1:end,1) = st';
le = load("letime.txt");
SE_tasks(1:end,2) = le';
% ����ĳ����۲���Ҫʱ��
duration = load("duration.txt");
duration = duration';
%% �õ���������Ӧ�����µĹ۲���ᡣ
% �����ܹ۲쵽��������Ŀ�ʼʱ��ͽ���ʱ��,�ܹ۲⵽������һ����Ч��
% ���ݿɼ�ʱ�䴰���ж������Ƿ����ǹ۲⵽��ÿ16�����ݱ�ʾ1������
% �����Ҫɾ������Ч�Ĺ۲���ᣬSE_sats_start��ʾ���ǹ۲⵽����Ŀ�ʼʱ��Stime
%  SE_sats_end��ʾ���ǹ۲⵽����Ľ���ʱ��Etime
SE_sats.start = load("Stime.txt");
SE_sats.end = load("Etime.txt");
size_start = size(SE_sats.start);
sats_num = size_start(2)/16; %���ǵ�����
% �õ���ʼ�Ĵ��ھ���
[SE_sats.Windows,sats_opp] = tasks_alloc_funs.Windows_Sats_opp(SE_sats,duration,SE_tasks);
sats_opp1 = sats_opp(1:end,1:SATS_N(1)); % �ӹ滮����1������Ĺ۲����
sats_opp2 = sats_opp(1:end,SATS_N(1)+1:SATS_N(1)+SATS_N(2)); % �ӹ滮����2������Ĺ۲����
%% ��������������Чʱ�䴰����Щ������ĳ�������������ɸ�ʱ�䴰
SE_sats.TW_tasks = tasks_alloc_funs.TW_tasks(sats_opp,SE_sats,SE_tasks);
%% �������˻��Ļ���λ�úͺ��٣��ж������ܷ����˻��۲⵽
UAVs_opp1 = zeros(tasks_num,UAVs_N(1)); % ���˻�����1(�ӹ滮����3)������Ĺ۲����
UAVs_opp2 = zeros(tasks_num,UAVs_N(2)); % ���˻�����2(�ӹ滮����4)������Ĺ۲����
for i = 1:tasks_num
    dist_b1 = tasks_alloc_funs.distance_value(base1,t_pos(i,1:2));
    dist_b2 = tasks_alloc_funs.distance_value(base2,t_pos(i,1:2));
    if dist_b1 < D_UAV  % ���Ѳ�������ж�
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
%% ��ͻ�ȼ��㣬���ǳ�ͻ�ȼ�������˻���ͻ�ȼ���
%% ���ǳ�ͻ��Conflict���㣬ʱ�䴰���Ƿ�����ͻ,��������˳�ͻ����ͻ�ȶ��
Conflict_tasks.sats = zeros(tasks_num,sats_num);
for i = 1:tasks_num
    for j = 1:sats_num
        if ~isempty(SE_sats.TW_tasks{i,j}) % �������i������j�ϵ�ʱ�䴰Ϊ��
            Conflict_tasks.sats(i,j) = tasks_alloc_funs.Conflict_degree1(i,j,SE_sats,W,tasks_num); 
        else
            Conflict_tasks.sats(i,j) = 0;
        end
    end
end
for j = 1:sats_num
    max_conflict = max(Conflict_tasks.sats(:,j)); % ÿ�����ǵ�����ͻ��
    if max_conflict ~= 0
    Conflict_tasks.sats(:,j) = Conflict_tasks.sats(:,j)/max_conflict;
    end
end
%% ���ǳ�ͻ��Conflict����de�ڶ��ַ�������������ʦ�����֣��ж���Чʱ�䴰�Ƿ��н���
Conflict_tasks.sats2 = zeros(tasks_num,sats_num);
for i = 1:tasks_num
    for j = 1:sats_num
        if ~isempty(SE_sats.TW_tasks{i,j}) % �������i������j�ϵ�ʱ�䴰Ϊ��
            Conflict_tasks.sats2(i,j) = tasks_alloc_funs.Conflict_degree1_2(i,j,SE_sats,W,tasks_num); 
        else
            Conflict_tasks.sats2(i,j) = 0;
        end
    end
end
for j = 1:sats_num
    max_conflict = max(Conflict_tasks.sats2(:,j)); % ÿ�����ǵ�����ͻ��
    if max_conflict ~= 0
    Conflict_tasks.sats2(:,j) = Conflict_tasks.sats2(:,j)/max_conflict;
    end
end
%% ���˻���ͻ��Conflict���㣬�����ֳ�ͻ��һ�����������ͻ����һ����ʱ�䴰��ͻ
Conflict_tasks.uav1 = tasks_alloc_funs.Conflict_degree2(tasks_num,UAVs_N(1),...
    UAVs_opp1,D_UAV,base1,t_pos,V_UAV,SE_tasks,duration,W);
Conflict_tasks.uav2 = tasks_alloc_funs.Conflict_degree2(tasks_num,UAVs_N(2),...
    UAVs_opp2,D_UAV,base2,t_pos,V_UAV,SE_tasks,duration,W);
%% ��Դ���Ķȵļ���
%% ���������ǹ۲��µ���Դ���Ķ�











