% ���ò���,������䵽���ӹ滮����
base1 = [27/180*pi, 95/180*pi];% 2�����˻��Ļ��ص�γ�Ⱥ;���,ת���ɻ��ȵ���ʽ
base2 = [29/180*pi, 105/180*pi];
% base3 = [26/180*pi, 105/180*pi];
V_UAV = 1440; % ���˻��ķ����ٶ�400m/s,1440km/h��
D_UAV = 2000; % ���˻��������о���2000km
Resources = struct('s1',140,'s2',140,'s3',140,'s4',140,'s5',140,'s6',140,...
's7',140,'s8',140,'u1',180,'u2',180,'u3',180,'u4',180,'u5',180,'u6',180,...
'u7',180); %һ��8�����ǣ�7�����˻�
SATS_N = [5, 3]; % 5+3������
UAVs_N = [4,3]; % 4+3�����˻�
% uavs_num = UAVs_N(1)+UAVs_N(2); % ���˻�������
tasks_alloc_funs = task_alloc_funs; % �����ű���ֵ
tasks_alloc_funs2 = task_alloc_funs2; % �����ű���ֵ
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
%  ����Ҫ����ͨ������ļ���õ�����Чʱ�䴰�ĳ����Ƿ���ڹ۲������ĳ���ʱ��
SE_sats.start = load("Stime.txt");
SE_sats.end = load("Etime.txt");
size_start = size(SE_sats.start);
sats_num = size_start(2)/16; %���ǵ�����
% �õ���ʼ�Ĵ��ھ���������������µĹ۲����
[SE_sats.Windows,SATs.sats_opp] = tasks_alloc_funs.Windows_Sats_opp(SE_sats,duration,SE_tasks);
% �õ������������µ�����ȣ���ֵԽ�ߣ��۲����ԽС���������Դ�������Խ��
SATs.sats_oppweight = tasks_alloc_funs2.Opp_Weight(tasks_num,sats_num,...
    SATs.sats_opp,W);
SATs.sats_opp1 = SATs.sats_opp(1:end,1:SATS_N(1)); % �ӹ滮����1������Ĺ۲����
SATs.sats_opp2 = SATs.sats_opp(1:end,SATS_N(1)+1:SATS_N(1)+SATS_N(2)); % �ӹ滮����2������Ĺ۲����
%% ��������������Чʱ�䴰����Щ������ĳ�������������ɸ�ʱ�䴰����Ԫ�����ʽ
SE_sats.TW_tasks = tasks_alloc_funs.TW_tasks(SATs.sats_opp,SE_sats,SE_tasks);
%% ��������������Чʱ�䴰��������ĳ�����˻���ֻ��1��ʱ�䴰���Ծ������ʽ
% ��ͬ�Ļ��أ������Ӧ����Чʱ�䴰��һ��
UAVs.TW_tasks_1 = tasks_alloc_funs.TW_tasks2(SE_tasks,tasks_num,UAVs_N(1),base1,V_UAV,t_pos);
UAVs.TW_tasks_2 = tasks_alloc_funs.TW_tasks2(SE_tasks,tasks_num,UAVs_N(2),base2,V_UAV,t_pos);
%% �������������˻��Ĺ۲���ᣬ�������˻��Ļ���λ�úͺ��٣��������ж��Լ�ʱ�䴰�����жϣ�
UAVs.opp1 = tasks_alloc_funs.UAVs_opp(tasks_num,UAVs_N(1),base1,t_pos,UAVs.TW_tasks_1,D_UAV);
UAVs.opp2 = tasks_alloc_funs.UAVs_opp(tasks_num,UAVs_N(2),base2,t_pos,UAVs.TW_tasks_2,D_UAV);
% �õ����������˻��µ�����ȣ���ֵԽ�ߣ��۲����ԽС���������Դ�������Խ��
UAVs.opp1weight = tasks_alloc_funs2.Opp_Weight(tasks_num,UAVs_N(1),...
    UAVs.opp1,W);
UAVs.opp2weight = tasks_alloc_funs2.Opp_Weight(tasks_num,UAVs_N(2),...
    UAVs.opp2,W);
%% ��ͻ�ȼ��㣬���ǳ�ͻ�ȼ�������˻���ͻ�ȼ���
%% ���ǳ�ͻ��Conflict���㣬ʱ�䴰���Ƿ�����ͻ,�������ǵĿɼ�ʱ�䴰�ں�С�����ǹ۲�
%   �����Ŀ����Ҫ����ʱ�䣬�����Ϊ�����������Чʱ�䴰�����˼��ض�������ͻ��
%   �������ʱ�䴰û�з������棬���㷢����ͻ�ĸ��ʡ�
Conflict_tasks.sats = zeros(tasks_num,sats_num);
% ��meige���ǹ۲��£�ÿ������ĳ�ͻ����
SATs.conflict_withtasks = cell(tasks_num,sats_num); 
for i = 1:tasks_num
    for j = 1:sats_num
        if ~isempty(SE_sats.TW_tasks{i,j}) % �������i������j�ϵ�ʱ�䴰Ϊ��
            [Conflict_tasks.sats(i,j),SATs.conflict_withtasks{i,j}] = ...
       tasks_alloc_funs.Conflict_degree1(i,j,SE_sats,W,tasks_num,duration); 
        else
            Conflict_tasks.sats(i,j) = 0;
        end
    end
end
% �������ǵ�ƽ����ͻ��
Conflict_tasks.sats_ave = tasks_alloc_funs2.Conflict_average(tasks_num,...
    sats_num,SATs.sats_opp,Conflict_tasks.sats);
% ���۲����Ϊ0��λ���ϵ�Ԫ���滻��inf
Conflict_tasks.sats(SATs.sats_opp == 0) = inf; 
Conflict_tasks.sats_ave(SATs.sats_opp == 0) = inf; 
%% ���˻���ͻ��Conflict���㣬�����ֳ�ͻ��һ�����������ͻ����һ����ʱ�䴰��ͻ
% �������1�����˻��ĳ�ͻ�Ⱥ͸�����������������ĳ�ͻ�̶ȡ�
[Conflict_tasks.uav1,UAVs.conflict_withtasks1] = tasks_alloc_funs.Conflict_degree2(tasks_num,UAVs_N(1),...
    UAVs.opp1,D_UAV,base1,t_pos,V_UAV,UAVs.TW_tasks_1,duration,W);
% �������1�����˻���ƽ����ͻ��
Conflict_tasks.uav1_ave = tasks_alloc_funs2.Conflict_average(tasks_num,...
   UAVs_N(1),UAVs.opp1,Conflict_tasks.uav1);
% �������2�����˻��ĳ�ͻ�Ⱥ͸�����������������ĳ�ͻ�̶ȡ�
[Conflict_tasks.uav2,UAVs.conflict_withtasks2] = tasks_alloc_funs.Conflict_degree2(tasks_num,UAVs_N(2),...
    UAVs.opp2,D_UAV,base2,t_pos,V_UAV,UAVs.TW_tasks_2,duration,W);
% �������2�����˻���ƽ����ͻ��
Conflict_tasks.uav2_ave = tasks_alloc_funs2.Conflict_average(tasks_num,...
   UAVs_N(2),UAVs.opp2,Conflict_tasks.uav2);
% ���۲����Ϊ0��λ���ϵ�Ԫ���滻��inf
Conflict_tasks.uav1(UAVs.opp1 == 0) = inf; 
Conflict_tasks.uav1_ave(UAVs.opp1 == 0) = inf; 
Conflict_tasks.uav2(UAVs.opp2 == 0) = inf; 
Conflict_tasks.uav2_ave(UAVs.opp2 == 0) = inf; 
%% ��Դ���Ķȵļ���
%% ���������ǹ۲��µ���Դ���Ķ�
Consumers.sats = tasks_alloc_funs.Consumer_Resource(tasks_num,sats_num,...
    SATs.sats_opp,0,base1,t_pos,duration,3000,1,140,D_UAV,0.5,0.5,0);
Consumers.uavs1 = tasks_alloc_funs.Consumer_Resource(tasks_num,UAVs_N(1),...
    UAVs.opp1,1,base1,t_pos,duration,3000,1,140,D_UAV,1/3,1/3,1/3);
Consumers.uavs2 = tasks_alloc_funs.Consumer_Resource(tasks_num,UAVs_N(2),...
    UAVs.opp2,1,base2,t_pos,duration,3000,1,140,D_UAV,1/3,1/3,1/3);








