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
%% �����ľ�γ�ȣ�t_pos
% �����Ȩ�أ���ֵ��
W = load("weight.txt");
W = W';
lats = load("X.txt");
lons = load("Y.txt");
size_t = size(lats); % ����γ�ȵĴ�С������������ʾ����ĸ���
t_pos = zeros(size_t(2),2);
t_pos(1:end, 1) = lats';
t_pos(1:end, 2) = lons';
% �����Ӳ��ʱ�䴰
TW_tasks = zeros(size_t(2), 2);
st = load("Estime.txt");
TW_tasks(1:end,1) = st';
le = load("letime.txt");
TW_tasks(1:end,2) = le';
%% ���ݿɼ�ʱ�䴰���ж������Ƿ����ǹ۲⵽��ÿ16�����ݱ�ʾ1������
% �����Ŀɼ�ʱ�䴰
Windows = load("window.txt");
size_W = size(Windows); % ʱ�䴰�ھ���Ĵ�С
if size_t(2) ~= size_W(1)
    disp("������©�˲�������Ĺ۲�������")
end
sats_opp = zeros(size_t(2),size_W(2)/16); % ����i�����ǹ۲�Ļ��ᣬ1��ʾ���Ա��۲⣬����Ϊ0
for i = 1:size_W(1)
    indexes = find(Windows(i,1:end) > 0);
    sats_opp(i,floor(indexes/16)+1) = 1;
end
sats_opp1 = sats_opp(1:end,1:SATS_N(1)); % �ӹ滮����1������Ĺ۲����
sats_opp2 = sats_opp(1:end,SATS_N(1)+1:SATS_N(1)+SATS_N(2)); % �ӹ滮����2������Ĺ۲����
%% �������˻��Ļ���λ�úͺ��٣��ж������ܷ����˻��۲⵽
UAVs_opp1 = zeros(size_t(2),UAVs_N(1)); % ���˻�����1(�ӹ滮����3)������Ĺ۲����
UAVs_opp2 = zeros(size_t(2),UAVs_N(2)); % ���˻�����2(�ӹ滮����4)������Ĺ۲����
tasks_alloc_funs = task_alloc_funs;
for i = 1:size_t(2)
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
%% ���ǳ�ͻ�ȼ��㣬ʱ�䴰���Ƿ�����ͻ



