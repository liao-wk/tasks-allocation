function  funs = task_alloc_funs4
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
funs.Neighborhood_structure_cross = @Neighborhood_structure_cross; % ����ṹ����
funs.Neighborhood_structure_insertion = @Neighborhood_structure_insertion; % ����ṹ����
funs.Neighborhood_structure_delete_Conflict = @Neighborhood_structure_delete_Conflict; % ����ṹɾ��
funs.Neighborhood_structure_delete_Need = @Neighborhood_structure_delete_Need; % ����ṹɾ��
end
%% �Ӻ���1������ṹ�������� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [solu,task1,task2] = Neighborhood_structure_cross(T_schedule,Tabu1,tasks_Opportunity_subplan,tasks)
% ���������ӹ滮���ĵ��ѵ��ȷ����е�����������н���
subplan_num = size(T_schedule,1);
pp = randperm(subplan_num);
sub1 = pp(1); % ѡ����ӹ滮����1
sub2 = pp(2); % ѡ����ӹ滮����2
index1 = find(T_schedule(sub1,:)>0);
index2 = find(T_schedule(sub1,:)>0);
index1_num = size(index1,2);
index2_num = size(index2,2);
ppp1 = randperm(index1_num);
ppp2 = randperm(index2_num);
task1 = index1(ppp1(1)); % �ӹ滮����1��������
task2 = index2(ppp2(1)); % �ӹ滮����2��������
% ���ɲ���Tabu1
while Tabu1(task1,task2) ~= 0
    % task1���ӹ滮����2���й۲���ᣬ���˳�
    while ~tasks_Opportunity_subplan(sub2,task1)
        ppp1 = randperm(index1_num);
        task1 = index1(ppp1(1));
    end
    % task2 ͬ��
    while ~tasks_Opportunity_subplan(sub1,task1)
        ppp2 = randperm(index2_num);
        task2 = index1(ppp2(1));
    end
end
% task1��task2���н���
solu = T_schedule;
solu(sub1,task1) = 0;
solu(sub2,task1) = 1;
solu(sub1,task2) = 1;
solu(sub2,task2) = 0;
% ͬʱ�������ӹ滮����ɾȥ��С����ȵ�����ɾȥ���������������ִ�е��ӹ滮����
index_sub1 = find(solu(sub1,:)>0); % �ӹ滮����sub1�з��䵽������
index_sub2 = find(solu(sub2,:)>0); % �ӹ滮����sub2�з��䵽������
[C,index_task1] = min(tasks(index_sub1',9));
[D,index_task2] = min(tasks(index_sub2',9));
solu(sub1,index_task1) = 0;
solu(sub2,index_task2) = 0;
% ��ɾ����������뵽������ִ�е��ӹ滮������
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
%% ɾ��ÿ���ӹ滮����ƽ����ͻ����������ѡ�������ӹ滮���Ĳ���
%  ����ÿ���ӹ滮�����ѷ��������ƽ����ͻ��
W = tasks(:,2);
% index1���ӹ滮����sub1�ѵ��ȵ�����������
Schedule_sub1 = index1; % ���ӹ滮����sub1�ѵ��ȵ�����
index1_num = size(index1,2); % ���ӹ滮����sub1�ѵ��ȵ���������
% �ж�sub1�������ֹ۲���Դ
if sub1 == 1 || sub2 == 2
    % ��������������ӹ滮���ĵĳ�ͻ��
    sub1_num = 0;
    clear sub_R;
    sub_R = []; % �ӹ滮����sub1��������Դ���
    for k = 1:subplan_num
        if subplan(k,2) == sub1
            sub1_num = sub1_num+1; % �ӹ滮����sub1�Ĺ۲���Դ����
            sub_R = [sub_R,subplan(k,2)];% �ӹ滮����sub1��������Դ���
        end
    end
    % ��Schedule,���Ǹ���Schedule_sub1��tasks_Opportunity�õ���
    Schedule = zeros(tasks_num,r_num);
    for i = 1:index1_num
        for j = 1:r_num
            if tasks_Oppotunity(Schedule_sub1(i),j) ~= 0 && subplan(j,2) == sub1
                Schedule(Schedule_sub1(i),j) = 1;
            end
        end
    end
    Conflict = zeros(index1_num,sub1_num); % ��2���Ƕ�Ӧ�ĳ�ͻ��
    Conflict(:,1) = index1; % ��1��Ϊ�ӹ滮����sub1�ѵ��ȵ�������
    for i = 1:index1_num
        for j = 1:sub_num
            R_code = sub_R(j); % ��Դ�ı��
            Conflict(i,2) = Conflict_value_sats_schedule(sats,Schedule_sub1(i),R_code,W,Schedule);
        end
    end
    
elseif sub1 == 3 ||sub1 == 4
 % ��������������ӹ滮���ĵĳ�ͻ��
    sub1_num = 0;
    clear sub_R;
    sub_R = []; % �ӹ滮����sub1��������Դ���
    for k = 1:subplan_num
        if subplan(k,2) == sub1
            sub1_num = sub1_num+1; % �ӹ滮����sub1�Ĺ۲���Դ����
            sub_R = [sub_R,subplan(k,2)];% �ӹ滮����sub1��������Դ���
        end
    end
    % ��Schedule,���Ǹ���Schedule_sub1��tasks_Opportunity�õ���
    Schedule = zeros(tasks_num,r_num);
    for i = 1:index1_num
        for j = 1:r_num
            if tasks_Oppotunity(Schedule_sub1(i),j) ~= 0 && subplan(j,2) == sub1
                Schedule(Schedule_sub1(i),j) = 1;
            end
        end
    end
    Conflict = zeros(index1_num,sub1_num); % ��2���Ƕ�Ӧ�ĳ�ͻ��
    Conflict(:,1) = index1; % ��1��Ϊ�ӹ滮����sub1�ѵ��ȵ�������
    for i = 1:index1_num
        for j = 1:sub_num
            R_code = sub_R(j); % ��Դ�ı��
            Conflict(i,2) =Conflict_Caculate_schedule(TW_uavs,tasks,base,V_UAV,D_UAV,Schedule_sub1(i),R_code,Schedule);
        end
    end
    
end
end
%% �Ӻ���2������ṹ����δ������� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function solu = Neighborhood_structure_insertion(solu,T_unschedule,R_subplan,...
    Conflict_tasks,subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV,Tabu2)
tasks_alloc_funs3 = task_alloc_funs3; % �����ű���ֵ

tasks_num = size(T_unschedule,2);
subplan_num = size(T_unschedule,1);
r_num = size(subplan,1);
duration = tasks(:,7);
%% ��δ��������������Ƚ��дӴ�С�������ٰ��ջ������Ŷȷ���
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
UnSchedule(q:end,:) = []; % ɾ������
[A,index3] = sort(UnSchedule(:,2),'descend');
UnSchedule = UnSchedule(index3,:);% ������ȴӴ�С����õ�δ���������UnSchedule
UnSchedule_tasks = UnSchedule(:,1);

%    ��������ɲ���ǰ���£�����С��ͻ��ԭ��UnSchedule�е������������ӹ滮����
%    �ڷ����ʱ���Ǹ��ӹ滮������Դ�����ƣ���Դ���������ӹ滮����Ϊ��λ

% ������ȴӴ�С��δ��������UnSchedule�е�������з���
UnSchedule_num = size(UnSchedule,1);
for i = 1:UnSchedule_num
    clear RC;
    clear subplan_list;
    k = UnSchedule_tasks(i,1); % �������ߵ�����ı��
    R_use = find(Conflict_tasks.average(k,:) < inf); % �ҳ�����Ŀ�����Դ,������
    C_Tab = tasks_alloc_funs3.Conflict_Caculate(subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV,solu,k); % �������ѵ��ȵ������ڿ�����Դ�µĳ�ͻ��
    RC(1:2,:) = [R_use;C_Tab]; % ��ѡ��Դ�ı�źͳ�ͻ�ȵĺϳ�
    RC_num = size(RC,2); % ����k�ľ��й۲����Ĺ۲���Դ����
    
    % ��RC�Ļ��������һ�У�RC��3��Ϊ��Ӧ���ӹ滮���ı��
    for q = 1:RC_num
        for qq = 1:r_num
            if RC(1,q) == subplan(qq,1)
                RC(3,q) = subplan(qq,2);
                break
            end
        end
    end
    
    %% ������ӹ滮���ĵĻ������Ŷȣ���ʣ����Դ���ӹ滮���ĵĻ������Ŷ�Ϊ0
    subplan_list2 = unique(RC(3,:)); % �ӹ滮���ı�����ظ���������
    subplan_list_num = size(subplan_list2,2); % ���õ��ӹ滮��������
    subplan_list(1,:) = subplan_list2; % subplan_list��1�����ӹ滮���ĵı��
    subplan_list(2,:) = zeros(1,subplan_list_num); % subplan_list��2��������k�ڶ�Ӧ���ӹ滮���ĵĻ������Ŷ�
    % ������ӹ滮���ĵ�ƽ����ͻ�ȵľ��岽�� $$ �������Ŷ� $$
    for m = 1:subplan_list_num % �����������k�ľ��й۲������ӹ滮����
        % ͬһ�ӹ滮�����ܵĳ�ͻ�ȣ������ͻ�����󻥲����Ŷ�
        for n = 1:RC_num
            if RC(3,n) == subplan_list(1,m)
                subplan_list(2,m) = subplan_list(2,m)+RC(2,n);
            end
        end
    end
    % ��������k�ڸ������й۲������ӹ滮���ĵĻ������Ŷ�
    subplan_list(2,:) = max(subplan_list(2,:))-subplan_list(2,:);
    for m = 1:subplan_list_num
        subplan_code = subplan_list(1,m); % �ӹ滮���ı��
        if R_subplan(subplan_code,1)<1 && R_subplan(subplan_code,2)< duration(k)
            subplan_list(2,m) = 0; % ����ӹ滮�����޾���ʣ����Դ�Ĺ۲���Դ,�������Ŷ�Ϊ0
        end
    end
    
    
    %% ���������ŶȽ��дӴ�С���򣬵�1���Ƕ�Ӧ���ӹ滮���ģ���2���Ƕ�Ӧ�Ļ������Ŷȡ�
    [B,index2] = sort(subplan_list(2,:),'descend');
    subplan_list = subplan_list(:,index2); % ͬ��ע��
    % ������k���䵽��d���ӹ滮���ģ����ӹ滮���Ŀ�����Դ����ʣ����Դ
    d = 1;
    while d <= subplan_list_num
        p_sub = subplan_list(1,d); % subplan_list�ĵ�1�����ӹ滮���ı��
        % ������ɲ���Tabu2,������Դ���ġ�
        if R_subplan(p_sub,1)>=1 && R_subplan(p_sub,2)>=duration(k) && Tabu2(p_sub,k)==0
            solu(p_sub,k) = 1; % ��δ��ɵ�������뵽�ӹ滮������
            R_subplan(p_sub,1) = R_subplan(p_sub,1)-1; % ���ػ�����
            R_subplan(p_sub,2) = R_subplan(p_sub,2)-duration(k); % �����۲�ʱ��
            break
        else
            d = d+1;
        end
    end
    if d == subplan_list_num+1
        solu(1,k) = 1; % ��δ��ɵ�������뵽�ӹ滮������
    end
end
end
%% �Ӻ���3������ṹɾ������������һ���ӹ滮�����ѵ��ȵ�������ѡ��һ���������Ŷ���С������%%%%%%%%%%%%%%%%%%
% ����ɾ����Ȼ����ӵ�����ӹ滮���ĵ�δ����������
function [T_schedule,T_unschedule,p_sub,t] = Neighborhood_structure_delete_Conflict(T_schedule,...
    T_unschedule,tasks,subplan,tasks_Opportunity,sats,uavs,base1,base2,V_UAV,D_UAV,Tabu1)
tasks_alloc_funs = task_alloc_funs1; % �����ű���ֵ
tasks_alloc_funs2 = task_alloc_funs2; % �����ű���ֵ
% ����ѡ������һ���ӹ滮����p_sub
tasks_num = size(T_schedule,2);
r_num = size(subplan,1);
W = tasks(:,2);
subplan_num = max(subplan(:,2)); % �ӹ滮���ĵ�����
p_sub = ceil(rand*subplan_num); % ����ѡ����ӹ滮���ĵı��
%% ���ѵ���������ѡ�񻥲����Ŷ���С�����񣬽���ɾ��
% �ȵõ����ӹ滮�����ѵ��ȵ����񼯣���01���룩
P_Schedule = find(T_schedule(p_sub,:)==1); % ������
P_Schedule_num = size(P_Schedule,2); % �ѵ������������
P_Conflict = zeros(P_Schedule_num,2); % ����ĳ�ͻ�ȣ���1���������ţ���2���ǳ�ͻ��
P_Conflict(:,1) = P_Schedule; % ��1����������
% ���ӹ滮����p_sub�𿪳ɶ�Ӧ�Ĺ۲���Դ�������ڹ۲���Դ�µĵ��Ⱦ���
% ���Ⱦ�����tasks_num�У�r_num�У��ѵ��ȵ������ҿ��õĹ۲���Դ����p_sub����Ӧ��Ԫ��Ϊ0
Schedule = zeros(tasks_num,r_num);
for i = 1:P_Schedule_num
    for j =1:r_num
        t = P_Schedule(i); % ����t
        if tasks_Opportunity(t,j)>0 && subplan(j,2)==p_sub
            Schedule(t,j) = 1; % ����������p_sub�Ŀ��ù۲���Դ�µĵ��Ⱦ���
        end
    end
end

for i = 1:P_Schedule_num
    %% ��������t���ӹ滮����p_sub�ϵĳ�ͻ��
    t = P_Schedule(i); % ����t
    % �����ж��ӹ滮����p_sub�������ֹ۲���Դ
    if p_sub == 1||p_sub == 2
        for j = 1:r_num
            % �жϹ۲���Դ�Ƿ�����p_sub������t�ڸ���Դ�¿���
            if subplan(j,2) == p_sub && tasks_Opportunity(t,j)>0
                P_Conflict(i,2)=P_Conflict(i,2)+tasks_alloc_funs.Conflict_value_sats_schedule(sats,t,j,W,Schedule);
            end
        end
        
        
    elseif p_sub == 3||p_sub == 4
        if p_sub == 3
            base = base1; % p_sub�Ļ���
            TW_uavs = uavs.TW_uavs1; % p_sub����Чʱ�䴰
        else
            base = base2;
            TW_uavs = uavs.TW_uavs2;
        end
        
        for j = 1:r_num
            % �жϹ۲���Դ�Ƿ�����p_sub������t�ڸ���Դ�¿���
            if subplan(j,2) == p_sub && tasks_Opportunity(t,j)>0
                P_Conflict(i,2)=P_Conflict(i,2)+tasks_alloc_funs2.Conflict_Caculate_schedule(TW_uavs,tasks,base,V_UAV,D_UAV,t,j,Schedule);
            end
        end
        
    end
end
%% p_sub�ѵ������񼯵Ļ������Ŷȣ����������е�����ͻ�ȼ���ͻ�Ⱦ��󣬻������Ŷ�ԽС����ִ�����Խ��
P_unConflict = P_Conflict; % p_sub�ѵ������񼯵Ļ������ŶȾ��󣬵�1���ѵ��������ţ���2�л������Ŷ�
P_unConflict(:,2) = max(P_Conflict(:,2)) - P_Conflict(:,2);


% ��P_unConflict���ճ�ͻ�ȴ�С�������� 
[A,index1] = sort(P_unConflict(:,2));
P_unConflict = P_unConflict(index1,:);
% ɾ���������Ŷ���С�����񣬲��뵽T_unschedule�����ɲ��� 
i = 1;
t = P_unConflict(i,1); % ����t
while Tabu1(p_sub,t) ~= 0 % ���������,��������
    i = i+1;
    if i == P_Schedule_num+1
        t = P_unConflict(1,1); % ����t
        break
    end
    t = P_unConflict(i,1); % ����t
end
T_unschedule(p_sub,t) = 1;
T_schedule(p_sub,t) = 0;

end
%% �Ӻ���4��������С�����ɾ���ӹ滮���ĵ�����
function [T_schedule,T_unschedule,p_sub,t2] = Neighborhood_structure_delete_Need(T_schedule,...
    T_unschedule,tasks,subplan,Tabu1)
% ����ѡ������һ���ӹ滮����p_sub
subplan_num = max(subplan(:,2)); % �ӹ滮���ĵ�����
p_sub = ceil(rand*subplan_num); % ����ѡ����ӹ滮���ĵı��
%% ���ѵ���������ѡ��ƽ����ͻ���������񣬽���ɾ��
% �ȵõ����ӹ滮�����ѵ��ȵ����񼯣���01���룩
P_Schedule = find(T_schedule(p_sub,:)==1); % ������
P_Schedule_num = size(P_Schedule,2); % �ѵ������������
%% ��p_sub���ѵ�������P_Schedule��ɾ���������С������
Need = zeros(P_Schedule_num,2); % ��1�����ѵ��������ţ���2�ж�Ӧ�������
Need(:,1) = P_Schedule';
Need(:,2) = tasks(P_Schedule',9);
% ������ȶ��ѵ��ȵ�������д�С���������
[A,index2] = sort(Need(:,2));
Need = Need(index2,:);
% ���ɲ��ԣ�����T_scheduleɾ��t2��T_unschedule��
i = 1;
t2 = Need(i,1); % ����t2
while Tabu1(p_sub,t2) ~= 0 % ���ɲ��ԣ�ֱ������t2������
    i = i+1;
    if i == P_Schedule_num+1
        t2 = Need(1,1); % ����t2
        break
    end
    t2 = Need(i,1); % ����t2
end
T_unschedule(p_sub,t2) = 1;
T_schedule(p_sub,t2) = 0;

end


