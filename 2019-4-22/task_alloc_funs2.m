function funs = task_alloc_funs2
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
funs.Conflict_average = @Conflict_average;
funs.Opp_Weight = @Opp_Weight; % �õ������������µ������
funs.Initial_solution = @Initial_solution; % ����ĳ�ֹ���õ���ʼ��
funs.conflict_check = @conflict_check; % ��ͻ�ȼ��
funs.conflict_dis_time = @conflict_dis_time; % ���˻��ľ����ͻ��ʱ�䴰�ж�
funs.resource_check = @resource_check; % ʣ����Դ���
end
%% �Ӻ���1�����������ڲ�ͬ�Ĺ۲���Դ�µ�ƽ����ͻ��
function Conflict_ave = Conflict_average(tasks_num,resource_num,opp,Conflict)
Conflict_ave = zeros(tasks_num,resource_num);
% �����������д���0��ÿһ�е�Ԫ�ظ���
for j = 1:resource_num
    A = opp(opp(:,j) > 0); % ��������д���0��Ԫ������
    num = size(A,1)-1; % ��������������һ
    Conflict_ave(:,j) = Conflict(:,j)/num; % ����Դr�µĸ��������ƽ����ͻ��
end
end
%% �Ӻ���2�����������ڹ۲���Դ�µ�����ȣ���ֵԽ�ߣ��۲����ԽС���������Դ�������Խ��
function oppweight = Opp_Weight(tasks_num,resource_num,opp,W)
oppweight = zeros(tasks_num,resource_num);
for j = 1:resource_num
    for i = 1:tasks_num
        if opp(i,j) ~= 0
            oppweight(i,j) = W(i)/opp(i,j); % ����i��Ȩ�س�������Դj�µĻ���
        else
            oppweight(i,j) = 0; % �����������Դ��û�й۲���ᣬ��ô�����Ϊ0.
        end
    end
end
end
%% �Ӻ���3����ȡ��ʼ�� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Solu_Tabu = Initial_solution(Resou_num,tasks_num,Conflict_tasks,Tasks,SE_sats,duration,UAVs,t_pos,...
    sats_start,sats_end,uavs1_start,uavs1_end)
Tasks_Tabu = zeros(tasks_num,Resou_num); % ��������䵽��ͬ����Դ��
tasks_demand = zeros(tasks_num,2);
tasks_demand(:,2) = Tasks.Opp_weight;
% ��tasks_demand����2�е����ִ�С��������
[tasks_demand(:,2),index] = sort(tasks_demand(:,2),'descend');
tasks_demand(:,1) = index; % index��������
vir_re = zeros(1,tasks_num); % �鹹��Դ,���������д�������
% TODO�������񵽸���Դ���У�������Ҫ�ж�δ����������Ƿ�����ѷ���������ͻ
for i = 1:tasks_num
    p = tasks_demand(i,1); % ��Ӧ��������
    % �ҵ�ÿ�������Ӧ�Ĺ۲����
    % ��ÿ���������һ����Դ��ѡ��,���������Ӧ�Ĺ۲��������ѡ��
    Candidates = find(Tasks.Opportunity(i,:)>0);
    C_size = size(Candidates,2); % ��Դ��ѡ���ĸ���
    conflict = zeros(2,C_size);  % ����i�ڸ���Դ�ĳ�ͻ��,��1������Դ������
    % ��2���������ڶ�Ӧ��Դ�ĳ�ͻ�ȴ�С
    conflict(1,:) = Candidates;
    conflict(2,:) = Conflict_tasks.Ave_conflict(i,Candidates);
    conflict_values = conflict;
    if C_size == 0 % ���û�пɹ۲���Դ������䵽������Դ����
        vir_re(p) = 1; 
    elseif C_size == 1 % ���ֻ��һ����Դ��������䵽��Ӧ����Դ����
        Tasks_Tabu(p,conflict(1,1)) = 1; 
    else % �кü����ɹ۲�Ĺ۲���Դ
        %  ð�����򷨣���ƽ����ͻ�Ƚ��д�С��������
        for x = 1:C_size
            conflict_values(:,x) = conflict(:,1);
            for y = x:C_size
                if conflict_values(2,x) > conflict(2,y)
                    temp = conflict_values(:,x);
                    conflict_values(:,x) = conflict(:,y);
                    conflict(:,y) = temp;
                end
            end
        end
    end
    % ѡ����Դ��ѡ���У�ƽ����ͻ����С����Դ���з��䣬ֱ�����񱻷��䵽��ʵ��Դ��������ԴΪֹ
    % ��ͻ�����飺��ǰ�������ѵ��������Ƿ�����ͻ
    % ��Դ���Ƽ�飺ʣ��ʱ����Դ�Ϳ��ػ���������
    conflict = conflict_values;
    for j = 1:C_size
        % ��ȡ����Դ������Щ�ѵ��ȵ�����
        resource_name = conflict(1,j); % �ҵ���j����Դ
        % �ѵ��ȵ��������ڵ�j����Դ�¹۲�������1������
        indexes = find(Tasks_Tabu(:,j)==1);
        % �ж������Ƿ�������Դresource_name���ѵ��ȵ������Ƿ��ͻ��0��ʾ����ͻ������Ϊ1
        check_num = conflict_check(resource_name,SE_sats,indexes,duration,UAVs,t_pos,p,...
    sats_start,sats_end,uavs1_start,uavs1_end);
        if check_num == 0 
            % ����ͻ�Ͱ�������䵽�����Դ���У�������䵽��һ����Դ���У�ֱ������Դ�ɷ�
            Tasks_Tabu(p,resource_name) = 1;
            break
        end
        if j == C_size
            vir_re(p) = 1;
        end
    end
end
% �����񶼷��䵽��Ӧ��Դ���к󣬰�����ȡ����
Tasks_Tabu = [Tasks_Tabu,vir_re']; % �õ���ʵ��Դ��������Դ������� 
Solu_Tabu = zeros(Resou_num+1,tasks_num);
for i = 1:Resou_num+1
    index = find(Tasks_Tabu(:,i) > 0);
    % ������������Ԫ�صĸ���
    m = size(index,1);
    Solu_Tabu(i,1:m) = index;
%     Solu_Tabu(i,m+1:end) = [];
end
end
%% �Ӻ���4����ͻ��飬δ�����������ѷ��������ĳ�ͻ��� %%%%%%%%%%%%%%%%%%%
function check_num = conflict_check(resource_name,SE_sats,indexes,duration,UAVs,t_pos,i,...
    sats_start,sats_end,uavs1_start,uavs1_end)
% ��ͻ���ָ��ʱ�Ƿ�ᷢ����Ȼ���棬���ǵı�Ȼ��ͻָ������Чʱ�䴰����
% ���˻��Ľ��濼���������ʱ�䴰����
% ���ж���Դj����������Դ���ѷ�������Դj������tasks_arranged,��Ӧ��ʱ�䴰ΪԪ����ʽ
% �ѷ���������������µ�ʱ�䴰��Ԫ�����ʽ
% �ѷ�������������˻��µ�ʱ�䴰�Ǿ������ʽ
j = resource_name;
check_num = 0; % ���費���ڳ�ͻ
% ����ط������ˣ��ر�ע�Ⱑ����������������������������������������!!!!!!!!!
if  j >= sats_start && j <= sats_end % ��Դj���������ж�
    % ��ȡ����i����Դj�µ�ʱ�䴰Ԫ��
    tw_objs = SE_sats.TW_tasks{i,j};
    t = size(tw_objs,1); % ����i��ʱ�䴰Ԫ������
    check_num = zeros(1,t); % ����i����Դj����t��ʱ�䴰��������
    for p = 1:t
        tw_obj = tw_objs(p,:); % ��ȡ����i����Դj�µ�һ����Чʱ�䴰
        m = size(indexes,1); % �ѷ������������
        % ��ȡ�ѷ������������Դj�µ�ʱ�䴰Ԫ��
        % ���ж�ȡÿһ�е�ʱ�䴰���ж��Ƿ���ڽ�������
        for q = 1:m
            TW = SE_sats.TW_tasks{indexes(q),j};
            % �ж�TWԪ���м���
            n = size(TW,1);
            check_num(1,p) = 1; % ���������Чʱ�䴰�Ľ���
            % ���ж�ȡ�ѷ�������ʱ�䴰Ԫ���е��У��������������i�������ʱ�䴰��break
            for k = 1:n
                tw = TW(k,:); % ��ȡ��k��ʱ�䴰
                if tw(2)-tw(1)+tw_obj(2)-tw_obj(1)-(max([tw(2),tw_obj(2)])-...
                        min([tw(1),tw_obj(1)])) <= 0
                    check_num(1,p) = 0; % �����ڽ���
                end
            end
            % ��������ѷ��������ʱ�䴰��������i�ĵ�p��ʱ�䴰���棬����ѭ��
            if check_num(1,p) == 1
                break
            end
        end
    end
    % �õ�����i����Դj�µ�ÿһ��ʱ�䴰��ͻ����������Ƿ���0�Ĵ��ڣ�����У�˵���ɷ�������Դj
    for p = 1:t
        if check_num(1,p) == 0
            check_num = 0; % �ɷ���
            break
        else
            check_num = 1; % ���ɷ�������Դj��
        end
    end
else % ��Դj�������˻�
    if j >= uavs1_start && j <= uavs1_end
    % ��ȡ����i����Դj�µ�ʱ�䴰,�����˻��۲���ֻ��һ��ʱ�䴰
    % TODO�ж��ѷ���������Ƿ��������i��ͻ,�������Լ�ʱ�䴰
    % ����ط������ˣ��ر�ע�Ⱑ����������������������������������������
        tw_obj = UAVs.TW_tasks_1(i,:);
        m = size(indexes,1); % �ѷ������������
        for q = 1:m
            tw = UAVs.TW_tasks_1(indexes(q),:); % ��ȡ�ѷ��������ʱ�䴰
            % �������ͻ�Լ�ʱ�䴰��ͻ
            dur1 = duration(i);
            dur2 = duration(indexes(q));
            t_pos1 = t_pos(i,:);
            t_pos2 = t_pos(indexes(q),:);
            check_num = conflict_dis_time(t_pos1,t_pos2,tw_obj,tw,dur1,dur2,base,V_UAV,D_UAV);
            if check_num == 1 % ���������δ���������Ȼ��ͻ���ѷ������񣬾��˳�
                break
            end
        end
    else
        tw_obj = UAVs.TW_tasks_2(i,:);
        m = size(indexes,1); % �ѷ������������
        for q = 1:m
            tw = UAVs.TW_tasks_2(indexes(q),:); % ��ȡ�ѷ��������ʱ�䴰
            % �������ͻ�Լ�ʱ�䴰��ͻ
            dur1 = duration(i);
            dur2 = duration(indexes(q));
            t_pos1 = t_pos(i,:);
            t_pos2 = t_pos(indexes(q),:);
            check_num = conflict_dis_time(t_pos1,t_pos2,tw_obj,tw,dur1,dur2,base,V_UAV,D_UAV);
            if check_num == 1 % ���������δ���������Ȼ��ͻ���ѷ������񣬾��˳�
                break
            end
        end
    end   
end
end
%% �Ӻ���5���������ͻ��ʱ�䴰��ͻ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function check_num = conflict_dis_time(t_pos1,t_pos2,tw_obj,tw,dur1,dur2,base,V_UAV,D_UAV)
% ����i��δ�������������q���ѷ��������
% ��������ڻ���1�ģ�����base1������ʹ��base2.
% tw_obj��������i����Чʱ�䴰��tw����δ��������p����Чʱ�䴰
% dur1������i�ĳ���ʱ�䣬dur2��δ��������p�ĳ���ʱ��
% t_pos1 ������i��λ�ã� t_pos2 ��δ��������p��λ�ã�
check_num = 0; 
distance3 = distance_value(base,t_pos1); % ����i�����֮��ľ���
distance1 = distance_value(t_pos2,t_pos1); % ����p������i֮��ľ���
time1 = distance1/V_UAV*3600; % ����p������i֮��ķ���ʱ��
distance2 = distance_value(base,t_pos2); % ����p�����֮��ľ���
distance = distance1+distance2+distance3;
% �������жϣ��ض���ͻ��
if distance > 2*D_UAV
    check_num = 1;
elseif tw_obj(1) >= tw(2) && tw(1)+time1+dur2 > tw_obj(2)
    check_num = 1;
elseif tw(1) >= tw_obj(2) && tw_obj(1)+time1+dur1 > tw(2)
    check_num = 1;
end
end


