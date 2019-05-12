function funs = task_alloc_funs2
%�������˻������ݴ���
%UNTITLED3 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
funs.distance_value = @distance_value; % ����������֮��ľ��루��γ�ȣ�
funs.TW_tasks_uavs = @TW_tasks_uavs; % �������˻��۲��µ���Чʱ�䴰
funs.Opportunity = @Opportunity; % �������˻��Ĺ۲����
funs.Conflict_value_uavs = @Conflict_value_uavs; % �������˻��ĳ�ͻ��
funs.Conflict_Caculate = @Conflict_Caculate; %���㵥�����������˻��µĳ�ͻ��
funs.Conflict_Caculate_schedule = @Conflict_Caculate_schedule; %�������������˻������ѵ�������ĳ�ͻ��
funs.Probability_conflict_type1 = @Probability_conflict_type1; % ��һ�ཻ��ĳ�ͻ�ȼ���
funs.Probability_conflict_type2 = @Probability_conflict_type2; % �ڶ��ཻ��ĳ�ͻ�ȼ���
funs.fit_addition = @fit_addition; % ���㸽������Ӧ��
end
%% �Ӻ���1����������������֮��ľ��루��γ�ȣ�%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function distance = distance_value(t_pos1,t_pos2)
r = 6371;
distance = 2*r*asin(sqrt(sin(0.5*(t_pos1(1)-t_pos2(1))).^2+cos(t_pos1(1))*...
    cos(t_pos2(1))*sin(0.5*(t_pos1(2)-t_pos2(2))).^2));
end
%% �Ӻ���2���������˻��۲��µ���Чʱ�䴰 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function TW = TW_tasks_uavs(tasks,base,V_UAV,D_UAV)
% 1.������󺽳�Լ����2.��������Ĺ涨�۲�����ʱ�������ʱ���Լ�����ʱ��
tasks_num = size(tasks,1); % ���������
t_pos = tasks(:,3:4); % �����λ�õ�
SE_tasks = tasks(:,5:6); % ����涨�۲�����ʱ�������ʱ��
duration = tasks(:,7); % ��������ĳ����۲�ʱ��
TW = zeros(tasks_num,2);
for i = 1:tasks_num
    % �������������λ�õ�֮��ľ���
    task_position = t_pos(i,:);
    distance = distance_value(base,task_position);
    % �������жϣ�������������룬ʱ�䴰Ϊ0
    if distance >= D_UAV
        TW(i,1) = 0;
        TW(i,2) = 0;
    else % ���Ŀ�������󺽳̷�Χ��
        tw_start = distance/V_UAV*3600; % ת���ɵ�λs
        TW(i,1) = max([tw_start,SE_tasks(i,1)]);
        TW(i,2) = SE_tasks(i,2);
    % ������µ�ʱ�䴰�Ŀ�ʼʱ����ϳ����۲�ʱ����ڽ���ʱ�䣬��ô��Ϊ0������
        if  TW(i,2) - TW(i,1) < duration(i)    
            TW(i,1) = 0;
            TW(i,2) = 0;
        end
    end
end
end
%% �Ӻ���3���������������˻��µĹ۲���� %%%%%%%%%%%%%%%%%%%%%%%%%%
function opp = Opportunity(tasks,UAVs_N,TW_uavs)
tasks_num = size(tasks,1); 
opp = zeros(tasks_num,UAVs_N); % ���˻����ض�����Ĺ۲����
for i = 1:tasks_num
    if TW_uavs(i,1) > 0 || TW_uavs(i,2) > 0
        opp(i,:) = 1;
    else
        opp(i,:) = 0;
    end
end
end
%% �Ӻ���4���������������˻��µĳ�ͻ�� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Conflicts = Conflict_value_uavs(UAVs_opp,TW_uavs,tasks,base,V_UAV,D_UAV)
tasks_num = size(UAVs_opp,1); 
UAVs_N = size(UAVs_opp,2);
Conflicts = zeros(tasks_num,UAVs_N); % �������ڸ����˻��µĳ�ͻ�Ⱦ���
for i = 1:tasks_num
    if UAVs_opp(i,1) ~= 0
         % ����i�����˻�j�ĳ�ͻ��
         Conflicts(i,:) = Conflict_Caculate(TW_uavs,tasks,base,V_UAV,D_UAV,i);
    else % �������i�����˻�j��û�й۲����
        Conflicts(i,:) = inf; 
    end
end
end
%% �Ӻ���5�����㵥�����������˻��µĳ�ͻ�� %%%%%%%%%%%%%%%%%%%%%%%%%%%
function conflict = Conflict_Caculate(TW_uavs,tasks,base,V_UAV,D_UAV,i)
conflict = 0;
t_pos = tasks(:,3:4);
duration = tasks(:,7);
W = tasks(:,2);
tasks_num = size(tasks,1);
distance3 = distance_value(base,t_pos(i,:)); % ����i�����֮��ľ���
tw1 = TW_uavs(i,:); % ����i����Чʱ�䴰
for p = 1:tasks_num
    if p == i || TW_uavs(p,1)+TW_uavs(p,2) == 0 % ʱ�䴰Ϊ0�������޹۲����
        continue
    else
        tw2 = TW_uavs(p,:); % ����p����Чʱ�䴰
        distance1 = distance_value(t_pos(p,:),t_pos(i,:)); % ����p������i֮��ľ���
        time1 = distance1/V_UAV*3600; % ����p������i֮��ķ���ʱ��
        distance2 = distance_value(base,t_pos(p,:)); % ����p�����֮��ľ���
        distance = distance1+distance2+distance3;
        % �������жϣ��ض���ͻ��
        if distance > 2*D_UAV
            conflict = conflict + W(p);
        %  ʱ�������жϣ����ض���ͻ�жϣ�
        %  ��ִ��������p,�������i�Ŀ�ʼִ��ʱ���������p�����ʱ��
        elseif TW_uavs(i,1) >= TW_uavs(p,2)
            if TW_uavs(p,1)+time1+duration(p) > TW_uavs(i,2)
                conflict = conflict + W(p);
                % �����ܳ�ͻ�жϣ������ཻ������ʱ�䴰����������ͻ�����
                % �����ܳ�ͻ�жϣ����ཻ������ʱ�䴰����������ͻ�����
            elseif  TW_uavs(p,2)+time1+duration(p) > TW_uavs(i,1) &&...
                    TW_uavs(p,1)+time1+duration(p) <= TW_uavs(i,2)
                dur1 = duration(i);
                dur2 = duration(p)+time1;
                tw1(2) = tw1(2) - duration(i); % ����i������ִ��ʱ��
                tw2(2) = tw2(2) - duration(p); % ����p������ִ��ʱ��
                prob = Probability_conflict_type1(tw1,tw2,dur1,dur2);
                conflict = conflict + W(p)*prob;
            end
            %  ʱ�������жϣ����ض���ͻ�жϣ�����ִ��������i,�������p�Ŀ�ʼִ��ʱ���������i�����ʱ��
        elseif TW_uavs(p,1) >= TW_uavs(i,2)
            if TW_uavs(i,1)+time1+duration(i) > TW_uavs(p,2)
                conflict = conflict + W(p);
            % �����ܳ�ͻ�жϣ������ཻ������ʱ�䴰����������ͻ�����
            elseif  TW_uavs(i,2)+time1+duration(i) > TW_uavs(p,1) &&...
                    TW_uavs(i,1)+time1+duration(i) <= TW_uavs(p,2)
                dur1 = duration(i)+time1;
                dur2 = duration(p);
                tw1(2) = tw1(2) - duration(i); % ����i������ִ��ʱ��
                tw2(2) = tw2(2) - duration(p); % ����p������ִ��ʱ��
                prob = Probability_conflict_type1(tw1,tw2,dur1,dur2);
                conflict = conflict + W(p)*prob;
            end
        %  ���ܳ�ͻ�жϣ�������������Ľ���ʱ�䴰�����ͻ������
        else
           dur1 = duration(i)+time1; 
           dur2 = duration(p)+time1;
           tw1(2) = tw1(2) - duration(i); % ����i������ִ��ʱ��
           tw2(2) = tw2(2) - duration(p); % ����p������ִ��ʱ��
           prob = Probability_conflict_type2(tw1,tw2,dur1,dur2);
           conflict = conflict + W(p)*prob;
        end 
    end
end
end
%% �Ӻ���6�����㵥�����������˻������ѵ�������ĳ�ͻ�� %%%%%%%%%%%%%%%%%%
function conflict = Conflict_Caculate_schedule(TW_uavs,tasks,base,V_UAV,D_UAV,i,j,Schedule)
conflict = 0;
t_pos = tasks(:,3:4);
duration = tasks(:,7);
W = tasks(:,2);
tasks_num = size(tasks,1);
distance3 = distance_value(base,t_pos(i,:)); % ����i�����֮��ľ���
tw1 = TW_uavs(i,:); % ����i����Чʱ�䴰
for p = 1:tasks_num
    if p == i || TW_uavs(p,1)+TW_uavs(p,2) == 0 || Schedule(p,j) == 0 % ʱ�䴰Ϊ0�������޹۲����
        continue
    else
        tw2 = TW_uavs(p,:); % ����p����Чʱ�䴰
        distance1 = distance_value(t_pos(p,:),t_pos(i,:)); % ����p������i֮��ľ���
        time1 = distance1/V_UAV*3600; % ����p������i֮��ķ���ʱ��
        distance2 = distance_value(base,t_pos(p,:)); % ����p�����֮��ľ���
        distance = distance1+distance2+distance3;
        % �������жϣ��ض���ͻ��
        if distance > 2*D_UAV
            conflict = conflict + W(p);
        %  ʱ�������жϣ����ض���ͻ�жϣ�
        %  ��ִ��������p,�������i�Ŀ�ʼִ��ʱ���������p�����ʱ��
        elseif TW_uavs(i,1) >= TW_uavs(p,2)
            if TW_uavs(p,1)+time1+duration(p) > TW_uavs(i,2)
                conflict = conflict + W(p);
                % �����ܳ�ͻ�жϣ������ཻ������ʱ�䴰����������ͻ�����
                % �����ܳ�ͻ�жϣ����ཻ������ʱ�䴰����������ͻ�����
            elseif  TW_uavs(p,2)+time1+duration(p) > TW_uavs(i,1) &&...
                    TW_uavs(p,1)+time1+duration(p) <= TW_uavs(i,2)
                dur1 = duration(i);
                dur2 = duration(p)+time1;
                tw1(2) = tw1(2) - duration(i); % ����i������ִ��ʱ��
                tw2(2) = tw2(2) - duration(p); % ����p������ִ��ʱ��
                prob = Probability_conflict_type1(tw1,tw2,dur1,dur2);
                conflict = conflict + W(p)*prob;
            end
            %  ʱ�������жϣ����ض���ͻ�жϣ�����ִ��������i,�������p�Ŀ�ʼִ��ʱ���������i�����ʱ��
        elseif TW_uavs(p,1) >= TW_uavs(i,2)
            if TW_uavs(i,1)+time1+duration(i) > TW_uavs(p,2)
                conflict = conflict + W(p);
            % �����ܳ�ͻ�жϣ������ཻ������ʱ�䴰����������ͻ�����
            elseif  TW_uavs(i,2)+time1+duration(i) > TW_uavs(p,1) &&...
                    TW_uavs(i,1)+time1+duration(i) <= TW_uavs(p,2)
                dur1 = duration(i)+time1;
                dur2 = duration(p);
                tw1(2) = tw1(2) - duration(i); % ����i������ִ��ʱ��
                tw2(2) = tw2(2) - duration(p); % ����p������ִ��ʱ��
                prob = Probability_conflict_type1(tw1,tw2,dur1,dur2);
                conflict = conflict + W(p)*prob;
            end
        %  ���ܳ�ͻ�жϣ�������������Ľ���ʱ�䴰�����ͻ������
        else
           dur1 = duration(i)+time1; 
           dur2 = duration(p)+time1;
           tw1(2) = tw1(2) - duration(i); % ����i������ִ��ʱ��
           tw2(2) = tw2(2) - duration(p); % ����p������ִ��ʱ��
           prob = Probability_conflict_type2(tw1,tw2,dur1,dur2);
           conflict = conflict + W(p)*prob;
        end 
    end
end
end
%% �Ӻ���7�������������ʱ�䴰���ڳ���ʱ���׼��ʱ���ԭ������ܵ��³�ͻ�Ŀ����Լ���
% tw�����������Ч��ִ��ʱ�䴰�����ǳ���ʱ���Ӱ�졣
function prob = Probability_conflict_type1(tw1,tw2,dur1,dur2)
S0 = abs(tw1(2)-tw1(1))*abs(tw2(2)-tw2(1)); % ���ε����
delta1= tw1(2)-tw2(2);
delta2 = tw1(1)-tw2(1);
if delta1*delta2 > 0 
    if delta1 > 0
        T12 = tw1(2);
        T11 = tw1(1);
        T22 = tw2(2);
        T21 = tw2(1);
        d1 = dur1;
        d2 = dur2;
    else
        T12 = tw2(2);
        T11 = tw2(1);
        T22 = tw1(2);
        T21 = tw1(1);
        d1 = dur2;
        d2 = dur1;
    end
    if T22 + d2 <= T11 % Լ��ֱ���ھ����������ϲ�
        prob = 0;
    else % Լ��ֱ���ھ��������ཻ
        if T22 + d2 < T12 && T11 - d2 > T21 % Լ��ֱ�������ϱ߽��������
            S1 = 0.5*(T22+d2-T11)*(T22+d2-T11);
        elseif T22 + d2 < T12 && T11 - d2 <= T21 % Լ��ֱ�������±߽�������� 
            S1 = 0.5*(2*T22-T11+2*d2-T12)*(T12-T11);
        elseif T22 + d2 >= T12 % Լ��ֱ���ھ����������²࣬������������
            S1 = S0;
        else % Լ��ֱ�������±߽��������
            S1 = S0 - 0.5*(T12-d2-T21)*(T12-d2-T21);
        end
        prob = (S0-S1)/S0;
    end
end
end
%% �Ӻ���8���ڲ�����׼��ʱ��ͳ���ʱ�������£����������ʱ�䴰�������ͻ��
function prob = Probability_conflict_type2(tw1,tw2,dur1,dur2)
delta1= tw1(2)-tw2(2);
delta2 = tw1(1)-tw2(1);
S0 = abs(tw1(2)-tw1(1))*abs(tw2(2)-tw2(1)); % ���ε����
% ����������Ӱ���ֵ����
% ���С��0��˵����һ��ʱ�䴰����һ��ʱ�䴰��,��������ʱ�䴰�ص�һ����
if delta1*delta2 < 0
    if delta1 > 0 
        T12 = tw2(2);
        T11 = tw2(1);
        T22 = tw1(2);
        T21 = tw1(1);
        d1 = dur1;
        d2 = dur2;
    else
        T12 = tw1(2);
        T11 = tw1(1);
        T22 = tw2(2);
        T21 = tw2(1);
        d1 = dur2; 
        d2 = dur1;
    end
    if T21+d2 > T11 && T21+d2 < T12 % �γ������Σ��������߽�
        S2 = 0.5*(T12-T21-d2)*(T12-d2-T21);
    elseif T21+d2 <= T11
        S2 = 0.5*(T11-2*d2-2*T21+T12)*(T12-T11);
    elseif T21+d2 > T11 && T21+d2 >= T12 % �����߽�
        S2 = 0;
    end
    if T22-d1 < T12  && T11+d1 < T22 % �γ������Σ��������߽硣
        S1 = 0.5*(T22-T11-d1)*(T22-d1-T11);
    elseif T22-d1 >= T12  % �γ�ֱ������
        S1 = 0.5*(2*T22-T12-T11-2*d1)*(T12-T11);
    elseif T22-d1 < T12 && T11+d1 >= T22
        S1 = 0;
    end
else
    if delta1 > 0
        T12 = tw1(2);
        T11 = tw1(1);
        T22 = tw2(2);
        T21 = tw2(1);
        d1 = dur1;
        d2 = dur2;
    else
        T12 = tw2(2);
        T11 = tw2(1);
        T22 = tw1(2);
        T21 = tw1(1);
        d1 = dur2;
        d2 = dur1;
    end
    if T11 + d1 < T22
        S1 = 0.5*(T22-T11-d1)*(T22-d1-T11);
    else
        S1 = 0;
    end
    if T11-d2 > T21
        S2 = 0.5*(T12-T11)*(T22-2*T21+T11-d2);
    else
        if d2+T22 < T12
            S2 = 0.5*(2*T12-2*d2-T22-T21)*(T22-T21);
        elseif d2+T22 >= T12 && T21+d2 < T12
            S2 = 0.5*(T12-d2-T21)*(T12-d2-T21);
        elseif d2+T22 >= T12 && T21+d2 >= T12
            S2 = 0;
        end
    end
end
prob = (S0-S1-S2)/S0; % ������ʱ�䴰֮��ĳ�ͻ��������
end
%% �Ӻ���9�����㸽�ӵ���Ӧ�� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function fit = fit_addition(number,N_open_close)
if number > N_open_close
    fit = -20000000*(number-N_open_close);
else
    fit = 0;
end
end






