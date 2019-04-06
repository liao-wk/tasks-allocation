function funs = task_alloc_funs
%UNTITLED6 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
funs.distance_value = @distance_value;
funs.time_windows = @time_windows; % ���㵥�������ڵ��������ϵ���Чʱ�䴰
funs.TW_tasks = @TW_tasks; % ��������ڸ��������ϵ���Чʱ�䴰
funs.Windows_Sats_opp = @Windows_Sats_opp; % ����Windows���������Ĺ۲�������
funs.Conflict_degree1 = @Conflict_degree1; % �������ǹ۲�������ĳ�ͻ�ȴ�С
funs.Conflict_degree2 = @Conflict_degree2; % �������˻��۲�������ĳ�ͻ�ȴ�С
funs.Judge_Dis_TW = @Judge_Dis_TW; % ���˻����������ʱ�䴰�ж�
funs.Probability_tasks = @Probability_tasks; % �������������ʱ�䴰�ĳ�ͻ����
funs.TW_tasks2 = @TW_tasks2; % ������������ڸ������˻��ϵ���Чʱ�䴰
funs.UAVs_opp = @UAVs_opp; % ������������ڸ������˻��ϵĹ۲����
end
%% �Ӻ���1�����ݾ�γ��������㵽���صľ���
function distance = distance_value(base,t_pos)
r = 6371;
distance = 2*r*asin(sqrt(sin(0.5*(base(1)-t_pos(1))).^2+cos(base(1))*...
    cos(t_pos(1))*sin(0.5*(base(2)-t_pos(2))).^2));
end
%% �Ӻ���2�����������ʱ�䴰��
function tw = time_windows(SE_tasks,SE_sats,i,j)
tw = [max([SE_tasks(i,1),SE_sats.start(i,j)]),min([SE_tasks(i,2),...
    SE_sats.end(i,j)])];
end
%% �Ӻ���3���������������ʱ�䴰����
function TW_tasks = TW_tasks(sats_opp,SE_sats,SE_tasks)
size_W = size(SE_sats.Windows); %Windows����Ĵ�С
sats_num = size_W(2)/16; %���ǵĸ���
tasks_num = size_W(1); % ����ĸ���
size_col = 2; % twʱ�䴰������,һ��ֻ�п�ʼʱ��ͽ���ʱ��
% ��TW_tasks����ֵ
TW_tasks = cell(tasks_num,sats_num);
% ���ǹ۲�Ļ������������ֵ,��Ϊtw�����ά��
size_max = max(max(sats_opp))+1;
% һ��������һ�����Ǹ��۲��¿����ж��ʱ�䴰�����Ҫ��ǰ��һ�������ռ�
% ����������������
for i = 1:tasks_num
    for p = 1:sats_num  % ���ǵı��
        q = 1; % �������Чʱ�䴰���������
        % ��ĳһ������ĳ�����ϵ�ʱ�䴰tw��һ�������ռ�
        tw = zeros(size_max,size_col);
        for j = 16*(p-1)+1:16*(p-1)+16
            % �����һ�����ǵĹ۲��£�16�У�������Windows�е�Ԫ�ش���0��
            %  ����i,jλ���ҵ���Чʱ�䴰
            if SE_sats.Windows(i,j) > 0
                tw(q, 1:end) = time_windows(SE_tasks,SE_sats,i,j);
                q = q+1;
            end
        end
        % ��ִ����һ�����ǵ����ڣ�16�У���ɾ��tw�����С�
        tw(q:end,:) = [];
        TW_tasks{i,p} = tw;
    end
end
end
%% �Ӻ���4�����������ĳ������Ĺ۲�ʱ��Ϊ��Чʱ�䣬��������
% �۲�Ľ���ʱ���ȥ��ʼʱ��
function [Windows,sats_opp] = Windows_Sats_opp(SE_sats,duration,SE_tasks)
Windows = SE_sats.end - SE_sats.start; % Ԥ�ȷ����ڴ�
size_t = size(SE_sats.start); %
tasks_num = size_t(1); % ����ĸ���
size_W = size(Windows); % ʱ�䴰�ھ���Ĵ�С
if tasks_num ~= size_W(1)
    disp("������©�˲�������Ĺ۲�������")
end
% �ж������Ƿ�����Чʱ����ڹ۲⵽����������ǣ�����Ӧ�Ĵ��ڳ���Ϊ0,
% ����ǣ������ǹ۲�Ľ���ʱ���ȥ��ʼʱ��
% ���ݸı�Ĵ��ھ���ȷ�������������µĹ۲�����ж��ٴ�
sats_opp = zeros(tasks_num,size_W(2)/16); % ����i�����ǹ۲�Ļ��ᣬ1��ʾ���Ա��۲⣬����Ϊ0
for i = 1:tasks_num  % ����ĸ���
    for j = 1:size_W(2)  % ȫ�����ǵĻ��ƴ�����1�������Ƶ�16��
        % ������ǵĹ۲�ʱ������Ч�۲�ʱ�䣬��������������Ϊ0
        if SE_sats.end(i,j) >= duration(i)+SE_tasks(i,1) &&...
                SE_sats.start(i,j) <= SE_tasks(i,2)-duration(i)
            Windows(i,j) = SE_sats.end(i,j) - SE_sats.start(i,j);
        else
            Windows(i,j) = 0;
        end
    end
    indexes = find(Windows(i,1:end) > 0);
    % һ��ʱ�䴰Ϊһ���۲����
    size_indexes = size(indexes);
    for k = 1:size_indexes(2)
        if sats_opp(i,floor(indexes(k)/16)+1) == 0
            sats_opp(i,floor(indexes(k)/16)+1) = 1;
        else
            sats_opp(i,floor(indexes(k)/16)+1) = sats_opp(i,floor(indexes(k)/16)+1)+1;
        end
    end
end
end
%% �Ӻ���5���������ǹ۲��³�ͻ�ȵĴ�С������������i��Ȼ��ͻ�Ϳ��ܳ�ͻ������q��¼����
%   ��ʱ�Ȳ�����Ȼ��ͻ��¼����
function Conflict = Conflict_degree1(i,j,SE_sats,W,tasks_num,duration)
m = size(SE_sats.TW_tasks{i,j},1); % ��������i������j����m��ʱ�䴰
Conflict = zeros(m,1);
for p = 1:m % p��������i������j�ϵĵ�p��ʱ�䴰
    tw_obj = SE_sats.TW_tasks{i,j}(p,:); % ��ʾ����i������j�ϵĵ�p��ʱ�䴰
    for q = 1:tasks_num
        prob = 0; % ��ʼ������i������q�ĳ�ͻ����
        % ����������񲻴���ʱ�䴰����������Ϊ����i
        if q == i || isempty(SE_sats.TW_tasks{q,j})
            continue
        else
            tw_tasks = SE_sats.TW_tasks{q,j}; % ʱ�䴰�����ж��
            k = size(tw_tasks,1); % �õ�����q��ʱ�䴰������
            for x = 1:k
                tw = tw_tasks(x,:); % �Ƚϵ�����ĵ�x��ʱ�䴰
                % ����i������q�ĳ�ͻ���ʼ��㣬���ó�ͻ���ʳ�������q�ļ�ֵ�õ���ͻ��
                if tw(2)-tw(1)+tw_obj(2)-tw_obj(1)-(max([tw(2),tw_obj(2)])-...
                        min([tw(1),tw_obj(1)])) > 0
                    prob = Probability_tasks(tw,tw_obj,duration,i,q);
                    %  ��ͻ���ۼ�
                    Conflict(p,1) = Conflict(p,1)+prob*W(q)/k;
                end
            end  
        end
    end
end
% ȡ����i������j��������Чʱ�䴰����С��ͻ��
Conflict = min(Conflict);
end
%% �Ӻ���6���������˻��۲�������ĳ�ͻ�ȴ�С
function Conflict = Conflict_degree2(tasks_num,UAVs_N,UAVs_opp,D_UAV,base,t_pos,V_UAV,SE_tasks,duration,W)
Conflict = zeros(tasks_num,UAVs_N);
% �������ͻ�ж�,��������㣬�������ʱ�䴰�ж�
for i = 1:tasks_num
    if UAVs_opp(i,1) ~= 0
        % ����i�����˻�j�ĳ�ͻ��
        Conflict(i,:) = Judge_Dis_TW(i,tasks_num,UAVs_opp,base,D_UAV,t_pos,V_UAV,SE_tasks,duration,W); 
    end
end
end
%% �Ӻ���7���������ʱ�䴰�ж�,SE_tasks��ʾ���������˻��µ���Чʱ�䴰
function conflict = Judge_Dis_TW(i,tasks_num,UAVs_opp,base,D_UAV,t_pos,V_UAV,SE_tasks,duration,W)
conflict = 0;
distance3 = distance_value(base,t_pos(i,:)); % ����i�����֮��ľ���
for p = 1:tasks_num
    if p == i || UAVs_opp(p,1) == 0
        continue
    else
        distance1 = distance_value(t_pos(p,:),t_pos(i,:)); % ����p������i֮��ľ���
        time1 = distance1/V_UAV*3600; % ����p������i֮��ķ���ʱ��
        distance2 = distance_value(base,t_pos(p,:)); % ����p�����֮��ľ���
        distance = distance1+distance2+distance3;
        % �������жϣ��ض���ͻ��
        if distance > 2*D_UAV
            conflict = conflict + W(p);
        %  ʱ�������жϣ����ض���ͻ�жϣ�
        %  ��ִ��������p,�������i�Ŀ�ʼִ��ʱ���������p�����ʱ��
        elseif SE_tasks(i,1) >= SE_tasks(p,2)
            if SE_tasks(p,1)+time1+duration(p) > SE_tasks(i,2)
                conflict = conflict + W(p);
            end
        %  ʱ�������жϣ����ض���ͻ�жϣ�����ִ��������i,�������p�Ŀ�ʼִ��ʱ���������i�����ʱ��
        elseif SE_tasks(p,1) >= SE_tasks(i,2)
            if SE_tasks(i,1)+time1+duration(i) > SE_tasks(p,2)
                conflict = conflict + W(p);
            end
        %  ���ܳ�ͻ�жϣ��������������ʱ�䴰
        else 
            prob = Probability_tasks(SE_tasks(i,:),SE_tasks(p,:),duration,i,p);
            conflict = conflict + W(p)*prob;
        end
    end
end
end
%% �Ӻ���8���������������ʱ�䴰��ͻ�ĸ���
function prob = Probability_tasks(tw,tw_obj,duration,i,q)
delta1= tw(2)-tw_obj(2);
delta2 = tw(1)-tw_obj(1);
S0 = abs(tw(2)-tw(1))*abs(tw_obj(2)-tw_obj(1)); % ���ε����
% ����������Ӱ���ֵ����
% ���С��0��˵����һ��ʱ�䴰����һ��ʱ�䴰��,��������ʱ�䴰�ص�һ����
if delta1*delta2 < 0
    if delta1 > 0 
        T12 = tw_obj(2);
        T11 = tw_obj(1);
        T22 = tw(2);
        T21 = tw(1);
        d1 = duration(i);
        d2 = duration(q);
    else
        T12 = tw(2);
        T11 = tw(1);
        T22 = tw_obj(2);
        T21 = tw_obj(1);
        d1 = duration(q); 
        d2 = duration(i);
    end
    if T21+d2 > T11
        S2 = 0.5*(T12-T21-d2)*(T12-d2-T21);
    else
        S2 = 0.5*(T11-2*d2-2*T21+T12)*(T12-T11);
    end
    if T22-d1 < T12
        S1 = 0.5*(T22-T11-d1)*(T22-d1-T11);
    else
        S1 = 0.5*(2*T22-T12-T11-2*d1)*(T12-T11);
    end
else
    if delta1 > 0
        T12 = tw(2);
        T11 = tw(1);
        T22 = tw_obj(2);
        T21 = tw_obj(1);
        d1 = duration(i);
        d2 = duration(q);
        
    else
        T12 = tw_obj(2);
        T11 = tw_obj(1);
        T22 = tw(2);
        T21 = tw(1);
        d1 = duration(q);
        d2 = duration(i);
    end
    S1 = 0.5*(T22-T11-d1)*(T22-d1-T11);
    if T11-d2 > T21
            S2 = 0.5*(T12-T11)*(T22-2*T21+T11-d2);
        else
            if d2+T22 < T12
                S2 = 0.5*(2*T12-2*d2-T22-T21)*(T22-T21);
            else
                S2 = 0.5*(T12-d2-T21)*(T12-d2-T21);
            end
    end
end
prob = (S0-S1-S2)/S0; % ������i������q֮��ĳ�ͻ��������
end
%% �Ӻ���9����������ڸ������˻��ϵ���Чʱ�䴰,ֻ�Ƚ�����Ŀ�ʼʱ������˻��ӻ��طɵ��������õ�ʱ��
function TW = TW_tasks2(SE_tasks,tasks_num,UAVs_num,base,V_UAV,t_pos)
TW = zeros(tasks_num,2);
for i = 1:tasks_num
    for j = 1:UAVs_num
        % �������������λ�õ�֮��ľ���
        task_position = t_pos(i,:);
        distance = distance_value(base,task_position);
        tw_start = distance/V_UAV*3600; % ת���ɵ�λs
        TW(i,1) = max([tw_start,SE_tasks(i,1)]);
        TW(i,2) = SE_tasks(i,2);
        % ������µ�ʱ�䴰�Ŀ�ʼʱ����ڽ���ʱ�䣬��ô��Ϊ0������
        if TW(i,1) > TW(i,2)
            TW(i,1) = 0;
            TW(i,2) = 0;
        end
    end
end
end
%% �Ӻ���10��������������ڸ������˻��ϵĹ۲����
function opp = UAVs_opp(tasks_num,UAVs_N,base,t_pos,TW_tasks,D_UAV)
opp = zeros(tasks_num,UAVs_N); % ���˻����ض�����Ĺ۲����
for i = 1:tasks_num
    dist_base = distance_value(base,t_pos(i,:));
    % ���Ѳ�������ж� �� ʱ�䴰�����ж�
    if dist_base < D_UAV && TW_tasks(i,2)-TW_tasks(i,1) ~= 0  
        opp(i,:) = 1;
    else
        opp(i,:) = 0;
    end
end
end



