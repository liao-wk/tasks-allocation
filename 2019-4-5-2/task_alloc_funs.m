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
funs.Conflict_degree1_2 = @Conflict_degree1_2; % �������ǹ۲�������ĳ�ͻ�ȴ�С
end
%% �Ӻ���1�����ݾ�γ��������㵽���صľ���
function distance = distance_value(base1,t_pos)
r = 6371;
distance = 2*r*asin(sqrt(sin(0.5*(base1(1)-t_pos(1))).^2+cos(base1(1))*...
    cos(t_pos(1))*sin(0.5*(base1(2)-t_pos(2))).^2));
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
%% �Ӻ���5���������ǹ۲��³�ͻ�ȵĴ�С
function Conflict = Conflict_degree1(i,j,SE_sats,W,tasks_num)
m = size(SE_sats.TW_tasks{i,j},1); % ��������i������j����m��ʱ�䴰
Conflict = zeros(m,1);
for p = 1:m % p��������i������j�ϵĵ�p��ʱ�䴰
    tw_obj = SE_sats.TW_tasks{i,j}(p,:); % ��ʾ����i������j�ϵĵ�p��ʱ�䴰
    TWValue_length = W(i)*(tw_obj(2)-tw_obj(1)); % ����ʱ�䴰�ĳ��ȳ��Զ�Ӧ�ļ�ֵ����Ϊ��ͻ�ȵķ�ĸ
    for q = 1:tasks_num
        cro_deg = 0; % ��ʼ������i������q�Ľ����Ϊ0
        % ����������񲻴���ʱ�䴰����������Ϊ����i
        if q == i || isempty(SE_sats.TW_tasks{q,j})
            continue
        else
            tw_tasks = SE_sats.TW_tasks{q,j}; % ʱ�䴰�����ж��
            k = size(tw_tasks,1); % �õ�����q��ʱ�䴰������
            for x = 1:k
                tw = tw_tasks(x,:); % �Ƚϵ�����ĵ�x��ʱ�䴰
                % ����ʱ�䴰�ĳ��ȳ��Զ�Ӧ�ļ�ֵ
                TWValue_length = TWValue_length+W(q)*(tw(2)-tw(1))/k;
                % �����cro_deg���㣬����н��棬������i��p��ʱ�䴰������q�Ľ���ȴ�С
                if tw(2)-tw(1)+tw_obj(2)-tw_obj(1)-(max([tw(2),tw_obj(2)])-...
                        min([tw(1),tw_obj(1)])) > 0
                    cro_deg = cro_deg+tw(2)-tw(1)+tw_obj(2)-tw_obj(1)-...
                        (max([tw(2),tw_obj(2)])-min([tw(1),tw_obj(1)])) ;
                end
            end
            % ������q�Ľ����ȡƽ��ֵ�������꽻��Ⱥ󣬼�������i������j�ϵĵ�p��ʱ�䴰�ĳ�ͻ�ȵķ�����
            Conflict(p,1) = Conflict(p,1)+cro_deg*W(q)/k;
        end
    end
    Conflict(p,1) = Conflict(p,1)/TWValue_length; % �����ͻ��
end
% ȡ����i������j�ϵ���С��ͻ��
Conflict = min(Conflict);
end
%% �Ӻ���6���������˻��۲�������ĳ�ͻ�ȴ�С
function Conflict = Conflict_degree2(tasks_num,UAVs_N,UAVs_opp,D_UAV,base,t_pos,V_UAV,SE_tasks,duration,W)
Conflict = zeros(tasks_num,UAVs_N(1));
% �������ͻ�ж�,��������㣬�������ʱ�䴰�ж�
for i = 1:tasks_num
    for j = 1:UAVs_N
        if UAVs_opp(i,j) ~= 0
        % ����i�����˻�j��ԭ��ͻ��
        Conflict(i,j) = Judge_Dis_TW(i,j,tasks_num,UAVs_opp,base,D_UAV,t_pos,V_UAV,SE_tasks,duration,W); 
        end
    end
end
for j = 1:UAVs_N
    max_Conflict = max(Conflict(:,j)); % �����˻�j�۲��µ����ԭ��ͻ��
    if max_Conflict ~= 0
        Conflict(:,j) = Conflict(:,j)/max_Conflict; % ���˻��۲��µĳ�ͻ��
    end
end
end
%% �Ӻ���7���������ʱ�䴰�ж�
function conflict = Judge_Dis_TW(i,j,tasks_num,UAVs_opp,base,D_UAV,t_pos,V_UAV,SE_tasks,duration,W)
conflict = 0;
distance3 = distance_value(base,t_pos(i,:)); % ����i�����֮��ľ���
for p = 1:tasks_num
    if p == i || UAVs_opp(p,j) == 0
        continue
    else
        distance1 = distance_value(t_pos(p,:),t_pos(i,:)); % ����p������i֮��ľ���
        time1 = distance1/V_UAV*3600; % ����p������i֮��ķ���ʱ��
        distance2 = distance_value(base,t_pos(p,:)); % ����p�����֮��ľ���
        distance = distance1+distance2+distance3;
        if distance > D_UAV
            conflict = conflict + W(p);
        elseif time1+SE_tasks(p,2)+duration(i) > SE_tasks(i,1)
            conflict = conflict + W(p);
        elseif time1+SE_tasks(1,1)+duration(i) > SE_tasks(p,2)
            conflict = conflict + W(p);
        end
    end
end
end
%% �Ӻ���8���������ǹ۲��³�ͻ�ȵĴ�С
function Conflict = Conflict_degree1_2(i,j,SE_sats,W,tasks_num)
m = size(SE_sats.TW_tasks{i,j},1); % ��������i������j����m��ʱ�䴰
Conflict = zeros(m,1);
for p = 1:m % p��������i������j�ϵĵ�p��ʱ�䴰
    tw_obj = SE_sats.TW_tasks{i,j}(p,:); % ��ʾ����i������j�ϵĵ�p��ʱ�䴰
    for q = 1:tasks_num
        % ����������񲻴���ʱ�䴰����������Ϊ����i
        if q == i || isempty(SE_sats.TW_tasks{q,j})
            continue
        else
            tw_tasks = SE_sats.TW_tasks{q,j}; % ʱ�䴰�����ж��
            k = size(tw_tasks,1); % �õ�����q��ʱ�䴰������
            for x = 1:k
                tw = tw_tasks(x,:); % �Ƚϵ�����ĵ�x��ʱ�䴰
                % ����ʱ�䴰�ĳ��ȳ��Զ�Ӧ�ļ�ֵ
                % �����cro_deg���㣬����н��棬������i��p��ʱ�䴰������q�Ľ���ȴ�С
                if tw(2)-tw(1)+tw_obj(2)-tw_obj(1)-(max([tw(2),tw_obj(2)])-...
                        min([tw(1),tw_obj(1)])) > 0
                    Conflict(p,1) = Conflict(p,1) + W(q)/k; % ||�н���ͼ��Ͻ����Ȩֵ||
                end
            end
        end
    end
end
% ȡ����i������j�ϵ���С��ͻ��
Conflict = min(Conflict);
end




