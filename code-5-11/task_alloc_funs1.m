function funs = task_alloc_funs1
%UNTITLED2 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
funs.Windows_Satsopp = @Windows_Satsopp; % ��ȡ�����������µ�ʱ�䴰�͹۲����
funs.opp_devide_weight = @opp_devide_weight; % ��ȡ�����ڹ۲���Դ�µ������
funs.TW_tasks_sats = @TW_tasks_sats; % ��ȡ�����������µ���Чʱ�䴰
funs.Conflict_value_sats = @Conflict_value_sats; % ��ȡ��������Դ�µĳ�ͻ�ȴ�С
funs.Conflict_value_sats_schedule = @Conflict_value_sats_schedule; % ��ȡ��������Դ�µ����ѵ��������ͻ�ȴ�С
funs.Window_sats = @Window_sats; % ���������ڸ�����ϵ����翪ʼʱ�䡢����ʼʱ��
end
%% �Ӻ���1����ȡ�����������µ�ʱ�䴰���Ⱥ͹۲���� %%%%%%%%%%%%%%%%%%%%%%%%
function [Windows,sats_opp] = Windows_Satsopp(sats,duration,SE_tasks)
Windows = sats.end - sats.start; % Ԥ�ȷ����ڴ�
pathway_num = size(Windows,2); % ���ǵĹ������
sats_num = pathway_num/16; % ���ǵĸ���
tasks_num = size(Windows,1); % ����ĸ���
sats_opp = zeros(tasks_num,sats_num); % ����i�����ǹ۲�Ļ��ᣬ1��ʾ���Ա��۲⣬����Ϊ0
for i = 1:tasks_num  % ����ĸ���
    for j = 1:pathway_num  % ȫ�����ǹ��deȦ��
        % ������ǵĹ۲�ʱ������Ч�۲�ʱ�䣬��������������Ϊ0
        if sats.end(i,j) >= duration(i)+SE_tasks(i,1) &&...
                sats.start(i,j) <= SE_tasks(i,2)-duration(i)
            Windows(i,j) = sats.end(i,j) - sats.start(i,j);
        else % �����������Ч�۲������
            Windows(i,j) = 0;
        end
    end
    indexes = find(Windows(i,1:end) > 0);
    % һ��ʱ�䴰Ϊһ���۲����
    opp_num = size(indexes,2);
    for k = 1:opp_num
        if sats_opp(i,floor(indexes(k)/16)+1) == 0
            sats_opp(i,floor(indexes(k)/16)+1) = 1;
        else
            sats_opp(i,floor(indexes(k)/16)+1) = sats_opp(i,floor(indexes(k)/16)+1)+1;
        end
    end
end
end
%% �Ӻ���2����ȡ�����ڹ۲���Դ�µ������ %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function tasks_value = opp_devide_weight(resource_num,opp,W)
tasks_num = size(W,1);
tasks_value = zeros(tasks_num,resource_num);
for j = 1:resource_num
    for i = 1:tasks_num
        if opp(i,j) ~= 0
            tasks_value(i,j) = W(i)/opp(i,j); % ����i��Ȩ�س�������Դj�µĻ���
        else
            tasks_value(i,j) = 0; % �����������Դ��û�й۲���ᣬ��ô�����Ϊ0.
        end
    end
end
end
%% �Ӻ���3��ȡ�����������µ���Чʱ�䴰����Ԫ�����ʽ�洢 %%%%%%%%%%%%%%%%%%%%%%
function TW_tasks = TW_tasks_sats(sats,SE_tasks)
% ����һ������,��������ʱ�䴰ȷ����Чʱ�䴰
f_tw = @(tw1,tw2) [max([tw1(1),tw2(1)]),min([tw1(2),tw2(2)])]; 

sats_num = size(sats.Windows,2)/16; 
tasks_num = size(sats.Windows,1); 
m = 2; % ʱ�䴰����������
TW_tasks = cell(tasks_num,sats_num); % ��TW_tasks����ֵ
k = max(max(sats.sats_opp))+1; % ���ǹ۲�������������Ϊʱ�䴰������ά��������ֵ�ô�
% һ��������һ�����Ǹ��۲��¿����ж��ʱ�䴰�����Ҫ��ǰ��һ�������ռ�
for i = 1:tasks_num
    for p = 1:sats_num  % ���ǵı��
        q = 1; % �������Чʱ�䴰���������
        % ��ĳһ������ĳ�����ϵ�ʱ�䴰tw��һ�������ռ�
        tw = zeros(m,k);
        for j = 16*(p-1)+1:16*(p-1)+16
            % �����һ�����ǵĹ۲��£�16�У�������Windows�е�Ԫ�ش���0��
            %  ����i,jλ���ҵ���Чʱ�䴰
            if sats.Windows(i,j) > 0
                % �Ȼ������涨���ʱ�䣬�ٻ�����ǹ۲��������ʱ�䴰��
                tw1 = SE_tasks(i,:);
                tw2 = [sats.start(i,j),sats.end(i,j)];
                tw(q, 1:end) = f_tw(tw1,tw2);
                q = q+1;
            end
        end
        % ��ִ����һ�����ǵ����ڣ�16�У���ɾ��tw�����С�
        tw(q:end,:) = [];
        TW_tasks{i,p} = tw;
    end
end
end
%% �Ӻ���4����ȡ��������Դ�µĳ�ͻ�ȴ�С %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Conflict = Conflict_value_sats(sats,i,j,W)
tasks_num = size(sats.TW_tasks,1);
m = size(sats.TW_tasks{i,j},1); % ��������i������j����m��ʱ�䴰
Conflict = zeros(m,1);
for p = 1:m % p��������i������j�ϵĵ�p��ʱ�䴰
    tw1 = sats.TW_tasks{i,j}(p,:); % ��ʾ����i������j�ϵĵ�p��ʱ�䴰
    for q = 1:tasks_num
        % ����������񲻴���ʱ�䴰����������Ϊ����i
        if q == i || isempty(sats.TW_tasks{q,j})
            continue
        else
            tw_tasks = sats.TW_tasks{q,j}; % ʱ�䴰�����ж��
            k = size(tw_tasks,1); % �õ�����q��ʱ�䴰������
            for x = 1:k
                tw2 = tw_tasks(x,:); % �Ƚϵ�����ĵ�x��ʱ�䴰
                % ��Чʱ�䴰�����ж�
                if tw2(2)-tw2(1)+tw1(2)-tw1(1)-(max([tw2(2),tw1(2)])-...
                        min([tw2(1),tw1(1)])) > 0
                    Conflict(p,1) = Conflict(p,1) + W(q)/k; %  ��ͻ���ۼ�
                end
            end
        end  
    end
end
end
%% �Ӻ���5����ȡ�����ڹ۲���Դ�����ѵ�������ĳ�ͻ�ȴ�С %%%%%%%%%%%%%%%%
function Conflict = Conflict_value_sats_schedule(sats,i,j,W,Schedule)
tasks_num = size(sats.TW_tasks,1);
m = size(sats.TW_tasks{i,j},1); % ��������i������j����m��ʱ�䴰
Conflict = zeros(m,1);
for p = 1:m % p��������i������j�ϵĵ�p��ʱ�䴰
    tw1 = sats.TW_tasks{i,j}(p,:); % ��ʾ����i������j�ϵĵ�p��ʱ�䴰
    for q = 1:tasks_num
        % ����������񲻴���ʱ�䴰����������Ϊ����i������Դj�²����ѵ��ȵ�����
        if q == i || isempty(sats.TW_tasks{q,j}) || Schedule(q,j) == 0
            continue
        else
            tw_tasks = sats.TW_tasks{q,j}; % ʱ�䴰�����ж��
            k = size(tw_tasks,1); % �õ�����q��ʱ�䴰������
            for x = 1:k
                tw2 = tw_tasks(x,:); % �Ƚϵ�����ĵ�x��ʱ�䴰
                % ��Чʱ�䴰�����ж�
                if tw2(2)-tw2(1)+tw1(2)-tw1(1)-(max([tw2(2),tw1(2)])-...
                        min([tw2(1),tw1(1)])) > 0
                    Conflict(p,1) = Conflict(p,1) + W(q)/k; %  ��ͻ���ۼ�
                end
            end
        end  
    end
end
end
%% �Ӻ���6�����������ڸ�����ϵ����翪ʼʱ�䡢����ʼʱ��
function [Stime,Etime] = Window_sats(sats,SE_tasks,duration)
tasks_num = size(SE_tasks,1);
pathways_num = size(sats.start,2); % �������
Stime = zeros(tasks_num,pathways_num);
Etime = zeros(tasks_num,pathways_num);
for i = 1:tasks_num
    for j = 1:pathways_num
        if sats.end(i,j) >= duration(i)+SE_tasks(i,1) &&...
                sats.start(i,j) <= SE_tasks(i,2)-duration(i)
            Stime(i,j) = max([sats.start(i,j),SE_tasks(i,1)]);
            Etime(i,j) = min([sats.end(i,j),SE_tasks(i,2)]);
        end
    end
end

end


