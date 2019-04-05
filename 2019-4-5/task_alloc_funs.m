function funs = task_alloc_funs
%UNTITLED6 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
funs.distance_value = @distance_value;
funs.time_windows = @time_windows; % ���㵥�������ڵ��������ϵ���Чʱ�䴰
funs.TW_tasks = @TW_tasks; % ��������ڸ��������ϵ���Чʱ�䴰
funs.Windows_Sats_opp = @Windows_Sats_opp; % 
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







