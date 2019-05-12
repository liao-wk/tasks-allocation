function funs = task_alloc_funs3
%UNTITLED4 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
funs.Opportunity_Sum = @Opportunity_Sum; % �������������������Դ�µĹ۲����
funs.Conflict_average = @Conflict_average; % ����ƽ����ͻ��
funs.Solution_generate = @Solution_generate; % ���ɳ�ʼ��
funs.Solution_generate_average = @Solution_generate_average; % ����ƽ����С��ͻ��ԭ�����ɳ�ʼ��
funs.Conflict_Caculate = @Conflict_Caculate; % �����������ѵ��ȵ�����ĳ�ͻ��
end
%% �Ӻ���1���������������������Դ�µĹ۲���� %%%%%%%%%%%%%%%%%%%%%%%%%
function tasks_opp = Opportunity_Sum(tasks_Opportunity)
tasks_num = size(tasks_Opportunity,1);
tasks_opp = zeros(tasks_num,1); % �۲�����ܺ�
for i = 1:tasks_num
    tasks_opp(i,1) = sum(tasks_Opportunity(i,:));
end
end
%% �Ӻ���2�����������ڲ�ͬ�Ĺ۲���Դ�µ�ƽ����ͻ��
function Conflict_ave = Conflict_average(resource_num,opp,Conflict)
tasks_num = size(opp,1);
Conflict_ave = zeros(tasks_num,resource_num);
% �����������д���0��ÿһ�е�Ԫ�ظ���
for j = 1:resource_num
    A = opp(opp(:,j) > 0); % ��������д���0��Ԫ������
    num = size(A,1)-1; % ��������������һ
    if num == -1 % �������Ƕ����е�����û��һ���۲����
        Conflict_ave(:,j) = inf;
    else
        Conflict_ave(:,j) = Conflict(:,j)/num; % ����Դr�µĸ��������ƽ����ͻ��
    end
end
end
%% �Ӻ���3�����ɳ�ʼ�� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Solu = Solution_generate(tasks,Conflict_tasks,R_remain,subplan)
% ������Ƚ��дӴ�С������
[Q,IX] = sort(tasks(:,9),'descend');
tasks2 = tasks(IX,:);
% ����ĳЩ������ܵĹ۲����Ϊ0,�����Ҫɾ������
% ����subplan����Ŀ
% ����subplan�������subplan��Ŀ
subplan_num = max(subplan(:,2));
r_num = size(subplan,1);
Solu = [];
tasks_num = size(tasks,1);
solu = zeros(subplan_num,tasks_num); % �������ľ�����ʽ
duration = tasks(:,7);
I = find(tasks(:,8)==0); % ������
nn = size(I,1);
if nn ~= 0
    disp('���������ܵĹ۲����Ϊ0');
    disp(I);
end
for i = 1:tasks_num
    k = tasks2(i,1); % ���������
    R_use = find(Conflict_tasks.average(k,:) < inf); % �ҳ�����Ŀ�����Դ,������
    C_Tab = Conflict_tasks.average(k,R_use); % �����ڿ�����Դ�µĳ�ͻ��
    RC = [R_use;C_Tab]; % ��ѡ��Դ�ı�źͳ�ͻ�ȵĺϳ�
    [A,index2] = sort(C_Tab);
    RC = RC(:,index2); % ����ͻ�Ƚ��д�С�������򣬵�1���Ƕ�Ӧ����Դ����2���Ƕ�Ӧ�ĳ�ͻ��
    p = 1; % ��RC���п�ѡ��Դ����������ţ�
    Re = RC(p,1); % ��ʼѡ�õĿ�ѡ��Դ���
    R_remain2 = R_remain;
    R_remain2(Re,1) = R_remain2(Re,1) - 1; % ʣ�࿪�ػ�������ȥ1
    R_remain2(Re,2) = R_remain2(Re,1) - duration(k); % ʣ��۲�ʱ���ȥ������ĳ���ʱ��
    while 1
        if R_remain2(Re,1) >= 0 &&  R_remain2(Re,2) >= 0
            for j = 1:r_num
                if subplan(j,1) == Re
                    suplan_code = subplan(j,2);
                    break
                end
            end
            solu(suplan_code,k) = 1; % �������񵽶�Ӧ����Դ��
            R_remain(Re,1) = R_remain(Re,1) - 1; % ʣ�࿪�ػ�������ȥ1
            R_remain(Re,2) = R_remain(Re,1) - duration(k); % ʣ��۲�ʱ���ȥ������ĳ���ʱ��
            break
        else
            Re = Rc(p,1);
            p = p+1;
        end
    end   
end
% TODO��SOLUת����������
% for i = 1:subplan_num
%     Solu = [Solu;solu(i,:)'];
% end
Solu = solu;
% TODO��ʼ����޸�����Ϊ���˻�������󺽳�Լ��������Ҫ�����н�����޸���
end
%% �Ӻ���4��������Сƽ����ͻ��ԭ��������񵽸��ӹ滮���� %%%%%%%%%%%%%%%%
function Solu = Solution_generate_average(tasks,Conflict_tasks,R_remain,subplan,sats,uavs,base1,base2,V_UAV,D_UAV)
% ������Ƚ��дӴ�С������
[Q,IX] = sort(tasks(:,9),'descend');
tasks2 = tasks(IX,:);
% ����ĳЩ������ܵĹ۲����Ϊ0,�����Ҫɾ������
% ����subplan����Ŀ
% ����subplan�������subplan��Ŀ
subplan_num = max(subplan(:,2));
r_num = size(subplan,1);
tasks_num = size(tasks,1);
solu = zeros(subplan_num,tasks_num); % �������ľ�����ʽ
duration = tasks(:,7);
I = find(tasks(:,8)==0); % ������
nn = size(I,1);
if nn ~= 0
    disp('���������ܵĹ۲����Ϊ0');
    disp(I);
end
% ÿ����һ������Ҫ����һ����һ�������ڸ�����Դ�µĳ�ͻ�ȣ��Դ˼��������ڸ��ӹ滮����
% ��ƽ����С��ͻ�ȣ�Ȼ����ƽ����С��ͻ�ȵ�ԭ���������
for i = 1:tasks_num
    clear RC;
    clear subplan_list;
    k = tasks2(i,1); % ���������
    R_use = find(Conflict_tasks.average(k,:) < inf); % �ҳ�����Ŀ�����Դ,������
    C_Tab1 = Conflict_tasks.average(k,R_use); % �����ڿ�����Դ�µĿ��ܳ�ͻ��
    C_Tab2 = Conflict_Caculate(subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV,solu,k); % �������ѵ��ȵ������ڿ�����Դ�µĳ�ͻ��
    C_Tab = 0.5*C_Tab1 + 0.5*C_Tab2; % ����ƽ����С��ͻ��%%%%%############%%
    RC(1:2,:) = [R_use;C_Tab]; % ��ѡ��Դ�ı�źͳ�ͻ�ȵĺϳ�
    RC_num = size(RC,2);
    % ��RCde����������һ�У�Ϊ��Ӧ���ӹ滮���ı��
    for p = 1:RC_num
        for q = 1:r_num
            if RC(1,p) == subplan(q,1)
                RC(3,p) = subplan(q,2);
                break
            end     
        end
    end
    
    % ������ӹ滮���ĵ�ƽ����ͻ��
    subplan_list2 = unique(RC(3,:)); % �ӹ滮�������ظ���������
    subplan_list_num = size(subplan_list2,2); % ���õ��ӹ滮��������
    subplan_list(1,:) = subplan_list2;
    subplan_list(2,:) = zeros(1,subplan_list_num);
    % ������ӹ滮���ĵ�ƽ����ͻ�ȵľ��岽��
    for m = 1:subplan_list_num
        num = 0;
        for n = 1:RC_num
            if RC(3,n) == subplan_list(1,m) &&R_remain(RC(1,n),1)>=1&&...
                    R_remain(RC(1,n),2)>=duration(k)
                subplan_list(2,m) = subplan_list(2,m)+RC(2,n);
                num = num + 1;
            end
        end
        if num ~= 0
            subplan_list(2,m) = subplan_list(2,m)/num;
        else
            subplan_list(2,m) = inf; % ����ӹ滮�����޾���ʣ����Դ�Ĺ۲���Դ
        end
    end
    
    % ��ƽ����ͻ�Ƚ��д�С�������򣬵�1���Ƕ�Ӧ���ӹ滮���ģ���2���Ƕ�Ӧ��ƽ����ͻ�ȡ�
    [A,index2] = sort(subplan_list(2,:));
    subplan_list = subplan_list(:,index2); % ͬ��ע��
    
    % ������k���䵽��p���ӹ滮���ģ����ӹ滮���ĵľ�����С��ͻ�ȵĿ�����Դ����ʣ����Դ
    p = 1;
    while p <= subplan_list_num
        p_sub = subplan_list(p,1);
        % ��p_sub���ӹ滮���ĵľ�����С��ͻ�ȵĿ�����Դ
        c = inf; % ��p���ӹ滮���ĵĿ�����Դ����С��ͻ��
        c_code = 0; % ��p���ӹ滮���ĵ���С��ͻ�ȵĿ�����Դ�ı��
        for j = 1:RC_num
            Re = RC(1,j);
            if RC(3,j) == p_sub && R_remain(Re,1)>=1 && R_remain(Re,2)>=duration(k)
                if RC(2,j) < c
                    c = RC(2,j);
                    c_code = j;
                end
            end
        end
        if c_code ~= 0
            RE = RC(1,c_code);
            R_remain(RE,1) = R_remain(RE,1) - 1;
            R_remain(RE,1) = R_remain(RE,1) - duration(k);
            solu(p_sub,k) = 1; % �������񵽶�Ӧ���ӹ滮������
            break
        else
            p = p+1;
        end
    end
    
    if p == subplan_list_num + 1 % �������ѡ�ӹ滮���ĵ���Դ���Ѿ��������
        p_sub = subplan_list(1,1);
        solu(p_sub,k) = inf; % �������񵽳�ͻ����С���ӹ滮�����ϣ��������
        disp('selected subplaners have been consumed!')
    end
    
end
Solu = solu;
end
%% �Ӻ���6�������������ѵ��ȵ�����ĳ�ͻ�� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Conflict = Conflict_Caculate(subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV,solu,k)
tasks_alloc_funs = task_alloc_funs1; % �������
tasks_alloc_funs2 = task_alloc_funs2; % �������
W = tasks(:,2);
r_num = size(subplan,1);
Conflict = zeros(r_num,1);
tasks_Opportunity = [sats.sats_opp,uavs.opp]; 
% ����soluȷ�������۲���Դ�ĵ������
tasks_num = size(solu,2);
Schedule = zeros(tasks_num,r_num);
subplan_num = size(solu,1);
for i = 1:subplan_num
    index_tasks = find(solu(i,:)==1); % ÿһ���ӹ滮�����ѵ�������ı��
    index_resouce = find(subplan(:,2)==i); % ÿһ���ӹ滮���ĵĹ۲���Դ�ı��
    index_resouce_num = size(index_resouce,1);
    for j = 1:index_resouce_num
        Schedule(index_tasks,index_resouce(j)) = 1;
        % �������ڸ���Դ��û�й۲���������ˢ��Ϊ0
        Schedule(index_tasks,tasks_Opportunity(index_tasks,:)==0) = 0;
    end
end
% ���ݸ����۲���Դ�ĵ����������Ԥ��������k�ĳ�ͻ��
for j = 1:r_num
    if subplan(j,2) == 1 || subplan(j,2) == 2 % ����۲���Դ������
        if ~isempty(sats.TW_tasks{k,j}) % �������i������j�ϵ�ʱ�䴰Ϊ��
            Conflict(j) = tasks_alloc_funs.Conflict_value_sats_schedule(sats,k,j,W,Schedule);
        else
            Conflict(j) = inf;
        end
    elseif  subplan(j,2) == 3 % ����۲���Դ�������˻�����1
        if uavs.opp1(k,1) ~= 0
            Conflict(j) = tasks_alloc_funs2.Conflict_Caculate_schedule...
                (uavs.TW_uavs1,tasks,base1,V_UAV,D_UAV,k,j,Schedule);
        else
            Conflict(j) = inf;
        end
    else % ����۲���Դ�������˻�����2
        if uavs.opp2(k,1) ~= 0
            Conflict(j) = tasks_alloc_funs2.Conflict_Caculate_schedule...
                (uavs.TW_uavs2,tasks,base2,V_UAV,D_UAV,k,j,Schedule);
        else
            Conflict(j) = inf;
        end
    end
end
% ɾ��Ԥ��������k�ڲ����ڹ۲����Ĺ۲���Դ�µĳ�ͻ��
Conflict(Conflict(:,1)==inf) = [];
Conflict = Conflict';
end






