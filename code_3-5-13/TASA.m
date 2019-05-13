function   TASA(Solu)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
%% ����ģ���˻��㷨�Ż�����������ֵ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T = 1000; % ��ʼ�¶�Ϊ1000��
tasks_num = size(tasks,1);
r_num = size(subplan,1);
subplan_num= max(subplan(:,2));
Tabu1 = zeros(tasks_num,tasks_num); % ���ɱ�1
Tabu2 = zeros(subplan_num,tasks_num); % ���ɱ�2
K = 0.89; % �¶�˥������
L = 20; % ����Ʒ�������
TabuL = 3; % ���ɱ�ĳ���
Pro_Tabu2 = 0.5; % ���ɱ�2�Ľ��ɸ���
SO = zeros(L,tasks_num*sunplan_num); % �����⼯
% ���ӹ滮�����ѵ��ȷ�����δ���ȷ����������Ʊ��룩;��Ӧ����������ֵ
% T1_schedule,T1_unschedule;T2_schedule,T2_unschedule;
% T3_schedule,T3_unschedule;T4_schedule,T4_unschedule��

% ��ȡT_Schedule,subplan_num�У�tasks_num�У�Ԫ��Ϊ1��Ϊ����Ӧ��subplan�ѵ�������

% ��ȡT_unschedule,subplan_num�У�tasks_num�У�Ԫ��Ϊ1��Ϊ����Ӧ��subplanδ��������

best_so_far = Solu; % ��ǰ���Ž�
best_so_far_fit =  profit1 + profit2 + profit3 + profit4; % ��ǰ���Ž��Ӧ����������ֵ
tasks_alloc_funs4 = task_alloc_funs4; % �����ű���ֵ
R_subplan = zeros(subplan_num,2); % �ӹ滮���ĵ�ʣ����Դ����󿪹ػ������������۲�ʱ�䣩
best_value = best_so_far_fit;
best_solution = best_so_far;
while T > 0.01
    for p = 1:L
        SO = zeros(L,tasks_num);
        SO_FIT = zeros(L,1);
        SO_cross = zeros(L,2);
        %% ������ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ��ȡtasks_Opportunity_subplan���������ӹ滮�����ϵĹ۲����
        tasks_Opportunity_subplan = zeros(subplan_num,tasks_num);
        tasks_Opportunity = [sats.sats_opp,uavs.opp]; 
        for i = 1:tasks_num
            for j = 1:r_num
               if tasks_Opportunity(i,j) ~= 0
                   tasks_Opportunity_subplan(subplan(j),2) = ...
                       tasks_Opportunity_subplan(subplan(j),2)+tasks_Opportunity(i,j);
               end
            end
        end
        
        [solu,task1,task2] = tasks_alloc_funs4.Neighborhood_structure_cross(T_schedule,tasks_Opportunity_subplan);
        % �ѵ��ȵ������ڸ��ӹ滮���������ĵ���Դ
        for j = 1:subplan_num
            for i = 1:tasks_num
                R_subplan(j,1) = R_subplan(j,1)-solu(j,i); % ���ػ�����
                R_subplan(j,2) = R_subplan(j,2)-solu(j,i)*tasks(i,7); % �����۲�ʱ��
            end
        end
        
        %% ��δ���ȵ���������С��ͻ��(���ѵ�������)��ԭ����뵽��Ӧ���ӹ滮����
        solu = tasks_alloc_funs4.Neighborhood_structure_insertion(solu,T_unschedule,R_subplan,...
                Conflict_tasks,subplan,sats,uavs,tasks,base1,base2,V_UAV,D_UAV);
        
        %% ����solu�õ�SOLU %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        SOLU = zeros(subplan_num*tasks_num,1);
        for j = 1:subplan_num
            SOLU((j-1)*tasks_num:j*tasks_num,1) = solu(j,:);
        end
        
        SO(P,:) = SOLU;
        SO_FIT(P) = FUN(SOLU);
        SO_cross(p,:) = [task1,task2];
    end
    %% ����׼���ж�&&���½��ɱ� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ����Ӧ�ȴӴ�С����
    [SO_FIT,index] = sort(SO_FIT,'descend');
    SO = SO(index,:);
    SO_cross = SO_cross(index,:);
    %                  ����׼��                           %
    if SO_FIT(1) > best_so_far_fit
        best_so_far_fit = SO_FIT(1);
        best_so_far = SO(1);
        %                   ���½��ɱ�                       %     
        [Tabu1,Tabu2] = tasks_alloc_funs4.Re_Tabu(Tabu1,Tabu2,TabuL,1);
    else
        for i = 1:L
            if Tabu1(task1,task2)==0 && probabilty_Tabu2 < Pro_Tabu2
                best_so_far_fit = SO_FIT(i);
                best_so_far = SO(i);
                [Tabu1,Tabu2] = tasks_alloc_funs4.Re_Tabu(Tabu1,Tabu2,TabuL,i);
                break
            end
        end
    end
    
    %% �˻���ԣ����ܸ��ý⣬��һ�����ʽ����ӽ� %%%%%%%%%%%%%%%%%%%%%%%%
    if best_so_far_fit < best_value 
        if exp(-1*abs(best_so_far_fit-best_so_far_fit_old)) > rand
            best_value = best_so_far_fit;
            best_solution = best_so_far;
            T = K*T;
        end
    else
        best_value = best_so_far_fit;
        best_solution = best_so_far;
        T = K*T;
    end 
end
end

%% ����ͼ�� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
plot(best_value)
hold on;
xlabel("x");
ylabel("y");
title(["�Ż�����ֵ��",num2str(best_value)]);
end
