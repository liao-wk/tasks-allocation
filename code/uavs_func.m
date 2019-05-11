function [profit_value,T_schedule,T_unschedule] = uavs_func(solu,tasks,base,UAVs_N,D_UAV,V_UAV,Max_Energy,Max_tasks,dur)
%% ��һ�������Ƚ���������䣬�ٽ��к����滮
% ȥ���ϰ���
tasks_num = size(solu,2);
% �����񼯵ı�Ž������±�ţ���1��ʼ�������Ҫ��һ����������ʾ�µı�Ŷ�Ӧ�ľɱ��
% ��һ�����±�ţ��ڶ����Ǿɱ��

Tasks = zeros(tasks_num,2);
p = 1;
for j = 1:tasks_num
    if solu(j) ~= 0
        Tasks(p,2) = j;
        p = p+1;
    end
end
Tasks(p:end,:) = [];
duration = dur(Tasks(:,2),:);
%% ���˻��������Ż��㷨
data=tasks(Tasks(:,2),3:4); % Ŀ��������
data(end+1,:) = base; % ���ϻ��صľ�γ������
Dot_Num=size(data,1);
tw=tasks(Tasks(:,2),5:6); % ����ʱ�䴰
interval=tw(:,2)-tw(:,1); % ʱ�䴰�ĳ���
profit=tasks(Tasks(:,2),2); % Ŀ��������ֵ
k=UAVs_N; %���˻���
Max_Voyage=2*D_UAV*ones(1,k); %���˻���������
Speed=V_UAV*ones(1,k); % ���˻����ٶ�
Max_tasks = Max_tasks*ones(1,k); % ��󿪹ػ�����Լ��
Max_Energy = Max_Energy*ones(1,k); % �������Լ��




middle_tw = zeros(Dot_Num-1,1);
%ʱ�䴰���м�ֵ
for i=1:Dot_Num-1
    middle_tw(i,1)=ceil((tw(i,1)+tw(i,2))/2);
end
x=data(1:Dot_Num-1,1);
y=data(1:Dot_Num-1,2);
sequence=randperm(Dot_Num-1);
sequence=sort(sequence);

 %% ������������֮�����̾��룬���ݾ�γ�����깫ʽ
 r = 6371;
 distance_value = @(t_pos1,t_pos2) 2*r*asin(sqrt(sin(0.5*(t_pos1(1)-...
     t_pos2(1))).^2+cos(t_pos1(1))*cos(t_pos2(1))*sin(0.5*(t_pos1(2)-t_pos2(2))).^2));
 Dis_Refer=zeros(Dot_Num,Dot_Num); % �����֮��ľ���
for i = 1:Dot_Num
    t_pos1 = data(i,:);
    for j = 1:Dot_Num
        t_pos2 = data(j,:);
        if i == j
            Dis_Refer(i,j) = nan;
        else
            Dis_Refer(i,j) = distance_value(t_pos1,t_pos2);
        end
    end
end
%% ������䣨����+�Ƕȵ�����
[resX1,resY1,record,sequence,alter_sequence] = FunK_mean( x,y,k,sequence );
[resX2,resY2,newsequence]=Angle_adjustment(x,y,k,data(size(data,1),1),data(size(data,1),2),sequence,alter_sequence,resX1,resY1,record);

%% ��ʼ������ȣ��������ָߵ�ѡ��
%ָ���е������ľ��롢ʱ�䴰�ĳ��ȡ������̶ȡ�λ�����ơ�����ֵ
%% �������,�����Ż�
T=100;%��ʼ�¶�
K=0.95;%˥������
L=50;%����Ʒ�������
tabul=k-1;%���ɳ���
%ָ���е������ľ��롢ʱ�䴰�ĳ��ȡ������̶ȡ�λ�����ơ�����ֵ
[total_reward,Dispatch,mission_undispatch]=FunDispatch(newsequence,Dis_Refer,interval,profit,tw,middle_tw,Dot_Num,k,Speed,Max_Voyage,Max_Energy,Max_tasks,duration);
 %����δ��ɵ�����
 mission_bedistributed=nonzeros(mission_undispatch)';
 numUAV=sum(Dispatch~=0,2);
 ptotal_reward(1)=total_reward;
 gtotal_reward=ptotal_reward(1);
 best_reward=ptotal_reward(1);
 best_Dispatch=Dispatch;
 best_unmission=mission_undispatch;
 gDispatch=Dispatch;
 Loop_Time=1;%��������
 best = []; % ������ʷ���Ž�
 best_value =total_reward; % ��ʷ���Ž�
while T>0.001
 %% ��δ��ɵ��������·��䣬���е����Ż�

 Tabu=zeros(k,Dot_Num-1);
 for i=1:L%��ͬһ�¶��¶��Ѱ�Ҿֲ����Ž�
     
     %�������δ��ɵ�����
     iterationsequence=gDispatch;
     %ÿʮ�ν����˻���ִ�������������
     numUAV=sum(gDispatch~=0,2);
     if mod(Loop_Time,10)==0
         UAV1=ceil(k*rand());
         UAV2=ceil(k*rand());
         while UAV1==UAV2
             UAV1=ceil(k*rand());
             UAV2=ceil(k*rand());
         end
         L1=ceil(numUAV(UAV1)*rand());
         L2=ceil(numUAV(UAV2)*rand());
         tmp= iterationsequence(UAV1,L1);
         iterationsequence(UAV1,L1)= iterationsequence(UAV2,L2);
         iterationsequence(UAV2,L2)=tmp;
     end
     numuav=sum(mission_undispatch~=0,2);
     Num_unmission=size(mission_bedistributed,2);
     UAV_unmission=ceil(rand(1,Num_unmission)*k);
     if Num_unmission==0
         break
     end
     
     %���½��ɱ�
     for m=1:k
         for n=1:numuav(m)
             Tabu(m,mission_undispatch(m,n))=tabul;
         end
     end
     for j=1:Num_unmission
         while Tabu(UAV_unmission(j),mission_bedistributed(j))~=0
             UAV_unmission(j)=ceil(rand()*k);
         end
     end
     for a=1:Num_unmission
         iterationsequence(UAV_unmission(a),numUAV(UAV_unmission(a))+1)=mission_bedistributed(a);
         numUAV=sum(iterationsequence~=0,2);
     end
     %���·���
     [total_reward1,Dispatch,mission_undispatch1]=FunDispatch(iterationsequence,Dis_Refer,interval,profit,tw,middle_tw,Dot_Num,k,Speed,Max_Voyage,Max_Energy,Max_tasks,duration);
     %����ȫ�����Ž�
     if total_reward1>best_reward
         best_reward=total_reward1;
         best_Dispatch=Dispatch;
         best_unmission=mission_undispatch1;
     end
     %����ֵ�����ڵ�ǰ���Ž⣬���
     if total_reward1>gtotal_reward
         gtotal_reward=total_reward1;
         gDispatch=Dispatch;
         %����δ��ɵ�����
         mission_undispatch= mission_undispatch1;
         mission_bedistributed=nonzeros(mission_undispatch1)';
     else
         %�Ը���ѡ���Ƿ�����½�
         if exp((total_reward1-gtotal_reward)/T)>rand()
             gDispatch=Dispatch;
             gtotal_reward=total_reward1;
             mission_undispatch=mission_undispatch1;
             mission_bedistributed=nonzeros(mission_undispatch1)';
         end
     end
     Loop_Time=Loop_Time+1;
     ptotal_reward(Loop_Time)=gtotal_reward;
     if gtotal_reward > best_value
         best_value = gtotal_reward;
     end
     best = [best,best_value];
     % ���½��ɱ�
     for m=1:k
         for n=1:Dot_Num-1
             if Tabu(m,n)~=0
                 Tabu(m,n)=Tabu(m,n)-1;
             end
         end
     end
 end
T=T*K;
end
profit_value = best(end); % ��ʷ����ֵ
% ����mission_bedistributed�õ�T_schedule
T_schedule = zeros(1,tasks_num);
mission_bedistributed_num = size(mission_bedistributed,2);
for i = 1:mission_bedistributed_num
    task_code = Tasks(mission_bedistributed(i),2);
    T_schedule(task_code) = 1;
end
T_total = zeros(1,tasks_num);
% ����T_schedule��T_total�ó�T_unschedule
T_total(Tasks(:,2)') = 1;
T_unschedule = T_total-T_schedule;

