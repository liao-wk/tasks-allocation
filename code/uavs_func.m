function [profit_value,T_schedule,T_unschedule] = uavs_func(solu,tasks,base,UAVs_N,D_UAV,V_UAV,Max_Energy,Max_tasks,dur)
%% 对一个机场先进行任务分配，再进行航迹规划
% 去掉障碍物
tasks_num = size(solu,2);
% 将任务集的编号进行重新编号，从1开始，因此需要设一个矩阵来表示新的编号对应的旧编号
% 第一列是新编号，第二列是旧编号

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
%% 无人机智能体优化算法
data=tasks(Tasks(:,2),3:4); % 目标点的坐标
data(end+1,:) = base; % 加上基地的经纬度坐标
Dot_Num=size(data,1);
tw=tasks(Tasks(:,2),5:6); % 生成时间窗
interval=tw(:,2)-tw(:,1); % 时间窗的长度
profit=tasks(Tasks(:,2),2); % 目标点的收益值
k=UAVs_N; %无人机数
Max_Voyage=2*D_UAV*ones(1,k); %无人机航程限制
Speed=V_UAV*ones(1,k); % 无人机的速度
Max_tasks = Max_tasks*ones(1,k); % 最大开关机次数约束
Max_Energy = Max_Energy*ones(1,k); % 最大能量约束




middle_tw = zeros(Dot_Num-1,1);
%时间窗的中间值
for i=1:Dot_Num-1
    middle_tw(i,1)=ceil((tw(i,1)+tw(i,2))/2);
end
x=data(1:Dot_Num-1,1);
y=data(1:Dot_Num-1,2);
sequence=randperm(Dot_Num-1);
sequence=sort(sequence);

 %% 计算任意两点之间的最短距离，根据经纬度坐标公式
 r = 6371;
 distance_value = @(t_pos1,t_pos2) 2*r*asin(sqrt(sin(0.5*(t_pos1(1)-...
     t_pos2(1))).^2+cos(t_pos1(1))*cos(t_pos2(1))*sin(0.5*(t_pos1(2)-t_pos2(2))).^2));
 Dis_Refer=zeros(Dot_Num,Dot_Num); % 点与点之间的距离
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
%% 任务分配（聚类+角度调整）
[resX1,resY1,record,sequence,alter_sequence] = FunK_mean( x,y,k,sequence );
[resX2,resY2,newsequence]=Angle_adjustment(x,y,k,data(size(data,1),1),data(size(data,1),2),sequence,alter_sequence,resX1,resY1,record);

%% 初始任务调度（根据评分高低选择）
%指标有到机场的距离、时间窗的长度、紧急程度、位置优势、收益值
%% 任务调度,迭代优化
T=100;%初始温度
K=0.95;%衰减参数
L=50;%马尔科夫链长度
tabul=k-1;%禁忌长度
%指标有到机场的距离、时间窗的长度、紧急程度、位置优势、收益值
[total_reward,Dispatch,mission_undispatch]=FunDispatch(newsequence,Dis_Refer,interval,profit,tw,middle_tw,Dot_Num,k,Speed,Max_Voyage,Max_Energy,Max_tasks,duration);
 %所有未完成的任务
 mission_bedistributed=nonzeros(mission_undispatch)';
 numUAV=sum(Dispatch~=0,2);
 ptotal_reward(1)=total_reward;
 gtotal_reward=ptotal_reward(1);
 best_reward=ptotal_reward(1);
 best_Dispatch=Dispatch;
 best_unmission=mission_undispatch;
 gDispatch=Dispatch;
 Loop_Time=1;%迭代次数
 best = []; % 储存历史最优解
 best_value =total_reward; % 历史最优解
while T>0.001
 %% 将未完成的任务重新分配，进行迭代优化

 Tabu=zeros(k,Dot_Num-1);
 for i=1:L%在同一温度下多次寻找局部最优解
     
     %随机分配未完成的任务
     iterationsequence=gDispatch;
     %每十次将无人机可执行任务随机交换
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
     
     %更新禁忌表
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
     %重新分配
     [total_reward1,Dispatch,mission_undispatch1]=FunDispatch(iterationsequence,Dis_Refer,interval,profit,tw,middle_tw,Dot_Num,k,Speed,Max_Voyage,Max_Energy,Max_tasks,duration);
     %保留全局最优解
     if total_reward1>best_reward
         best_reward=total_reward1;
         best_Dispatch=Dispatch;
         best_unmission=mission_undispatch1;
     end
     %收益值若大于当前最优解，替代
     if total_reward1>gtotal_reward
         gtotal_reward=total_reward1;
         gDispatch=Dispatch;
         %所有未完成的任务
         mission_undispatch= mission_undispatch1;
         mission_bedistributed=nonzeros(mission_undispatch1)';
     else
         %以概率选择是否接受新解
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
     % 更新禁忌表
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
profit_value = best(end); % 历史最优值
% 根据mission_bedistributed得到T_schedule
T_schedule = zeros(1,tasks_num);
mission_bedistributed_num = size(mission_bedistributed,2);
for i = 1:mission_bedistributed_num
    task_code = Tasks(mission_bedistributed(i),2);
    T_schedule(task_code) = 1;
end
T_total = zeros(1,tasks_num);
% 根据T_schedule和T_total得出T_unschedule
T_total(Tasks(:,2)') = 1;
T_unschedule = T_total-T_schedule;

