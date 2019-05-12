function [profit,T_schedule,T_unschedule] = sats_func(solu,sats,tasks,subplan,Energy_C,Volum_C,agent_code)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
% Volum_C = 3000; % 观测时间限制
% Energy_C = 140; % 最大开关机次数限制
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
%% 求解卫星的智能体算法 
N=size(Tasks,1);% 目标个数
pathway_num = 16; % 单颗卫星的轨道数量
% 该智能体包含的卫星编号
index_res = subplan(subplan(:,2)==agent_code,1);
index_res_num = size(index_res,1);
Stime = zeros(N,pathway_num*index_res_num);
Etime = zeros(N,pathway_num*index_res_num);
window = zeros(N,pathway_num*index_res_num);
for qq = 1:index_res_num
    Stime(:,((qq-1)*pathway_num+1:qq*pathway_num)')=sats.Stime(Tasks(:,2),((index_res(qq)-1)*pathway_num+1:index_res(qq)*pathway_num)');
    Etime(:,((qq-1)*pathway_num+1:qq*pathway_num)')=sats.Etime(Tasks(:,2),((index_res(qq)-1)*pathway_num+1:index_res(qq)*pathway_num)');
    window(:,((qq-1)*pathway_num+1:qq*pathway_num)')=sats.Windows(Tasks(:,2),((index_res(qq)-1)*pathway_num+1:index_res(qq)*pathway_num)');
end

pathways_num = size(Stime,2);
duration = tasks(Tasks(:,2),7);
weight = tasks(Tasks(:,2),2);
G=100; % 初始温度
l=zeros(pathways_num,1);
NC=1;
T=1:N;
F=zeros(1,N); % 罚函数
Rho=0.1; % 信息素蒸发系数
Alpha=1;
Beta=5;
DG=0.99;% 衰减参数
Tau=ones(N,pathways_num);% 信息素矩阵
Pbest=[];
%%%%%%%%%%%%%%%%找出不能够观测的任务%%%%%%%%%%%%%%%%%%%
for i=1:N
    if isempty(find(window(i,:),1))
        T(T==i)=[];% 将不能被观测到的任务删除
    end
end
O=zeros(pathways_num,N);% 轨道任务矩阵
Obest=zeros(pathways_num,N);% 最优方案
O1=zeros(pathways_num,N);

while G> 0.01
    D=T;% 记录上次待分配的任务
    O2=O1;% 记录上次的分配方案
    Profit=0;% 更新本次迭代的收益值
    while ~isempty(T)
              P1=zeros(1,length(T));% 任务T集合的权重
              for i=1:length(T)
                 P1(i)=weight(T(i))-F(T(i));
              end
              t1=find(P1==max(P1));
              t=T(t1(1));% 找到收益值最大的任务t
              %找到能观测到t任务的轨道集合ind1,ind1(k)代表某一具体轨道
              ind1=find(window(t,:));
              if length(ind1)==1
                 fo=ind1(1);
                 l=length(find(O(fo,:)));
                 O(fo,l+1)=t;
              end
              if length(ind1)>1
              %构造轨道评分矩阵，第一行：任务号；第二行：任务冲突数；第三行：轨道上的任务数；第四行：轨道评分
              Pro=zeros(1,length(ind1)); % 各轨道选中概率表
              GF=zeros(4,length(ind1));
              GF(1,1:length(ind1))=ind1;
              %找出各个轨道上现有的任务集合O(ind1(k),ind2)
              for k=1:length(ind1)
                      ind2=find(O(ind1(k),:)); % 选中轨道上所有任务的下标值
                      GF(3,k)=length(ind2); % ind1(k)轨道上的任务数目
                  for x=1:length(ind2) % 轨道ind1(k)上的各个任务ind2(x)比较
                  %t任务在ind1（k）轨道上的时间窗口%%%%%%%%%Stime(t,ind1(k))该次分配的任务t在ind1(k)轨道上的冲突度
                     if  (Stime(O(ind1(k),ind2(x)),ind1(k))<=Etime(t,ind1(k))<=Etime(O(ind1(k),ind2(x)),ind1(k))) || (Stime(O(ind1(k),ind2(x)),ind1(k))<=Stime(t,ind1(k))<=Etime(O(ind1(k),ind2(x)),ind1(k))) 
                          GF(2,k)=GF(2,k)+1;
%                      else
%                           GF(2,k)=GF(2,k)++0;
                     end
                  end
              end
             %%对轨道表进行归一化处理，冲突数和任务数两行取各自的值/总值作为自己在该组轨道中的影响程度大小，冲突度和任务数各自对任务完成的影响因子分别为0.7和0.3
             % 如果各轨道的冲突度和任务数都为0的话，则该部分影响因子为0
             con=sum(GF(2,:));
             eq=sum(GF(3,:));
             if (con==0)&&(eq==0)
                 GF(2,:)=rand;
                 GF(3,:)=rand;
             elseif con==0
                 GF(2,:)=0;
             elseif eq==0
                 GF(3,:)=0;
             else
                 GF(2,:)=(con-GF(2,:))./con;
                 GF(3,:)=(eq-GF(3,:))./eq;
             end
             GF(4,:)=0.7*GF(2,:)+0.3*GF(3,:);
             %%%%%%%%%%%概率选择轨道%%%%%%%%%%%%
             for k=1:length(ind1)
                 Pro(k)=(Tau(t,ind1(k))^Alpha)*(GF(4,k)^Beta);
             end
             Pro=Pro./(sum(Pro)); 
             Pcum=cumsum(Pro);
             Select=find(Pcum>=rand);
             fo=ind1(Select(1)); % 最终选择的轨道
             l=length(find(O(fo,:)));
             O(fo,l+1)=t; % 轨道上t任务的位置
            end
            T(T==t)=[]; % 将该任务从待分配任务集中删除
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%对所有轨道进行任务调度%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%对所有轨道进行任务调度%%%%%%%%%%%%%%%%%%%%%
    for i=1:pathways_num
        number=length(find(O(i,:)));
        if number>0
            %%%%%%%%%%%%%%%%%%%%%%删除其中一个轨道上权重最小的任务以保证跳出局部最优解%%%%%%%%%%%%%%%%
            ch=find(O1(i,:));
            ch1=length(find(O1(i,:)));
            if ch1>1
                PS=zeros(1,ch1);
                for k1=1:ch1
                    PS(k1)=weight(O1(i,ch(k1)));
                end
                ch2=find(PS==min(PS)); % 找到权重最小的任务
                T=[T,O1(i,ch(ch2(1)))];
                for k=ch(ch2(1)):N-1
                    O1(i,k)=O1(i,k+1);
                end
            end

            TA=O1(i,:);
            P2=zeros(1,number); % 待插入任务的权重矩阵
              for k=1:number
                P2(1,k)=weight(O(i,k));
              end
           while number>0
            %%%%%%%%%优先插入权重高的任务%%%%%%%%%%%%%%%%%%%
              ind3=find(P2==max(P2)); % 找到权重最大的任务
              lin=length(ind3);
              if lin==1
                ind4=1; 
                j=O(i,ind3(1));
              else
                ind4=round(rand*(lin-1))+1;
                j=O(i,ind3(ind4)); % 选择待插入任务j
              end
              P2(ind3(ind4))=0; % 将本次插入任务的权重归零
              nf=0;
            %%%%%%%%%%%%%%%%%%%%%%%判断任务j是否能够成功插入%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%当该轨道无任务时直接插入j%%%%%%%%%%%%%%
              if length(find(O1(i,:)))==0
                  TA(1)=j;
                  nf=1;
              end
              %%%%%%%%%%%%%%%%%%%%%%%%%当轨道上有任务时%%%%%%%%%%%%%%%%%%%
              if length(find(O1(i,:)))>0
                  s=length(find(TA));
                  % 判断能否插入起始位置
                  if Etime(j,i) <= Stime(TA(1),i)
                  %if Etime(j,i)+abs(QJ(j)-QJ(TA(1)))<=Stime(TA(1),i) % 判断能否插入起始位置
                     for k1=s:-1:1
                         TA(k1+1)=TA(k1);
                     end
                     TA(1)=j;
                     nf=1;
                  end
                  % 判断能否插入结束位置
                  if Etime(TA(s),i) <= Stime(j,i)
                  %if Etime(TA(s),i)+abs(QJ(TA(s))-QJ(j))<=Stime(j,i)
                      TA(s+1)=j;
                      nf=1;
                  end
                  % 如果任务j可以插在任务site后
                  site=0;
                  for k2=1:s-1
                       if Etime(TA(k2),i) <= Stime(j,i)&&Etime(j,i) <= Stime(TA(k2+1),i)
                        %if Etime(TA(k2),i)+abs(QJ(TA(k2))-QJ(j))<=Stime(j,i)&&Etime(j,i)+abs(QJ(TA(k2+1))-QJ(j))<=Stime(TA(k2+1),i)
                            site=k2;
                       end
                  end
                  if site>0
                     for k3=s:-1:site+1
                         TA(k3+1)=TA(k3);
                     end
                     TA(site+1)=j;
                     nf=1;
                  end
              end
              O1(i,:)=TA;
              if nf==0 % 如果任务未被插入则放入待分配任务集合T中
                  T=[T,j];
              end
              number=number-1;
          end
        end
        while length(find(O1(i,:)))>Energy_C % 如果超过了能量约束
            PB=zeros(1,length(find(O1(i,:))));
            TB=O1(i,:);
            for k=1:length(find(O1(i,:)))
                PB(k)=weight(O1(i,k));
            end
            Delete=find(PB==min(PB));
            delete=Delete(1); % 待删除的任务
            T=[T,TB(delete)]; % 将删除的任务放入待分配任务集中
            for k=delete:N-1
                TB(k)=TB(K+1);
            end
            O1(i,:)=TB;
        end
        mem=0;
        for k=1:length(find(O1(i,:)))
            mem=mem+duration(O1(i,k));
        end
        while mem>Volum_C % 如果超过了容量约束
            PM=zeros(1,length(find(O1(i,:))));
            TM=O1(i,:);
            for k=1:length(find(O1(i,:)))
                PM(k)=weight(O1(i,k));
            end
            Delete=find(PM==min(PM));
            delete=Delete(1); % 待删除的任务
            T=[T,TM(delete)];
            for k=delete:N-1
                TM(k)=TM(K+1);
            end
            O1(i,:)=TM;
        end
    end
    O=zeros(pathways_num,N);%%%%%%%%%%清空此次轨道任务矩阵
    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%对本次迭代未完成的任务增加罚因子%%%%%%%%%%%%%%%%%%%%%%%%%
    F(T)=F(T)+0.2;
    for k=1:pathways_num
        ind=find(O1(k,:));
        if length(ind)>0 % 如果选中的轨道上有任务
            for k2=1:length(ind)
                Profit=Profit+weight(O1(k,ind(k2)));
            end
        end
    end
    PROFIT(NC)=Profit;
    if NC==1
        Pbest=[Pbest,Profit];
    end
    if NC>1
        delta_e=PROFIT(NC)-PROFIT(NC-1);
        if  delta_e>0
            O1=O1;
        else
            if exp(-delta_e/10*G)>rand()
                O1=O1;
            else
                O1=O2;
                T=D;
            end
        end
    end
    if Profit>=max(Pbest)
       Pbest=[Pbest,Profit];
       Obest=O1;
%     else
%        Pbest=[Pbest,max(Pbest)];
%        Obest=Obest;
    end
        %%%%%%%%%%%%%%%%%%%%%%%%%更新信息素%%%%%%%%%%%%%%%%%%%
        num=length(find(O1));
        Delta_Tau=zeros(N,pathways_num);
        for k1=1:pathways_num
            for k2=1:length(find(O1(k1,:)))
                Delta_Tau(O1(k1,k2),k1)=Profit/num;
            end
        end
        Tau=(1-Rho).*Tau+Delta_Tau;
        NC=NC+1;
        G=G*DG;
end
wrs=length(find(Obest));

T_schedule = zeros(1,tasks_num); % 已完成任务集合
% 根据Obest获得T_schedule，从而T_unschedule
for i = 1:pathways_num
    for j = 1:N
        if Obest(i,j) ~= 0
            task_code = Tasks(Obest(i,j),2); % 找到对应的任务编号
            if T_schedule(task_code) == 0
                T_schedule(task_code) = 1;
            else
                T_schedule(task_code) = inf;
                disp('error');
            end
        end
    end
end
T_total = zeros(1,tasks_num);
T_total(Tasks(:,2)') = 1;
T_unschedule = T_total - T_schedule; % 未完成任务集合
profit = Pbest(end); % 最优适应度

end


