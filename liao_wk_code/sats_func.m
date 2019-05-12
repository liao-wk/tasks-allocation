function [profit,T_schedule,T_unschedule] = sats_func(solu,sats,tasks,subplan,Energy_C,Volum_C,agent_code)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
% Volum_C = 3000; % �۲�ʱ������
% Energy_C = 140; % ��󿪹ػ���������
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
%% ������ǵ��������㷨 
N=size(Tasks,1);% Ŀ�����
pathway_num = 16; % �������ǵĹ������
% ����������������Ǳ��
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
G=100; % ��ʼ�¶�
l=zeros(pathways_num,1);
NC=1;
T=1:N;
F=zeros(1,N); % ������
Rho=0.1; % ��Ϣ������ϵ��
Alpha=1;
Beta=5;
DG=0.99;% ˥������
Tau=ones(N,pathways_num);% ��Ϣ�ؾ���
Pbest=[];
%%%%%%%%%%%%%%%%�ҳ����ܹ��۲������%%%%%%%%%%%%%%%%%%%
for i=1:N
    if isempty(find(window(i,:),1))
        T(T==i)=[];% �����ܱ��۲⵽������ɾ��
    end
end
O=zeros(pathways_num,N);% ����������
Obest=zeros(pathways_num,N);% ���ŷ���
O1=zeros(pathways_num,N);

while G> 0.01
    D=T;% ��¼�ϴδ����������
    O2=O1;% ��¼�ϴεķ��䷽��
    Profit=0;% ���±��ε���������ֵ
    while ~isempty(T)
              P1=zeros(1,length(T));% ����T���ϵ�Ȩ��
              for i=1:length(T)
                 P1(i)=weight(T(i))-F(T(i));
              end
              t1=find(P1==max(P1));
              t=T(t1(1));% �ҵ�����ֵ��������t
              %�ҵ��ܹ۲⵽t����Ĺ������ind1,ind1(k)����ĳһ������
              ind1=find(window(t,:));
              if length(ind1)==1
                 fo=ind1(1);
                 l=length(find(O(fo,:)));
                 O(fo,l+1)=t;
              end
              if length(ind1)>1
              %���������־��󣬵�һ�У�����ţ��ڶ��У������ͻ���������У�����ϵ��������������У��������
              Pro=zeros(1,length(ind1)); % �����ѡ�и��ʱ�
              GF=zeros(4,length(ind1));
              GF(1,1:length(ind1))=ind1;
              %�ҳ�������������е����񼯺�O(ind1(k),ind2)
              for k=1:length(ind1)
                      ind2=find(O(ind1(k),:)); % ѡ�й��������������±�ֵ
                      GF(3,k)=length(ind2); % ind1(k)����ϵ�������Ŀ
                  for x=1:length(ind2) % ���ind1(k)�ϵĸ�������ind2(x)�Ƚ�
                  %t������ind1��k������ϵ�ʱ�䴰��%%%%%%%%%Stime(t,ind1(k))�ôη��������t��ind1(k)����ϵĳ�ͻ��
                     if  (Stime(O(ind1(k),ind2(x)),ind1(k))<=Etime(t,ind1(k))<=Etime(O(ind1(k),ind2(x)),ind1(k))) || (Stime(O(ind1(k),ind2(x)),ind1(k))<=Stime(t,ind1(k))<=Etime(O(ind1(k),ind2(x)),ind1(k))) 
                          GF(2,k)=GF(2,k)+1;
%                      else
%                           GF(2,k)=GF(2,k)++0;
                     end
                  end
              end
             %%�Թ������й�һ��������ͻ��������������ȡ���Ե�ֵ/��ֵ��Ϊ�Լ��ڸ������е�Ӱ��̶ȴ�С����ͻ�Ⱥ����������Զ�������ɵ�Ӱ�����ӷֱ�Ϊ0.7��0.3
             % ���������ĳ�ͻ�Ⱥ���������Ϊ0�Ļ�����ò���Ӱ������Ϊ0
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
             %%%%%%%%%%%����ѡ����%%%%%%%%%%%%
             for k=1:length(ind1)
                 Pro(k)=(Tau(t,ind1(k))^Alpha)*(GF(4,k)^Beta);
             end
             Pro=Pro./(sum(Pro)); 
             Pcum=cumsum(Pro);
             Select=find(Pcum>=rand);
             fo=ind1(Select(1)); % ����ѡ��Ĺ��
             l=length(find(O(fo,:)));
             O(fo,l+1)=t; % �����t�����λ��
            end
            T(T==t)=[]; % ��������Ӵ�����������ɾ��
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%�����й�������������%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%�����й�������������%%%%%%%%%%%%%%%%%%%%%
    for i=1:pathways_num
        number=length(find(O(i,:)));
        if number>0
            %%%%%%%%%%%%%%%%%%%%%%ɾ������һ�������Ȩ����С�������Ա�֤�����ֲ����Ž�%%%%%%%%%%%%%%%%
            ch=find(O1(i,:));
            ch1=length(find(O1(i,:)));
            if ch1>1
                PS=zeros(1,ch1);
                for k1=1:ch1
                    PS(k1)=weight(O1(i,ch(k1)));
                end
                ch2=find(PS==min(PS)); % �ҵ�Ȩ����С������
                T=[T,O1(i,ch(ch2(1)))];
                for k=ch(ch2(1)):N-1
                    O1(i,k)=O1(i,k+1);
                end
            end

            TA=O1(i,:);
            P2=zeros(1,number); % �����������Ȩ�ؾ���
              for k=1:number
                P2(1,k)=weight(O(i,k));
              end
           while number>0
            %%%%%%%%%���Ȳ���Ȩ�ظߵ�����%%%%%%%%%%%%%%%%%%%
              ind3=find(P2==max(P2)); % �ҵ�Ȩ����������
              lin=length(ind3);
              if lin==1
                ind4=1; 
                j=O(i,ind3(1));
              else
                ind4=round(rand*(lin-1))+1;
                j=O(i,ind3(ind4)); % ѡ�����������j
              end
              P2(ind3(ind4))=0; % �����β��������Ȩ�ع���
              nf=0;
            %%%%%%%%%%%%%%%%%%%%%%%�ж�����j�Ƿ��ܹ��ɹ�����%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%���ù��������ʱֱ�Ӳ���j%%%%%%%%%%%%%%
              if length(find(O1(i,:)))==0
                  TA(1)=j;
                  nf=1;
              end
              %%%%%%%%%%%%%%%%%%%%%%%%%�������������ʱ%%%%%%%%%%%%%%%%%%%
              if length(find(O1(i,:)))>0
                  s=length(find(TA));
                  % �ж��ܷ������ʼλ��
                  if Etime(j,i) <= Stime(TA(1),i)
                  %if Etime(j,i)+abs(QJ(j)-QJ(TA(1)))<=Stime(TA(1),i) % �ж��ܷ������ʼλ��
                     for k1=s:-1:1
                         TA(k1+1)=TA(k1);
                     end
                     TA(1)=j;
                     nf=1;
                  end
                  % �ж��ܷ�������λ��
                  if Etime(TA(s),i) <= Stime(j,i)
                  %if Etime(TA(s),i)+abs(QJ(TA(s))-QJ(j))<=Stime(j,i)
                      TA(s+1)=j;
                      nf=1;
                  end
                  % �������j���Բ�������site��
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
              if nf==0 % �������δ�������������������񼯺�T��
                  T=[T,j];
              end
              number=number-1;
          end
        end
        while length(find(O1(i,:)))>Energy_C % �������������Լ��
            PB=zeros(1,length(find(O1(i,:))));
            TB=O1(i,:);
            for k=1:length(find(O1(i,:)))
                PB(k)=weight(O1(i,k));
            end
            Delete=find(PB==min(PB));
            delete=Delete(1); % ��ɾ��������
            T=[T,TB(delete)]; % ��ɾ����������������������
            for k=delete:N-1
                TB(k)=TB(K+1);
            end
            O1(i,:)=TB;
        end
        mem=0;
        for k=1:length(find(O1(i,:)))
            mem=mem+duration(O1(i,k));
        end
        while mem>Volum_C % �������������Լ��
            PM=zeros(1,length(find(O1(i,:))));
            TM=O1(i,:);
            for k=1:length(find(O1(i,:)))
                PM(k)=weight(O1(i,k));
            end
            Delete=find(PM==min(PM));
            delete=Delete(1); % ��ɾ��������
            T=[T,TM(delete)];
            for k=delete:N-1
                TM(k)=TM(K+1);
            end
            O1(i,:)=TM;
        end
    end
    O=zeros(pathways_num,N);%%%%%%%%%%��մ˴ι���������
    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%�Ա��ε���δ��ɵ��������ӷ�����%%%%%%%%%%%%%%%%%%%%%%%%%
    F(T)=F(T)+0.2;
    for k=1:pathways_num
        ind=find(O1(k,:));
        if length(ind)>0 % ���ѡ�еĹ����������
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
        %%%%%%%%%%%%%%%%%%%%%%%%%������Ϣ��%%%%%%%%%%%%%%%%%%%
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

T_schedule = zeros(1,tasks_num); % ��������񼯺�
% ����Obest���T_schedule���Ӷ�T_unschedule
for i = 1:pathways_num
    for j = 1:N
        if Obest(i,j) ~= 0
            task_code = Tasks(Obest(i,j),2); % �ҵ���Ӧ��������
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
T_unschedule = T_total - T_schedule; % δ������񼯺�
profit = Pbest(end); % ������Ӧ��

end


