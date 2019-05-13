
%功能描述：计算任意两点之间的最短距离
function score=Score(newsequence,Dis_Refer,interval,profit,tw,airportnumber)
% 初始化
mission_airport=zeros(size(newsequence,1),size(newsequence,2));
mission_interval=zeros(size(newsequence,1),size(newsequence,2));
mission_starting=zeros(size(newsequence,1),size(newsequence,2));
mission_location=zeros(size(newsequence,1),size(newsequence,2));
mission_profit=zeros(size(newsequence,1),size(newsequence,2));
for i=1:size(newsequence,1)
    for j=1:size(newsequence,2)
        if newsequence(i,j)~=0
            mission_airport(i,j)=Dis_Refer(newsequence(i,j),airportnumber);
            mission_interval(i,j)=interval(newsequence(i,j));
            mission_starting(i,j)=tw(newsequence(i,j),1);
            mission_profit(i,j)=profit(newsequence(i,j),1);
        end
    end
end
for i=1:size(newsequence,1)
    for j=1:size(newsequence,2)
        if newsequence(i,j)~=0
            for p=1:size(newsequence,2)
                if p~=j&&newsequence(i,p)~=0
                  mission_location(i,j)=mission_location(i,j)+Dis_Refer(newsequence(i,j),newsequence(i,p));
                end
            end
        end
    end
end
%对每一个指标进行归一化
% new_record=sum(newsequence~=0,2);
sum_mission_airport=sum(mission_airport,2);
sum_mission_interval=sum(mission_interval,2);
sum_mission_starting=sum(mission_starting,2);
sum_mission_profit=sum(mission_profit,2);
sum_mission_location=sum(mission_location,2);
for i=1:size(newsequence,1)
    for j=1:size(newsequence,2)
        mission_airport(i,j)=mission_airport(i,j)/sum_mission_airport(i);
        mission_interval(i,j)=mission_interval(i,j)/sum_mission_interval(i);
        mission_starting(i,j)=mission_starting(i,j)/sum_mission_starting(i);
        mission_profit(i,j)=mission_profit(i,j)/sum_mission_profit(i);
        mission_location(i,j)=mission_location(i,j)/sum_mission_location(i);
    end
end
for i=1:size(newsequence,1)
    for j=1:size(newsequence,2)
        %可以为每一个指标加入权重
         mission_score(i,j)=0.1*mission_airport(i,j)+0.2*mission_interval(i,j)+0.2*mission_starting(i,j)+0.3*mission_profit(i,j)+0.2*mission_location(i,j);
         mission_score(i,j)=roundn(mission_score(i,j),-4);
    end
end

%对mission_score按照从大到小进行排序并返回任务序号
[bigmission_score,index]=sort(mission_score,2,'descend');
for i=1:size(newsequence,1)
    for j=1:size(newsequence,2)
         bigsequence(i,j)=newsequence(i,index(i,j));
    end
end
score.mission_score=mission_score;
score.mission_airport=mission_airport;
score.mission_interval=mission_interval;
score.mission_starting=mission_starting;
score.mission_location=mission_location;
score.bigmission_score=bigmission_score;
score.bigsequence=bigsequence;
end
