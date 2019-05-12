%功能描述：角度调整
function [resX,resY,newsequence] = Angle_adjustment(x,y,N,x0,y0,sequence,alter_sequence,resX,resY,record)
%x,y依旧采用最初的任务集
%计算在以机场为坐标原点的坐标系中，任务点与机场连线与正横坐标的夹角大小%
%angle行数和列数与sequence一致，存储着夹角值%
angle=zeros(size(alter_sequence,1),size(alter_sequence,2));
for i=1:N
    for j=1:size(sequence,2)
        if sequence(i,j)~=0
           sintheta(i,j)=(y(sequence(i,j))-y0)/sqrt((x(sequence(i,j))-x0)^2+(y(sequence(i,j))-y0)^2);
           costheta(i,j)=(x(sequence(i,j))-x0)/sqrt((x(sequence(i,j))-x0)^2+(y(sequence(i,j))-y0)^2);
           %求出角度
           theta(i,j)=atan2(sintheta(i,j),costheta(i,j));
           angle(i,j)=theta(i,j)*180/pi;
           if angle(i,j)<0
              angle(i,j)=angle(i,j)+360;
           end
        end
    end
end
%找出angle中每行的最小值和最大值,minangle和maxangle是N*1的矩阵
angle1=angle;
angle2=angle;
angle1(angle1==0)=inf; 
minangle=min(angle1,[],2);
angle2(angle2==0)=-inf; 
maxangle=max(angle2,[],2);
%在360度的空间内选取小范围的角度
for i=1:N
    for j=1:size(sequence,2)
      if maxangle(i)-minangle(i)>=180
        if angle1(i,j)>180
          angle1(i,j)=angle1(i,j)-360;
          angle2(i,j)=angle2(i,j)-360;
          minangle(i)=min(angle1(i,:));
          maxangle(i)=max(angle2(i,:));
          angle(i,j)=angle1(i,j);
          end
      end
    end
end


%计算每一个簇的任务点到机场的距离%
%distance行数和列数与oldsequence一致，存储着距离%
distance=zeros(size(alter_sequence,1),size(alter_sequence,2));
for i=1:N
    for j=1:size(sequence,2)
        if sequence(i,j)~=0
            distance(i,j)=sqrt((x(sequence(i,j))-x0)^2+(y(sequence(i,j))-y0)^2);
        end
    end
end
%找出distance中每行最大值,maxdistance是N*1的矩阵
maxdistance=max(distance,[],2);


%根据角度对k-means算法分配的结果进行调整
for i=1:N
    for j=1:size(sequence,2)
        for m=1:N
%             调整过的任务点不再参加下次调整
            if alter_sequence(i,j)~=0 && alter_sequence(i,j)==sequence(i,j) 
                if i~=m
                    %有些角度在第三象限和第四象限，有些簇的表达这些角是用负号表示，有的是180-360
                    if (minangle(m)<0&&angle(i,j)>180)||(minangle(m)>180&&angle(i,j)<0)
                       if((angle(i,j)-360)>=minangle(m)&&(angle(i,j)-360)<=maxangle(m)&&distance(i,j)<=maxdistance(m))
                          resX(m,record(m)+1)=x(alter_sequence(i,j));
                          resY(m,record(m)+1)=y(alter_sequence(i,j));
                          resX(i,j)=inf;
                          resY(i,j)=inf;
                          alter_sequence(m,record(m)+1)=sequence(i,j);
                          alter_sequence(i,j)=0;
                          %假设是将sequence(i,j)调入第m个聚类中，则angle中也会相应作出调整,第i个聚类的最大值和最小值也会作出相应变化;第i个聚类的距离最大值也会变化
                          angle1(i,j)=inf;
                          angle2(i,j)=-inf;
                          angle1(m,record(m)+1)=angle(i,j)-360;
                          angle2(m,record(m)+1)=angle(i,j)-360;
                          angle(m,record(m)+1)=angle(i,j)-360;
                          angle(i,j)=0;
                          distance(m,record(m)+1)=distance(i,j);
                          distance(i,j)=0;
                          record(m)=record(m)+1;
                          record(i)=record(i)-1;
                          %更新第i个聚类的角度最大值和最小值以及距离最大值
                          minangle(i)=min(angle1(i,:));
                          maxangle(i)=max(angle2(i,:));
                          maxdistance(i)=max(distance(i,:));
                       else
                           if ((angle(i,j)+360)>=minangle(m)&&(angle(i,j)+360)<=maxangle(m)&&distance(i,j)<=maxdistance(m))
                               resX(m,record(m)+1)=x(alter_sequence(i,j));
                               resY(m,record(m)+1)=y(alter_sequence(i,j));
                               resX(i,j)=inf;
                               resY(i,j)=inf;
                               alter_sequence(m,record(m)+1)=sequence(i,j);
                               alter_sequence(i,j)=0;
                               %假设是将sequence(i,j)调入第m个聚类中，则angle中也会相应作出调整,第i个聚类的最大值和最小值也会作出相应变化;第i个聚类的距离最大值也会变化
                               angle1(i,j)=inf;
                               angle2(i,j)=-inf;
                               angle1(m,record(m)+1)=angle(i,j)-360;
                               angle2(m,record(m)+1)=angle(i,j)-360;
                               angle(m,record(m)+1)=angle(i,j)-360;
                               angle(i,j)=0;
                               distance(m,record(m)+1)=distance(i,j);
                               distance(i,j)=0;
                               record(m)=record(m)+1;
                               record(i)=record(i)-1;
                               %更新第i个聚类的角度最大值和最小值以及距离最大值
                               minangle(i)=min(angle1(i,:));
                               maxangle(i)=max(angle2(i,:));
                               maxdistance(i)=max(distance(i,:)); 
                           end
                       end
                    else 
                        if(angle(i,j)>=minangle(m)&&angle(i,j)<=maxangle(m)&&distance(i,j)<=distance(m))
                          resX(m,record(m)+1)=x(alter_sequence(i,j));
                          resY(m,record(m)+1)=y(alter_sequence(i,j)); 
                          resX(i,j)=inf; 
                          resY(i,j)=inf; 
                          alter_sequence(m,record(m)+1)=sequence(i,j);
                          alter_sequence(i,j)=0;
                          %假设是将sequence(i,j)调入第m个聚类中，则angle中也会相应作出调整,第i个聚类的最大值和最小值也会作出相应变化
                          angle1(i,j)=inf;
                          angle2(i,j)=-inf;
                          angle1(m,record(m)+1)=angle(i,j);
                          angle2(m,record(m)+1)=angle(i,j);
                          angle(m,record(m)+1)=angle(i,j);
                          angle(i,j)=0;
                          distance(m,record(m)+1)=distance(i,j);
                          distance(i,j)=0;
                          record(m)=record(m)+1;
                          record(i)=record(i)-1;
                          %更新第i个聚类的最大值和最小值
                          minangle(i)=min(angle1(i,:));
                          maxangle(i)=max(angle2(i,:));
                          maxdistance(i)=max(distance(i,:));
                        end
                    end
                end
               else 
                   break;
            end
        end
     end
end
newsequence=alter_sequence;
A=zeros(N,size(alter_sequence,2));

for i=1:N
    k=1;
    for j=1:size(alter_sequence,2)
        if newsequence(i,j)~=0
            A(i,j)=k;
            k=k+1;
        end
    end
end
for i=1:N
    for j=1:size(alter_sequence,2)
        if newsequence(i,j)~=0 && j~=A(i,j)
            newsequence(i,A(i,j))=newsequence(i,j);
            newsequence(i,j)=0;
        end
    end
end     
resX(resX==0)=inf;
resY(resY==0)=inf;
for i=1:N
    for j=1:size(alter_sequence,2)
        if resX(i,j)~=inf&& j~=A(i,j)
            resX(i,A(i,j))=resX(i,j);
            resX(i,j)=inf;
        end
    end
end
for i=1:N
    for j=1:size(alter_sequence,2)
        if resY(i,j)~=inf && j~=A(i,j)
            resY(i,A(i,j))=resY(i,j);
            resY(i,j)=inf;
        end
    end
end     
end

