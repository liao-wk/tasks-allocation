%�����������Ƕȵ���
function [resX,resY,newsequence] = Angle_adjustment(x,y,N,x0,y0,sequence,alter_sequence,resX,resY,record)
%x,y���ɲ������������
%�������Ի���Ϊ����ԭ�������ϵ�У�������������������������ļнǴ�С%
%angle������������sequenceһ�£��洢�żн�ֵ%
angle=zeros(size(alter_sequence,1),size(alter_sequence,2));
for i=1:N
    for j=1:size(sequence,2)
        if sequence(i,j)~=0
           sintheta(i,j)=(y(sequence(i,j))-y0)/sqrt((x(sequence(i,j))-x0)^2+(y(sequence(i,j))-y0)^2);
           costheta(i,j)=(x(sequence(i,j))-x0)/sqrt((x(sequence(i,j))-x0)^2+(y(sequence(i,j))-y0)^2);
           %����Ƕ�
           theta(i,j)=atan2(sintheta(i,j),costheta(i,j));
           angle(i,j)=theta(i,j)*180/pi;
           if angle(i,j)<0
              angle(i,j)=angle(i,j)+360;
           end
        end
    end
end
%�ҳ�angle��ÿ�е���Сֵ�����ֵ,minangle��maxangle��N*1�ľ���
angle1=angle;
angle2=angle;
angle1(angle1==0)=inf; 
minangle=min(angle1,[],2);
angle2(angle2==0)=-inf; 
maxangle=max(angle2,[],2);
%��360�ȵĿռ���ѡȡС��Χ�ĽǶ�
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


%����ÿһ���ص�����㵽�����ľ���%
%distance������������oldsequenceһ�£��洢�ž���%
distance=zeros(size(alter_sequence,1),size(alter_sequence,2));
for i=1:N
    for j=1:size(sequence,2)
        if sequence(i,j)~=0
            distance(i,j)=sqrt((x(sequence(i,j))-x0)^2+(y(sequence(i,j))-y0)^2);
        end
    end
end
%�ҳ�distance��ÿ�����ֵ,maxdistance��N*1�ľ���
maxdistance=max(distance,[],2);


%���ݽǶȶ�k-means�㷨����Ľ�����е���
for i=1:N
    for j=1:size(sequence,2)
        for m=1:N
%             ������������㲻�ٲμ��´ε���
            if alter_sequence(i,j)~=0 && alter_sequence(i,j)==sequence(i,j) 
                if i~=m
                    %��Щ�Ƕ��ڵ������޺͵������ޣ���Щ�صı����Щ�����ø��ű�ʾ���е���180-360
                    if (minangle(m)<0&&angle(i,j)>180)||(minangle(m)>180&&angle(i,j)<0)
                       if((angle(i,j)-360)>=minangle(m)&&(angle(i,j)-360)<=maxangle(m)&&distance(i,j)<=maxdistance(m))
                          resX(m,record(m)+1)=x(alter_sequence(i,j));
                          resY(m,record(m)+1)=y(alter_sequence(i,j));
                          resX(i,j)=inf;
                          resY(i,j)=inf;
                          alter_sequence(m,record(m)+1)=sequence(i,j);
                          alter_sequence(i,j)=0;
                          %�����ǽ�sequence(i,j)�����m�������У���angle��Ҳ����Ӧ��������,��i����������ֵ����СֵҲ��������Ӧ�仯;��i������ľ������ֵҲ��仯
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
                          %���µ�i������ĽǶ����ֵ����Сֵ�Լ��������ֵ
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
                               %�����ǽ�sequence(i,j)�����m�������У���angle��Ҳ����Ӧ��������,��i����������ֵ����СֵҲ��������Ӧ�仯;��i������ľ������ֵҲ��仯
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
                               %���µ�i������ĽǶ����ֵ����Сֵ�Լ��������ֵ
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
                          %�����ǽ�sequence(i,j)�����m�������У���angle��Ҳ����Ӧ��������,��i����������ֵ����СֵҲ��������Ӧ�仯
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
                          %���µ�i����������ֵ����Сֵ
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

