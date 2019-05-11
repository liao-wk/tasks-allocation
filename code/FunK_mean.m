function [ resX,resY,record,sequence,alter_sequence] = FunK_mean( x,y,k,task )
% ���ܣ�
%     ʵ��k-mean�����㷨
% ���룺
%     ��ά���ݣ��ֱ���x,y����һά������������ά��
%     k �Ƿֳɵ���������
%     task�Ǹû�����Ҫִ�е��������
% �����
%     k�е���������
%     ��Ӧͬ���ĵ�n�У�����ŵ�n�������Ԫ��
%     record: ��¼��ÿһ�е���ЧԪ�صĸ���

    j = 1;
    % ������Ԥ����һЩ�ռ�
    % seedX �� seedY �д������������
    seedX = zeros(1,k);
    seedY = zeros(1,k);
    oldSeedX = zeros(1,k);
    oldSeedY = zeros(1,k);
    resX0 = zeros(k,length(x));
    resY0 = zeros(k,length(x));
    oldsequence0=zeros(size(resX0,1),size(resX0,2));
    % record������¼resX��ÿһ����ЧԪ�صĸ���
    record0 = zeros(1,k); 
    for i = 1:k % ����k���������, ע�⣺ �������������Ԫ�ؼ���
        seedX(i) = x(ceil(rand()*length(resX0)));
        seedY(i) = y(ceil(rand()*length(resX0)));
        % Ϊ��֤���Ӳ��ص�
        if (i > 1 && seedX(i) == seedX(i-1) && seedY(i) == seedY(i-1))
            i =i -1; % ���²���һ������
        end
    end
    %k�����ĵ������
    seedX;
    seedY;
    while 1%��Զѭ��
        %disp(['�½��'])
        record0(:) = 0; % ����Ϊ��
        resX0(:) = 0;
        resY0(:) = 0;
        oldsequence0(:) = 0;
        for i = 1:length(x) % ������Ԫ�ر���
            % �������жϱ���Ԫ��Ӧ�ù�Ϊ��һ�࣬���������Ǹ���ŷ����þ����������ж�
            % k-mean�㷨��ΪԪ��Ӧ�ù�Ϊ������������Ӵ������
            distanceMin = 1;
            for j = 2:k
                if (power(x(i)-seedX(distanceMin),2)+power(y(i)-seedY(distanceMin),2))... 
                    > (power(x(i)-seedX(j),2) + power(y(i)-seedY(j),2))
                    distanceMin = j;
                end
            end
            % ������Ԫ�ص�������鲢
            resX0(distanceMin,record0(distanceMin)+1) = x(i);
            resY0(distanceMin,record0(distanceMin)+1) = y(i);
            oldsequence0(distanceMin,record0(distanceMin)+1)=task(i);
            record0(distanceMin) = record0(distanceMin) + 1;
        end
        oldSeedX = seedX;
        oldSeedY = seedY;
        %�ƶ���������������
        record0;
        for i = 1:k
            if record0(i) == 0
                continue;
            end
            seedX(i) = sum(resX0(i,:))/record0(i);
            seedY(i) = sum(resY0(i,:))/record0(i);
        end

        % ������εõ������Ӻ��ϴε�����һ�£�����Ϊ������ϡ�
        if mean([seedX == oldSeedX seedY == oldSeedY]) == 1 % ��仰���������˼���� if seedX == oldSeedX && seedY == oldSeedY
            break;
        end
    end
    %�������ֻ�Ƕ�resX,resY��sequence��ռ�õ��ڴ��С���м򵥵��Ż�
    maxPos = max(record0);
    resX0 = resX0(:,1:maxPos);
    resY0 = resY0(:,1:maxPos);
    sequence0=oldsequence0(:,1:maxPos);
    %ȥ����0��0������
    resX0(resX0==0)=inf;
    resY0(resY0==0)=inf;
    
    %����record0�Ĵ�С��sequence0��resX0,resY0,oldsequence0��������
    [record,index]=sort(record0,'descend');
    for i=1:k
        sequence(i,:)=sequence0(index(i),:);
        oldsequence(i,:)=oldsequence0(index(i),:);
        resX(i,:)=resX0(index(i),:);
        resY(i,:)=resY0(index(i),:);
    end 
    alter_sequence=sequence;

end
