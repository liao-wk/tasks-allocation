function [ resX,resY,record,sequence,alter_sequence] = FunK_mean( x,y,k,task )
% 功能：
%     实现k-mean聚类算法
% 输入：
%     二维数据，分别用x,y两个一维向量代表两个维度
%     k 是分成的类别的数量
%     task是该机场所要执行的任务序号
% 输出：
%     k行的两个矩阵
%     对应同样的第n行，存放着第n类的所有元素
%     record: 记录着每一行的有效元素的个数

    j = 1;
    % 下面是预分配一些空间
    % seedX 和 seedY 中存放着所有种子
    seedX = zeros(1,k);
    seedY = zeros(1,k);
    oldSeedX = zeros(1,k);
    oldSeedY = zeros(1,k);
    resX0 = zeros(k,length(x));
    resY0 = zeros(k,length(x));
    oldsequence0=zeros(size(resX0,1),size(resX0,2));
    % record用来记录resX中每一行有效元素的个数
    record0 = zeros(1,k); 
    for i = 1:k % 产生k个随机种子, 注意： 随机种子是来自元素集合
        seedX(i) = x(ceil(rand()*length(resX0)));
        seedY(i) = y(ceil(rand()*length(resX0)));
        % 为保证种子不重叠
        if (i > 1 && seedX(i) == seedX(i-1) && seedY(i) == seedY(i-1))
            i =i -1; % 重新产生一个种子
        end
    end
    %k个中心点的坐标
    seedX;
    seedY;
    while 1%永远循环
        %disp(['新结果'])
        record0(:) = 0; % 重置为零
        resX0(:) = 0;
        resY0(:) = 0;
        oldsequence0(:) = 0;
        for i = 1:length(x) % 对所有元素遍历
            % 下面是判断本次元素应该归为哪一类，这里我们是根据欧几里得距离进行类别判定
            % k-mean算法认为元素应该归为距离最近的种子代表的类
            distanceMin = 1;
            for j = 2:k
                if (power(x(i)-seedX(distanceMin),2)+power(y(i)-seedY(distanceMin),2))... 
                    > (power(x(i)-seedX(j),2) + power(y(i)-seedY(j),2))
                    distanceMin = j;
                end
            end
            % 将本次元素点进行类别归并
            resX0(distanceMin,record0(distanceMin)+1) = x(i);
            resY0(distanceMin,record0(distanceMin)+1) = y(i);
            oldsequence0(distanceMin,record0(distanceMin)+1)=task(i);
            record0(distanceMin) = record0(distanceMin) + 1;
        end
        oldSeedX = seedX;
        oldSeedY = seedY;
        %移动种子至其类中心
        record0;
        for i = 1:k
            if record0(i) == 0
                continue;
            end
            seedX(i) = sum(resX0(i,:))/record0(i);
            seedY(i) = sum(resY0(i,:))/record0(i);
        end

        % 如果本次得到的种子和上次的种子一致，则认为分类完毕。
        if mean([seedX == oldSeedX seedY == oldSeedY]) == 1 % 这句话所想表达的意思就是 if seedX == oldSeedX && seedY == oldSeedY
            break;
        end
    end
    %下面代码只是对resX,resY，sequence所占用的内存大小进行简单的优化
    maxPos = max(record0);
    resX0 = resX0(:,1:maxPos);
    resY0 = resY0(:,1:maxPos);
    sequence0=oldsequence0(:,1:maxPos);
    %去除（0，0）坐标
    resX0(resX0==0)=inf;
    resY0(resY0==0)=inf;
    
    %根据record0的大小对sequence0，resX0,resY0,oldsequence0进行排序
    [record,index]=sort(record0,'descend');
    for i=1:k
        sequence(i,:)=sequence0(index(i),:);
        oldsequence(i,:)=oldsequence0(index(i),:);
        resX(i,:)=resX0(index(i),:);
        resY(i,:)=resY0(index(i),:);
    end 
    alter_sequence=sequence;

end
