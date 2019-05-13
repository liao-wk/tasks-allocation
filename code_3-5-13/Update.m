
%功能描述：当第一个任务改变的时候，更新其他任务的执行序号
function location=Update(location,q,p)
% 输入：行向量
%           新的任务所在的位置
%           新任务插在某个任务的前面
% 输出：新的行向量
location(q)=location(p);
for i=1:q-1
    if location(i)>=location(q)
       location(i)=location(i)+1;
    end
end
end