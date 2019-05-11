
%功能描述：当第一个任务改变的时候，更新其他任务的执行序号
function location=Update_one(location,q)
% 输入：行向量
%           新的第一个任务所在的位置
% 输出：新的行向量
location(q)=1;
for i=1:q-1
    location(i)=location(i)+1;
end
end
