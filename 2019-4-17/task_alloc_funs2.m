function funs = task_alloc_funs2
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
funs.Conflict_average = @Conflict_average;
funs.Opp_Weight = @Opp_Weight; % 得到任务在卫星下的需求度
end
%% 子函数1，计算任务在不同的观测资源下的平均冲突度
function Conflict_ave = Conflict_average(tasks_num,resource_num,opp,Conflict)
Conflict_ave = zeros(tasks_num,resource_num);
% 计算机会矩阵中大于0的每一列的元素个数
for j = 1:resource_num
    A = opp(opp(:,j) > 0); % 机会矩阵中大于0的元素向量
    num = size(A,1)-1; % 该向量的行数减一
    Conflict_ave(:,j) = Conflict(:,j)/num; % 在资源r下的各个任务的平均冲突度
end
end
%% 子函数2，计算任务在观测资源下的需求度，价值越高，观测机会越小的任务对资源的需求度越高
function oppweight = Opp_Weight(tasks_num,resource_num,opp,W)
oppweight = zeros(tasks_num,resource_num);
for j = 1:resource_num
    for i = 1:tasks_num
        if opp(i,j) ~= 0
            oppweight(i,j) = W(i)/opp(i,j); % 任务i的权重除以在资源j下的机会
        else
            oppweight(i,j) = 0; % 如果任务在资源下没有观测机会，那么需求度为0.
        end
    end
end
end

