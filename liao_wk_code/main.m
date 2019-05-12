
%功能描述：运行这个脚本，就可以连续执行30次
iter_num = 30; % 迭代次数
tasks_num = 200; % 任务数量
subplan_num = 4; % 子规划中心的数量
Results = zeros(iter_num,tasks_num*subplan_num); % 运行结果存储于此
Results_fit = zeros(iter_num,1); % 运行结果存储于此
for i = 1:iter_num
	disp(i);
	[Results(i,:),Results_fit(i)] = ActFun(i);
end
% 找出适应度最高的结果
[best_fit,index1] = max(Results_fit); % 最优适应度值
best_so_far = Results(index1,:); % 最优解向量
% 将结果储存在excel当中
xls.write('Results.xlsx',Results)
xls.write('Results_fit.xlsx',Results_fit)