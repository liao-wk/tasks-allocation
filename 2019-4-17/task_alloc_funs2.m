function funs = task_alloc_funs2
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
funs.Conflict_average = @Conflict_average;
funs.Opp_Weight = @Opp_Weight; % �õ������������µ������
end
%% �Ӻ���1�����������ڲ�ͬ�Ĺ۲���Դ�µ�ƽ����ͻ��
function Conflict_ave = Conflict_average(tasks_num,resource_num,opp,Conflict)
Conflict_ave = zeros(tasks_num,resource_num);
% �����������д���0��ÿһ�е�Ԫ�ظ���
for j = 1:resource_num
    A = opp(opp(:,j) > 0); % ��������д���0��Ԫ������
    num = size(A,1)-1; % ��������������һ
    Conflict_ave(:,j) = Conflict(:,j)/num; % ����Դr�µĸ��������ƽ����ͻ��
end
end
%% �Ӻ���2�����������ڹ۲���Դ�µ�����ȣ���ֵԽ�ߣ��۲����ԽС���������Դ�������Խ��
function oppweight = Opp_Weight(tasks_num,resource_num,opp,W)
oppweight = zeros(tasks_num,resource_num);
for j = 1:resource_num
    for i = 1:tasks_num
        if opp(i,j) ~= 0
            oppweight(i,j) = W(i)/opp(i,j); % ����i��Ȩ�س�������Դj�µĻ���
        else
            oppweight(i,j) = 0; % �����������Դ��û�й۲���ᣬ��ô�����Ϊ0.
        end
    end
end
end

