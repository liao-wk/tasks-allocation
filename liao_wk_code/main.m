
%������������������ű����Ϳ�������ִ��30��
iter_num = 30; % ��������
tasks_num = 200; % ��������
subplan_num = 4; % �ӹ滮���ĵ�����
Results = zeros(iter_num,tasks_num*subplan_num); % ���н���洢�ڴ�
Results_fit = zeros(iter_num,1); % ���н���洢�ڴ�
for i = 1:iter_num
	disp(i);
	[Results(i,:),Results_fit(i)] = ActFun(i);
end
% �ҳ���Ӧ����ߵĽ��
[best_fit,index1] = max(Results_fit); % ������Ӧ��ֵ
best_so_far = Results(index1,:); % ���Ž�����
% �����������excel����
xls.write('Results.xlsx',Results)
xls.write('Results_fit.xlsx',Results_fit)