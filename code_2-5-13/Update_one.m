
%��������������һ������ı��ʱ�򣬸������������ִ�����
function location=Update_one(location,q)
% ���룺������
%           �µĵ�һ���������ڵ�λ��
% ������µ�������
location(q)=1;
for i=1:q-1
    location(i)=location(i)+1;
end
end
