
%��������������һ������ı��ʱ�򣬸������������ִ�����
function location=Update(location,q,p)
% ���룺������
%           �µ��������ڵ�λ��
%           ���������ĳ�������ǰ��
% ������µ�������
location(q)=location(p);
for i=1:q-1
    if location(i)>=location(q)
       location(i)=location(i)+1;
    end
end
end