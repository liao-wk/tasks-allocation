%% matlab元组的使用
TW = cell(6,6);
TW{1} = [3,4;4,5];
TW{2,1} = [2,4];
tw = TW{1};
% tw = [tw;[15,6]];
TW{1} = tw;
A = rand(8,2);
A(1:2,1) = inf;
A1 = A;
[a,b] = min(A(:,1));
% A = sort(A);
% inf也可以参与排序
[A,index] = sort(A); % 默认升序排列
