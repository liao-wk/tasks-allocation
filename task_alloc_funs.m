function funs = task_alloc_funs
%UNTITLED6 此处显示有关此函数的摘要
%   此处显示详细说明
funs.distance_value = @distance_value;
end
%% 子函数1，根据经纬度求任务点到基地的距离
function distance = distance_value(base1,t_pos)
r = 6371;
distance = 2*r*asin(sqrt(sin(0.5*(base1(1)-t_pos(1))).^2+cos(base1(1))*...
    cos(t_pos(1))*sin(0.5*(base1(2)-t_pos(2))).^2));
end


