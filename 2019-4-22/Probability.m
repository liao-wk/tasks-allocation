%% �Ӻ���8���������������ʱ�䴰��ͻ�ĸ���
function prob = Probability(tw,tw_obj,dur1,dur2)
if tw(2)-tw(1)+tw_obj(2)-tw_obj(1)-(max([tw(2),tw_obj(2)])-min([tw(1),tw_obj(1)])) > 0
    tw(2) = tw(2) - dur1;
    tw_obj(2) = tw_obj(2) - dur2;
    % tw��tw_obj����Чʱ�䴰��dur1��dur2�ֱ���tw��tw_obj��Ӧ�ĳ���ʱ�䡣
    % ������ô���룺Probability_tasks(tw_obj,tw,dur2,dur1)
    delta1= tw(2)-tw_obj(2); 
    delta2 = tw(1)-tw_obj(1);
    S0 = abs(tw(2)-tw(1))*abs(tw_obj(2)-tw_obj(1)); % ���ε����
    % ����������Ӱ���ֵ����
    % ���С�ڵ���0��˵����һ��ʱ�䴰����һ��ʱ�䴰��,��������ʱ�䴰�ص�һ����
    if delta1*delta2 <= 0
        if delta1 > 0 
            T12 = tw_obj(2);
            T11 = tw_obj(1);
            T22 = tw(2);
            T21 = tw(1);
            d1 = dur2;
            d2 = dur1;
        else
            T12 = tw(2);
            T11 = tw(1);
            T22 = tw_obj(2);
            T21 = tw_obj(1);
            d1 = dur1; 
            d2 = dur2;
        end
        if T21+d2 > T11 && T21+d2 < T12 % �γ������Σ��������߽�
            S2 = 0.5*(T12-T21-d2)*(T12-d2-T21);
        elseif T21+d2 <= T11
            S2 = 0.5*(T11-2*d2-2*T21+T12)*(T12-T11);
        elseif T21+d2 > T11 && T21+d2 >= T12 % �����߽�
            S2 = 0;
        end
        if T22-d1 < T12  && T11+d1 < T22 % �γ������Σ��������߽硣
            S1 = 0.5*(T22-T11-d1)*(T22-d1-T11);
        elseif T22-d1 >= T12  % �γ�ֱ������
            S1 = 0.5*(2*T22-T12-T11-2*d1)*(T12-T11);
        elseif T22-d1 < T12 && T11+d1 >= T22
            S1 = 0;
        end
    else
        if delta1 > 0
            T12 = tw(2);
            T11 = tw(1);
            T22 = tw_obj(2);
            T21 = tw_obj(1);
            d1 = dur1;
            d2 = dur2;
        else
            T12 = tw_obj(2);
            T11 = tw_obj(1);
            T22 = tw(2);
            T21 = tw(1);
            d1 = dur2;
            d2 = dur1;
        end
        if T11+d1 < T22
            S1 = 0.5*(T22-T11-d1)*(T22-d1-T11);
        else
            S1 = 0;
        end
        if T11-d2 > T21
            S2 = 0.5*(T12-T11)*(T22-2*T21+T11-d2);
        else
            if d2+T22 < T12
                S2 = 0.5*(2*T12-2*d2-T22-T21)*(T22-T21);
            elseif d2+T22 >= T12 && T21+d2 < T12
                S2 = 0.5*(T12-d2-T21)*(T12-d2-T21);
            elseif d2+T22 >= T12 && T21+d2 >= T12
                S2 = 0;
            end
        end
    end
    prob = (S0-S1-S2)/S0; % ������ʱ�䴰֮��ĳ�ͻ��������
else
    prob = 0;
end
