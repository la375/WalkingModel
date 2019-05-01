% find knee from fitting

function [ vector_fit ] = state_fit_plot(xvector, s1,s2,w1,w2,mid1,mid2)

%     x = hip;
%     [A] = state_point_finding( xvector );
%     t1 = A(2);
%     t2 = A(3);
%     t3 = A(4);
%     disp('run state_fit_plot')
    A = sort([mid1, length(xvector)/2, mid2+length(xvector)/2])
    t1 = A(1);
    t2 = A(2);
    t3 = A(3);

    s1_period = xvector(1: t1);
    s2_period = xvector(t1+1: t2);
    w1_period = xvector(t2+1: t3);
    w2_period = xvector(t3+1: end);

    x = s1_period;
    knee_s1 = s1.p1*x.^4 + s1.p2*x.^3 + s1.p3*x.^2 + s1.p4*x + s1.p5;
    x = s2_period;
    knee_s2 = s2.p1*x.^4 + s2.p2*x.^3 + s2.p3*x.^2 + s2.p4*x + s2.p5;
    x = w1_period;
    knee_w1 = w1.p1*x.^4 + w1.p2*x.^3 + w1.p3*x.^2 + w1.p4*x + w1.p5;
    x = w2_period;
    knee_w2 = w2.p1*x.^4 + w2.p2*x.^3 + w2.p3*x.^2 + w2.p4*x + w2.p5;

    vector_fit = [knee_s1;knee_s2;knee_w1;knee_w2];

end
