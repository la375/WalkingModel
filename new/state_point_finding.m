function [A] = state_point_finding(x)

% x = knee;
y = [];
mid = 75;%find( x == min(x));
figure;hold on
A = [0 0 0 0];
% del =abs( x(2)-x(1));
for i = 1: length(x)-1
   
    if i<mid && abs(x(i+1)-x(i))<0.0001
        state = 2;
        if A(2)==0
            A(2) = i;
        end
    elseif i>mid && (x(i+1)-x(i)) > 0.1
        state = 3;
        if A(3)==0
            A(3) = i;
        end
    elseif i>mid && ( x(i+1)-x(i)) < 0.0001
        state = 4;
        if A(4)==0
            A(4) = i;
        end
    else 
        state = 1;
    end
    

    plot(i,state,'o')
end 
% figure
% plot(x(1:end-1),y)
A = sort(A)
end