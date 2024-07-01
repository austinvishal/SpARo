function plotlineparallel(p1,p2,p3)
%this function plots line passing through p3 and parallel to line made by
%p1,p2

slope= (p2(2)-p1(2))/(p2(1)-p1(1));
x = 0 : 0.01:0.1;
y=slope*(x-p3(1))+p3(2); %y - y3 = slope * (x - x3)

plot(x, y)
end