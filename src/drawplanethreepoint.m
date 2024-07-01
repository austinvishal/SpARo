function[normal,D]= drawplanethreepoint(p1,p2,p3,facecolor,edgecolor,plotoption,scale)
normal=cross(p1-p2,p1-p3);
x=[p1(1) p2(1) p3(1)];
y=[p1(1) p2(1) p3(1)];
z=[p1(1) p2(1) p3(1)];

% coefficients of plane equation
A=normal(1);
B=normal(2);
C=normal(3);
D=-dot(normal,p1);
% scale=2;
 offset=0.01*8;
% offset=0.01*1;
center_plat= [sum(x)/3 sum(y)/3 sum(z)/3];

if C~=0
xLim = [center_plat(1)-offset center_plat(1)+offset];
yLim = [center_plat(2)-offset center_plat(2)+offset];
[Y,X] = meshgrid(yLim,xLim);
Z = (A * X + B * Y + D)/ (-C);
if plotoption==1
mesh(X,Y,Z,'FaceAlpha','0.5','FaceColor',facecolor,'EdgeColor',edgecolor)
end
elseif B~=0
  xLim = [center_plat(1)-0.01*15 center_plat(1)+0.01*15];
  zLim = [center_plat(3)-0.01*15 center_plat(3)+0.01*15];
  [X,Z] = meshgrid(xLim,zLim);
  Y = (A * X + C * Z + D)/ (-B);
if plotoption==1
  mesh(X,Y,Z,'FaceAlpha','0.5','FaceColor',facecolor,'EdgeColor',edgecolor)
end
elseif A~=0
zLim = [center_plat(3)-0.01*15 center_plat(3)+0.01*15];
yLim = [center_plat(2)-0.01*15 center_plat(2)+0.01*15];
[Y,Z] = meshgrid(yLim,zLim);
X = (C * Z + B * Y + D)/ (-A);
if plotoption==1
mesh(X,Y,Z,'FaceAlpha','0.5','FaceColor',facecolor,'EdgeColor',edgecolor)
end
end
%Decide on a suitable showing range
% xLim = [min(x) max(x)];
% zLim = [min(z) max(z)];
% [X,Z] = meshgrid(xLim,zLim);
% Y = (A * X + C * Z + D)/ (-B);
% Order = [1 2  4 3];
% % figure();
% patch(X(Order),Y(Order),Z(Order),'b');

% zLim = [min(z) max(z)];
% yLim = [min(y) max(y)];
% [Y,Z] = meshgrid(yLim,zLim);
% X = (C * Z + B * Y + D)/ (-A);
% reOrder = [1 2  4 3];
% figure();patch(X(reOrder),Y(reOrder),Z(reOrder),'r');
% grid on;
% alpha(0.3);
% scale=2;
% center_plat= [sum(x)/3 sum(y)/3 sum(z)/3];
% xLim = [min(x) max(x)];
% yLim = [min(y) max(y)];
% [Y,X] = meshgrid(yLim,xLim);
% xLim = [center_plat(1)-min(x) center_plat(1)+max(x)];
% yLim = [center_plat(2)-min(y) center_plat(2)+max(y)];
% % xLim = [center_plat(1)-0.01*15 center_plat(1)+0.01*15];
% % yLim = [center_plat(2)-0.01*15 center_plat(2)+0.01*15];
% % [Y,X] = meshgrid(yLim,xLim);
% % % Z = (A * (X-center_plat(1)) + B * (Y-center_plat(2)) + D)/ (-C);
% % Z = (A * X + B * Y + D)/ (-C);
% % reOrder = [1 2  4 3];
% % % figure();
% % %  patch(X(reOrder),Y(reOrder),Z(reOrder),'FaceAlpha','0.5');
% %  mesh(X,Y,Z,'FaceAlpha','0.5')
% surf(X(reOrder),Y(reOrder),Z(reOrder),'FaceColor','r','Facealpha',0.6, ...
% 'EdgeColor','none','AmbientStrength',0.6)

% patch(X(reOrder),Y(reOrder),Z(reOrder),'r');
% grid on;
% alpha(0.3);
% grid on;
% alpha(0.3);
% syms x y z real
% P = [x,y,z];
% realdot(normal,P-p1) %n.(r-b)=0 n=<A,B,C>; b=<a,b,c> <A,B,C>.(x-a,y-b,z-c)
end