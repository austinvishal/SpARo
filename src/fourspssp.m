syms alpha beta lambda r_o real

%     set(gcf, 'color', 'white','units','pixels','position',[0 0 1920 1080]);


% alpha=0;
% beta=0;
alpha=pi/3;
lambda=0;
r_o=1.945; % in meters
% r_o=3;
a= 1.25; % a is distance from center to base coordinates
l1=1.25; l3=1.25;% 
l2=1; l4=1;  % l1,l3 and l2,l4 rectangular lengths of platform
% Bi=[X_Bi Y_Bi Z_Bi]'; %base coodinate wrt to base frame
B1=[-1.25;0;0];
B2=[0;1.25;0];
B3=[1.25;0;0];
B4=[0;-1.25;0];
m_b1=[-0.795;-0.088;1.945];
m_b2=[0.088;0.795;1.945];
m_b3=[0.795;0.088;1.945];
m_b4=[-0.088;-0.795;1.945];
% m_bi=[x_bi y_bi z_bi]'; % wrt to moving frame
% bi=[X_bi Y_bi Z_bi]';
O=[0 0 0]'; % base center origin
% o=[X_o Y_o Z_o]'; % PLATFORM center point
%% video
% % vidObj=VideoWriter('4spssp1.mp4');
% % vidObj.FrameRate = 24;
% % open(vidObj);
% t_ini=1.5;
% % t_fin=2.6;
% t_fin=6.6;
% R=2;
% for t1=t_ini:0.1:t_fin
%     alpha=R*cos(0.5*t1);
%     beta=R*sin(0.5*t1);
for beta=-pi/3:0.1:pi/3
    for r_o=1.945:2.1  
%%
B_Rot_m=Rz(alpha)*Ry(beta)*Rz(lambda)   ;%rotation matrix ZYZ convention Euler
o=r_o*B_Rot_m(:,3);
% bi=B_Rot_m*m_bi+o ;%bi vector at the platform, lower case is platform
% four pose paramters alpha , beta, lambda, r_o (as central prismatic length variable is r_o)
b1=B_Rot_m*m_b1+o;
b2=B_Rot_m*m_b2+o;
b3=B_Rot_m*m_b3+o;
b4=B_Rot_m*m_b4+o;
center_plat=(b1+b3)/2;
%% inversekinematics
% given pose calculate(alpha , beta, lambda, r_o ) active leg lenths (r1,r2,r3,r4)
r1=norm(b1-B1);
r2=norm(b2-B2);
r3=norm(b3-B3);
r4=norm(b4-B4);
delta_1=(b1-B1)/r1;
delta_2=(b2-B2)/r2;
delta_3=(b3-B3)/r3;
delta_4=(b4-B4)/r4;
delta_5=(o-O)/r_o;
 e_1=b1-o;
  e_2=b2-o;
   e_3=b3-o;
    e_4=b4-o;
%% general equations
% r_i= norm(bi-Bi);
% delta_i=(bi-Bi)/r_i ; %unitvector active 4 prismatic legs
% e_i= bi-o;

%% plotting

%base square
xs_b= [B1(1) B2(1) B3(1) B4(1)];% here triangle is formed with three unique vertices, other three points are same here otherwise need to plot polygon
ys_b= [B1(2) B2(2) B3(2) B4(2)];
zs_b= [B1(3) B2(3) B3(3) B4(3)];
fill3(xs_b,ys_b,zs_b,'r','LineStyle','none')
hold on
%platform square
xp_b= [b1(1) b2(1) b3(1) b4(1)];% here triangle is formed with three unique vertices, other three points are same here otherwise need to plot polygon
yp_b= [b1(2) b2(2) b3(2) b4(2)];
zp_b= [b1(3) b2(3) b3(3) b4(3)];
fill3(xp_b,yp_b,zp_b,'r','LineStyle','none')
hold on
 %% points
 plot3(B1(1),B1(2),B1(3),'o','MarkerFaceColor','k')
 hold on
 plot3(B2(1),B2(2),B2(3),'o','MarkerFaceColor','k')
 hold on
 plot3(B3(1),B3(2),B3(3),'o','MarkerFaceColor','k')
 hold on
 plot3(B4(1),B4(2),B4(3),'o','MarkerFaceColor','k')
 hold on
 
  plot3(b1(1),b1(2),b1(3),'o','MarkerFaceColor','k')
 hold on
 plot3(b2(1),b2(2),b2(3),'o','MarkerFaceColor','k')
 hold on
 plot3(b3(1),b3(2),b3(3),'o','MarkerFaceColor','k')
 hold on
 plot3(b4(1),b4(2),b4(3),'o','MarkerFaceColor','k')
 hold on
 jointfc='r';
 jointec='k';
 linkfc='r';
 %% plot joints
 prismatic_joint_axis(0.01,[B1(1),B1(2),B1(3)],[b1(1),b1(2),b1(3)],jointfc,jointec,linkfc)
 hold on 
 prismatic_joint_axis(0.01,[B2(1),B2(2),B2(3)],[b2(1),b2(2),b2(3)],jointfc,jointec,linkfc)
 hold on
 prismatic_joint_axis(0.01,[B3(1),B3(2),B3(3)],[b3(1),b3(2),b3(3)],jointfc,jointec,linkfc)
 hold on
 prismatic_joint_axis(0.01,[B4(1),B4(2),B4(3)],[b4(1),b4(2),b4(3)],jointfc,jointec,linkfc)
 hold on
 
%  prismatic_joint_axis(0.01,[o(1),o(2),o(3)],[O(1),O(2),O(3)],jointfc,jointec,linkfc)
prismatic_joint_axis(0.01,[center_plat(1),center_plat(2),center_plat(3)],[O(1),O(2),O(3)],jointfc,jointec,linkfc)
 hold on
 jointfc='k';
%  k1_B=p_os-base_s/norm(p_os-base_s);
sphere_joint_axis(10,delta_1,0.05,eye(3),B1',jointfc,jointec)
sphere_joint_axis(10,delta_2,0.05,eye(3),B2',jointfc,jointec)
sphere_joint_axis(10,delta_3,0.05,eye(3),B3',jointfc,jointec)
sphere_joint_axis(10,delta_4,0.05,eye(3),B4',jointfc,jointec)

sphere_joint_axis(10,delta_1,0.05,eye(3),b1',jointfc,jointec)
sphere_joint_axis(10,delta_2,0.05,eye(3),b2',jointfc,jointec)
sphere_joint_axis(10,delta_3,0.05,eye(3),b3',jointfc,jointec)
sphere_joint_axis(10,delta_4,0.05,eye(3),b4',jointfc,jointec)
sphere_joint_axis(10,delta_5,0.05,eye(3),O'+[0,0,0.045],jointfc,jointec)
view(10.126508917753073,39.679461181786593)
box on
  drawnow
  pause(0.1)
  
 hold off
%     currFrame= getframe(gcf);
%            writeVideo(vidObj, currFrame);
%     end
    end
end
%      close(gcf)
%  close(vidObj);