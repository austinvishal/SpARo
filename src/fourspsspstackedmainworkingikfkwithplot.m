%% kinematics of n- (4 sps+sp)
clc 
clear all
close all
% parameter notation
%n - no. of identical 4-sps+sp PKMs
%i- the PKM entity unit -from bottom to top
%n_i1- upper platform
%n_i0- lower platform
% r_ij - active limbs (here four of them) i= 1...n, j=1..4
% r_io- passive SP limb length
% l_i1- length of n_i1 rectangle
%l_i2- width of n_i1 rectangle
% L_i- side length of n_i0 square
% B_ij- coordinates of spherical joint at n_i1 upper platform
% A_ij- coordinates of spherical joint at n_io base platform
%o_i-1- coordinate of passive spherical joint on n_io
% theta= angle between n_(i-1)i and n_i0
%n_10- base platform
%n_n0- terminal
n=3; % no. of platforms is 3
global sa ca
% syms beta lambda real
alpha= deg2rad(21.8014);
beta=deg2rad(12.4736);
lambda= deg2rad(-22.8961);
% alpha= deg2rad(19.9651);
% beta=deg2rad(18.3838);
% lambda= deg2rad(-35.4613);
L=[0.5 0.4257 0.3758];
l1=[0.45 0.4 0.35];
l2=[0.4 0.35 0.3];
%  r_o=0.7270; %0.7270
  r_o=0.7108/cos(beta); % Z_0/cos(beta)  in meters
 O=[0 0 0]'; % base center origin
%
[A_i1,A_i2,A_i3,A_i4,b1,b2,b3,b4,r1,r2,r3,r4,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat]=ik_fourspssp(1,L,l1,l2,alpha,beta,lambda,r_o,O)
% A_i1=[L(1) -L(1) 0]';
% A_i2=[L(1) L(1) 0]';
% A_i3=[-L(1) L(1) 0]';
% A_i4=[-L(1) -L(1) 0]';
%  
% B_i1=[l1(1) -l2(1) 0]';
% B_i2=[l1(1) l2(1) 0]';
% B_i3=[-l1(1) l2(1) 0]';
% B_i4=[-l1(1) -l2(1) 0]';
% nio_Rot_ni1=Rz(alpha)*Ry(beta)*Rz(lambda)   ;%rotation matrix ZYZ convention Euler
% Xi0=r_o*nio_Rot_ni1(1,3);
% Yi0=r_o*nio_Rot_ni1(2,3);
% Zi0=r_o*nio_Rot_ni1(3,3);
% % r_o=Zi0/cos(beta);
% o_i=[Xi0 Yi0 Zi0]';
% % inverse position kinematics given alpha,beta,gamma,r0- find r1,r2,r3,r4 actuator lengths
% 
% b1=nio_Rot_ni1*B_i1+o_i;
% b2=nio_Rot_ni1*B_i2+o_i;
% b3=nio_Rot_ni1*B_i3+o_i;
% b4=nio_Rot_ni1*B_i4+o_i;
% center_plat=(b1+b3)/2;
% % given pose calculate(alpha , beta, lambda, r_o ) active leg lenths (r1,r2,r3,r4)
% r1=norm(b1-A_i1)
% r2=norm(b2-A_i2)
% r3=norm(b3-A_i3)
% r4=norm(b4-A_i4)
% % axis prismatic
% delta_1=(b1-A_i1)/r1;
% delta_2=(b2-A_i2)/r2;
% delta_3=(b3-A_i3)/r3;
% delta_4=(b4-A_i4)/r4;
% delta_5=(o_i-O)/r_o;
% e_1=b1-o_i;
%   e_2=b2-o_i;
%    e_3=b3-o_i;
%     e_4=b4-o_i;
%% plotting

%base square
xs_b= [A_i1(1) A_i2(1) A_i3(1) A_i4(1)];% here triangle is formed with three unique vertices, other three points are same here otherwise need to plot polygon
ys_b= [A_i1(2) A_i2(2) A_i3(2) A_i4(2)];
zs_b= [A_i1(3) A_i2(3) A_i3(3) A_i4(3)];
fill3(xs_b,ys_b,zs_b,'r','LineStyle','none')
hold on
%platform square
xp_b= [b1(1) b2(1) b3(1) b4(1)];% here triangle is formed with three unique vertices, other three points are same here otherwise need to plot polygon
yp_b= [b1(2) b2(2) b3(2) b4(2)];
zp_b= [b1(3) b2(3) b3(3) b4(3)];
fill3(xp_b,yp_b,zp_b,'r','LineStyle','none')
hold on
 %% points
 plot3(A_i1(1),A_i1(2),A_i1(3),'o','MarkerFaceColor','k')
 hold on
 plot3(A_i2(1),A_i2(2),A_i2(3),'o','MarkerFaceColor','k')
 hold on
 plot3(A_i3(1),A_i3(2),A_i3(3),'o','MarkerFaceColor','k')
 hold on
 plot3(A_i4(1),A_i4(2),A_i4(3),'o','MarkerFaceColor','k')
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
 prismatic_joint_axis(0.01,[A_i1(1),A_i1(2),A_i1(3)],[b1(1),b1(2),b1(3)],jointfc,jointec,linkfc)
 hold on 
 prismatic_joint_axis(0.01,[A_i2(1),A_i2(2),A_i2(3)],[b2(1),b2(2),b2(3)],jointfc,jointec,linkfc)
 hold on
 prismatic_joint_axis(0.01,[A_i3(1),A_i3(2),A_i3(3)],[b3(1),b3(2),b3(3)],jointfc,jointec,linkfc)
 hold on
 prismatic_joint_axis(0.01,[A_i4(1),A_i4(2),A_i4(3)],[b4(1),b4(2),b4(3)],jointfc,jointec,linkfc)
 hold on
 
%  prismatic_joint_axis(0.01,[o(1),o(2),o(3)],[O(1),O(2),O(3)],jointfc,jointec,linkfc)
prismatic_joint_axis(0.01,[center_plat(1),center_plat(2),center_plat(3)],[O(1),O(2),O(3)],jointfc,jointec,linkfc)
 hold on
 jointfc='k';
%  k1_B=p_os-base_s/norm(p_os-base_s);
sphere_joint_axis(10,delta_1,0.02,eye(3),A_i1',jointfc,jointec)
sphere_joint_axis(10,delta_2,0.02,eye(3),A_i2',jointfc,jointec)
sphere_joint_axis(10,delta_3,0.02,eye(3),A_i3',jointfc,jointec)
sphere_joint_axis(10,delta_4,0.02,eye(3),A_i4',jointfc,jointec)

sphere_joint_axis(10,delta_1,0.02,eye(3),b1',jointfc,jointec)
sphere_joint_axis(10,delta_2,0.02,eye(3),b2',jointfc,jointec)
sphere_joint_axis(10,delta_3,0.02,eye(3),b3',jointfc,jointec)
sphere_joint_axis(10,delta_4,0.02,eye(3),b4',jointfc,jointec)
 sphere_joint_axis(10,delta_5,0.02,eye(3),O'+[0,0,0.015],jointfc,jointec)
view(10.126508917753073,39.679461181786593)
box on
  drawnow
  pause(0.1)
  
%  hold off



%% Forward kinematics- sylvester dialytic elimination theory
% clc
% syms beta real
% % given r1,r2,r3,r4 find alpha,beta,gamma,r0
% r1=0.70;
% r2=0.66;
% r3=0.76;
% r4=0.80;
% L=[0.5 0.4257 0.3758];
% l1=[0.45 0.4 0.35];
% l2=[0.4 0.35 0.3];
% Q1=(2*L(1)^2+l1(1)^2+l2(1)^2)/4
% Xi0=(r3^2+r4^2-r1^2-r2^2)/(4*L(1));
% Yi0=(r1^2+r4^2-r2^2-r3^2)/(4*L(1));
% fn1=(r1^2+r4^2-r2^2-r3^2)/(r3^2+r4^2-r1^2-r2^2);
% alpha= atan(fn1); 
% % once alpha is determined, we use the elimination approach, Weirstrass
% % substitution to change it to polynomial and obtain beta and lambda
% % first ti1 is solved, then ti2, beta, lambda then Zi0
% t1=tan(beta/2); t2=tan(lambda/2);
% sa=sin(alpha);
% ca=cos(alpha);
% s15=l1(1)*sa+l2(1)*sa-((r1^2-r2^2+r3^2-r4^2)/(2*L(1)));
% s14=2*l1(1)*ca+2*l2(1)*ca;
% s13=-l1(1)*sa-l2(1)*sa-((r1^2-r2^2+r3^2-r4^2)/(2*L(1)));
% s12=-l1(1)*sa+l2(1)*sa-((r1^2-r2^2+r3^2-r4^2)/(2*L(1)));
% s11=2*l1(1)*ca-2*l2(1)*ca;
% s10=l1(1)*sa-l2(1)*sa-((r1^2-r2^2+r3^2-r4^2)/(2*L(1)));
% % next set of coefficients
% s29=-4*(Xi0/ca)^2;
% s28=-16*Q1-12*(Xi0/ca)^2+4*(r1^2+r2^2+r3^2+r4^2)+8*ca*L(1)*(l1(1)-l2(1));
% s27=-4*(Xi0/ca)^2;
% s26=-16*sa*L(1)*(l1(1)-l2(1));
% s25=-16*Q1-12*(Xi0/ca)^2+4*(r1^2+r2^2+r3^2+r4^2)-8*ca*L(1)*(l1(1)+l2(1));
% s24=-16*Q1-12*(Xi0/ca)^2+4*(r1^2+r2^2+r3^2+r4^2)+8*ca*L(1)*(-l1(1)+l2(1));
% s23=-16*sa*(l1(1)+l2(1));
% s22=-16*Q1-12*(Xi0/ca)^2+4*(r1^2+r2^2+r3^2+r4^2)+8*ca*L(1)*(l1(1)+l2(1));
% s21=-4*(Xi0/ca)^2;
% s20=-4*(Xi0/ca)^2;
% p12=s15*t1^2+s12;
% p11=s14*t1^2+s11;
% p10=s13*t1^2+s10;
% p22=s29*t1^6+s28*t1^4+s25*t1^2+s21;
% p21=s26*t1^4+s23*t1^2;
% p20=s27*t1^6+s24*t1^4+s22*t1^2+s20;
% D=  [p12 p11 p10 0
%     p22 p21 p20 0
%     0 p12 p11 p10
%     0 p22 p21 p20];
% fun = @(t1)D;
% % xval = fzero(@(t1)det(fun(t1)),0)
% % det(D)
% % r0=sqrt(Xi0^2+Yi0^2+Zi0^2);
% %
% %%
% syms t1 t2 real
% eqn=(2866672816455297203002330607399549553136750304773462608137281395*t1^2)/381757057298519801779952850353305317155994720826990528296877293568 + (9936869772332903782546115136290726575500408072018613883110375505971*t1^4)/76351411459703960355990570070661063431198944165398105659375458713600 - (3690542461931680987855349946364741476583672144864047607041738228709*t1^6)/2385981608115748761124705314708158232224967005168690801855483084800 - (319733162043845991350446416154694909808542056567894374498110961622293*t1^8)/4771963216231497522249410629416316464449934010337381603710966169600 - (5473208968457916466766371115634128330583931789742237727123540533685197*t1^10)/38175705729851980177995285035330531715599472082699052829687729356800 - (306277465985739438311394234015191623940207295272200860071427439827661*t1^12)/4771963216231497522249410629416316464449934010337381603710966169600 + (1052334826346104857631209384644546294914426518850612260721205370835*t1^14)/381757057298519801779952850353305317155994720826990528296877293568 - (86342036584993898977964296750977294610510111918531052682918919125*t1^16)/3054056458388158414239622802826442537247957766615924226375018348544 - 18672585766650930615555784275982199830038781210296372835669125/190878528649259900889976425176652658577997360413495264148438646784
%  %  S =solve(eqn,t1,'Real',true)
%   t1_a = vpasolve(eqn)
% t1=subs(t1,t1_a(5))
% angle_beta=vpa(rad2deg(2*atan(t1_a(5)))) % beta value in degrees
% eqn2=(s15*subs(t1)^2+s12)*t2^2+(s14*subs(t1)^2+s11)*t2+s13*subs(t1)^2+s10;
% %   eqn2=p12*t2^2+p11*t2+p10
%   t2_a = vpasolve(eqn2)
%   angle_lambda=vpa(rad2deg(2*atan(t2_a(1))))
%   Z_o=vpa(0.7270*cosd(angle_beta))