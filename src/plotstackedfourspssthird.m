function[nio_Rot_ni1a,nio_Rot_ni1b,nio_Rot_ni1c,Rcom,Rcom1,o_i1,o_i2,o_i3]...
    = plotstackedfourspssthird(A_i1a,A_i2a,A_i3a,A_i4a,b1,b2,b3,b4,r1,r2,r3,r4,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat,O,alpha,beta,lambda,o_i,r_o)
%n10 base
% n11 -n20
% n21 n30
% n31 terminal
% 
% n-3
% base n10    up1 n11
% up1base n20 up2 n21
% up2base n30 up3 n31
% 
% n20 and n11 - coplanar fixe, common centre 45 deg. can also be 0 deg
% n30 and n21 45 deg
% 
% main transformation matrix base to terminal is 
% n10Rn11*n11Rn20*n20R21*21nR30*n30Rn31
% n11Rn20- R(45deg) use compsite rot fn k-2
% n21Rn30- R(45deg) k-3
% n10Rn11,n20R21,n30Rn31 depends on their quats or euler or axisangle by fk
% or given ik
nio_Rot_ni1a=Rz(alpha(1))*Ry(beta(1))*Rz(lambda(1))
nio_Rot_ni1b=Rz(alpha(2))*Ry(beta(2))*Rz(lambda(2))
nio_Rot_ni1c=Rz(alpha(3))*Ry(beta(3))*Rz(lambda(3))
Xi0=r_o(1)*nio_Rot_ni1a(1,3);
Yi0=r_o(1)*nio_Rot_ni1a(2,3);
Zi0=r_o(1)*nio_Rot_ni1a(3,3);
Xi02=r_o(2)*nio_Rot_ni1b(1,3);
Yi02=r_o(2)*nio_Rot_ni1b(2,3);
Zi02=r_o(2)*nio_Rot_ni1b(3,3);
Xi03=r_o(3)*nio_Rot_ni1c(1,3);
Yi03=r_o(3)*nio_Rot_ni1c(2,3);
Zi03=r_o(3)*nio_Rot_ni1c(3,3);
% r_o=Zi0/cos(beta);
o_i1=[Xi0 Yi0 Zi0]';
o_i2=[Xi02 Yi02 Zi02]';
o_i3=[Xi03 Yi03 Zi03]';
% o_ia=o_i1+o_i2+o_i3;
% o_ia=o_i1+o_i2+o_i3;
Rcom=eye(3);
Rcom1=eye(3);
%  Rcom= compositerot(2,45); %starts from 2
%  Rcom1= compositerot(3,45);

% A_i1a
% A_i2a
% A_i3a
% A_i4a
% nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*A_i1a
% nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*A_i2a
% nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*A_i3a
% nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*A_i4a
% composite transformations: A1*A2*P3 + A1*P2 +P1 A1,A2 rotation matrices , P1,P2 positions
% four stack: A1*A2*A3*P4+A1*A2*P3 +A1*P2 +P1
A_i1=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*A_i1a+nio_Rot_ni1a*Rcom*o_i2+o_i1;
A_i2=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*A_i2a+nio_Rot_ni1a*Rcom*o_i2+o_i1;
A_i3=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*A_i3a+nio_Rot_ni1a*Rcom*o_i2+o_i1;
A_i4=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*A_i4a+nio_Rot_ni1a*Rcom*o_i2+o_i1;
% A_i1=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*A_i1a+nio_Rot_ni1a*Rcom*o_i1+nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*o_i2
% A_i2=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*A_i2a+nio_Rot_ni1a*Rcom*o_i1+nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*o_i2
% A_i3=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*A_i3a+nio_Rot_ni1a*Rcom*o_i1+nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*o_i2
% A_i4=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*A_i4a+nio_Rot_ni1a*Rcom*o_i1+nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*o_i2
xs_b= [A_i1(1) A_i2(1) A_i3(1) A_i4(1)];% here triangle is formed with three unique vertices, other three points are same here otherwise need to plot polygon
ys_b= [A_i1(2) A_i2(2) A_i3(2) A_i4(2)];
zs_b= [A_i1(3) A_i2(3) A_i3(3) A_i4(3)];
fill3(xs_b,ys_b,zs_b,'r','LineStyle','none')
hold on
% %platform square
b1=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*b1+nio_Rot_ni1a*Rcom*o_i2+o_i1;
b2=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*b2+nio_Rot_ni1a*Rcom*o_i2+o_i1;
b3=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*b3+nio_Rot_ni1a*Rcom*o_i2+o_i1;
b4=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*b4+nio_Rot_ni1a*Rcom*o_i2+o_i1;
% second %b1 nio_Rot_ni1a*Rcom*(nio_Rot_ni1*B_i1+o_i)+o_i1
%third stack %b1 nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*(nio_Rot_ni1*B_i1+o_i)+o_i1+o_i2
% b1=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*b1+nio_Rot_ni1a*Rcom*o_i1+nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*o_i2; %b1 nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*(nio_Rot_ni1*B_i1+o_i)+o_i1+o_i2
% b2=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*b2+nio_Rot_ni1a*Rcom*o_i1+nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*o_i2;
% b3=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*b3+nio_Rot_ni1a*Rcom*o_i1+nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*o_i2;
% b4=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*b4+nio_Rot_ni1a*Rcom*o_i1+nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*o_i2;
% b1=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*b1+o_i1+o_i2; %b1 nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*(nio_Rot_ni1*B_i1+o_i)+o_i1+o_i2
% b2=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*b2+o_i1+o_i2;
% b3=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*b3+o_i1+o_i2;
% b4=nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*b4+o_i1+o_i2;

% b1=nio_Rot_ni1b*Rcom1*nio_Rot_ni1a*Rcom*b1+o_i1+o_i2;
% b2=nio_Rot_ni1b*Rcom1*nio_Rot_ni1a*Rcom*b2+o_i1+o_i2;
% b3=nio_Rot_ni1b*Rcom1*nio_Rot_ni1a*Rcom*b3+o_i1+o_i2;
% b4=nio_Rot_ni1b*Rcom1*nio_Rot_ni1a*Rcom*b4+o_i1+o_i2;
% b1=nio_Rot_ni1b*Rcom1*nio_Rot_ni1a*Rcom*b1+o_i1+o_i2;
% b2=nio_Rot_ni1b*Rcom1*nio_Rot_ni1a*Rcom*b2+o_i1+o_i2;
% b3=nio_Rot_ni1b*Rcom1*nio_Rot_ni1a*Rcom*b3+o_i1+o_i2;
% b4=nio_Rot_ni1b*Rcom1*nio_Rot_ni1a*Rcom*b4+o_i1+o_i2;
% b1=nio_Rot_ni1b*Rcom1*nio_Rot_ni1a*Rcom*b1+o_i1+nio_Rot_ni1b*Rcom*o_i2;
% b2=nio_Rot_ni1b*Rcom1*nio_Rot_ni1a*Rcom*b2+o_i1+nio_Rot_ni1b*Rcom*o_i2;
% b3=nio_Rot_ni1b*Rcom1*nio_Rot_ni1a*Rcom*b3+o_i1+nio_Rot_ni1b*Rcom*o_i2;
% b4=nio_Rot_ni1b*Rcom1*nio_Rot_ni1a*Rcom*b4+o_i1+nio_Rot_ni1b*Rcom*o_i2;
xp_b= [b1(1) b2(1) b3(1) b4(1)];% here triangle is formed with three unique vertices, other three points are same here otherwise need to plot polygon
yp_b= [b1(2) b2(2) b3(2) b4(2)];
zp_b= [b1(3) b2(3) b3(3) b4(3)];
fill3(xp_b,yp_b,zp_b,'r','LineStyle','none')
hold on
 % points
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
 prismatic_joint_axis(0.01,[A_i1(1),A_i1(2),A_i1(3)],[b1(1),b1(2),b1(3)],jointfc,jointec,linkfc);
 hold on 
 prismatic_joint_axis(0.01,[A_i2(1),A_i2(2),A_i2(3)],[b2(1),b2(2),b2(3)],jointfc,jointec,linkfc);
 hold on
 prismatic_joint_axis(0.01,[A_i3(1),A_i3(2),A_i3(3)],[b3(1),b3(2),b3(3)],jointfc,jointec,linkfc);
 hold on
 prismatic_joint_axis(0.01,[A_i4(1),A_i4(2),A_i4(3)],[b4(1),b4(2),b4(3)],jointfc,jointec,linkfc);
 hold on
   center_plata=(b1+b3)/2;
% %  prismatic_joint_axis(0.01,[o(1),o(2),o(3)],[O(1),O(2),O(3)],jointfc,jointec,linkfc)
o_i2j=nio_Rot_ni1a*Rcom*o_i2;
prismatic_joint_axis(0.01,[center_plata(1),center_plata(2),center_plata(3)],[(o_i1(1)'+o_i2j(1)'),(o_i1(2)'+o_i2j(2)'),(o_i1(3)'+o_i2j(3)')],jointfc,jointec,linkfc);

% prismatic_joint_axis(0.01,[center_plata(1),center_plata(2),center_plata(3)],[(o_i1(1)'+o_i2(1)'),(o_i1(2)'+o_i2(2)'),(o_i1(3)'+o_i2(3)')],jointfc,jointec,linkfc);

% prismatic_joint_axis(0.01,[o_ia(1),o_ia(2),o_ia(3)],[center_plat(1),center_plat(2),center_plat(3)],jointfc,jointec,linkfc);
 hold on
  jointfc='k';
%  k1_B=p_os-base_s/norm(p_os-base_s);
sphere_joint_axis(10,delta_1,0.02,eye(3),A_i1',jointfc,jointec);
sphere_joint_axis(10,delta_2,0.02,eye(3),A_i2',jointfc,jointec);
sphere_joint_axis(10,delta_3,0.02,eye(3),A_i3',jointfc,jointec);
sphere_joint_axis(10,delta_4,0.02,eye(3),A_i4',jointfc,jointec);

sphere_joint_axis(10,delta_1,0.02,eye(3),b1',jointfc,jointec);
sphere_joint_axis(10,delta_2,0.02,eye(3),b2',jointfc,jointec);
sphere_joint_axis(10,delta_3,0.02,eye(3),b3',jointfc,jointec);
sphere_joint_axis(10,delta_4,0.02,eye(3),b4',jointfc,jointec);
delta_5=(o_i-o_i2)/r_o(3);
 sphere_joint_axis(10,delta_5,0.02,eye(3),center_plat'+[0,0,0.025],jointfc,jointec);
  sphere_joint_axis(10,delta_5,0.02,eye(3),(o_i1+o_i2j)'+[0,0,0.035],jointfc,jointec);
%   sphere_joint_axis(10,delta_5,0.02,eye(3),(o_i1+o_i2)'+[0,0,0.035],jointfc,jointec);
end