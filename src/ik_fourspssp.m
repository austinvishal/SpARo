function[A_i1,A_i2,A_i3,A_i4,b1,b2,b3,b4,r1,r2,r3,r4,delta_1,delta_2,delta_3,delta_4,delta_5,e_1,e_2,e_3,e_4,center_plat,nio_Rot_ni1,o_i,xi,yi]=ik_fourspssp(i,L,l1,l2,alpha,beta,lambda,...
    r_o,O)
A_i1=[L(i) -L(i) 0]';
A_i2=[L(i) L(i) 0]';
A_i3=[-L(i) L(i) 0]';
A_i4=[-L(i) -L(i) 0]';
 
B_i1=[l1(i) -l2(i) 0]';
B_i2=[l1(i) l2(i) 0]';
B_i3=[-l1(i) l2(i) 0]';
B_i4=[-l1(i) -l2(i) 0]';
nio_Rot_ni1=Rz(alpha(i))*Ry(beta(i))*Rz(lambda(i))   ;%rotation matrix ZYZ convention Euler
xi=nio_Rot_ni1(1:3,1);
yi=nio_Rot_ni1(1:3,2);
Xi0=r_o(i)*nio_Rot_ni1(1,3);
Yi0=r_o(i)*nio_Rot_ni1(2,3);
Zi0=r_o(i)*nio_Rot_ni1(3,3);
% r_o=Zi0/cos(beta);
o_i=[Xi0 Yi0 Zi0]';
% inverse position kinematics given alpha,beta,gamma,r0- find r1,r2,r3,r4 actuator lengths

b1=nio_Rot_ni1*B_i1+o_i;
b2=nio_Rot_ni1*B_i2+o_i;
b3=nio_Rot_ni1*B_i3+o_i;
b4=nio_Rot_ni1*B_i4+o_i;
center_plat=(b1+b3)/2;
% given pose calculate(alpha , beta, lambda, r_o ) active leg lenths (r1,r2,r3,r4)
r1=norm(b1-A_i1)
r2=norm(b2-A_i2)
r3=norm(b3-A_i3)
r4=norm(b4-A_i4)
% axis prismatic
delta_1=(b1-A_i1)/r1;
delta_2=(b2-A_i2)/r2;
delta_3=(b3-A_i3)/r3;
delta_4=(b4-A_i4)/r4;
delta_5=(o_i-O)/r_o(i); %note this just calculates for base n=1, needs to be modified for stack
e_1=b1-o_i;
  e_2=b2-o_i;
   e_3=b3-o_i;
    e_4=b4-o_i;
end