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
% adjust parameters
dt = 0.001;
Npoints = 20;
threshold = 0.05;

global sa ca
% syms beta lambda real
% alpha= deg2rad(21.8014);
% beta=deg2rad(12.4736);
% lambda= deg2rad(-22.8961);
% alpha= deg2rad(19.9651);
% beta=deg2rad(18.3838);
% lambda= deg2rad(-35.4613);
%    set(gcf, 'color', 'white','units','pixels','position',[0 0 1920 1080]);
%    vidObj=VideoWriter('doublestackedspsssp.mp4');
% vidObj.FrameRate = 24;
% open(vidObj);
% for a1a=11.8014
%  for a1a=11.8014:41.8014
%     for a2a=18.3838
% alpha=deg2rad([0 0 75.6454]);
% beta=deg2rad([12.4736  0 22.3011 ]);
% lambda=deg2rad([0 0 0]);
% alpha=deg2rad([21.8014 19.9651 75.6454]);
% beta=deg2rad([12.4736 18.3838 22.3011]);
% lambda=deg2rad([-22.8961 -35.4613 -40.2939]);
% alpha=deg2rad([21.8014 19.9651 75.6454]);
% beta=deg2rad([12.4736 18.3838 22.3011]);
% lambda=deg2rad([-22.8961 -35.4613 -40.2939]);
alpha=deg2rad([0 0 0]);
beta=deg2rad([0 0 0]);
lambda=deg2rad([0 0 0]);
% alpha=deg2rad([21.8014 19.9651 75.6454]);
% beta=deg2rad([12.4736 18.3838 22.3011]);
% lambda=deg2rad([-22.8961 -35.4613 -40.2939]);
L=[0.5 0.4257 0.3758];
l1=[0.45 0.4 0.35];
l2=[0.4 0.35 0.3];
r_o=[0.7270 0.6571 0.6809];
%  r_o=0.7270; %0.7270
%  r_o=0.7108/cos(beta); % Z_0/cos(beta)  in meters
 O=[0 0 0]'; % base center origin
%% trajectory
% Line
% X_start = [-0.5;0.5;6]; X_end = [2.5;3.5;7];
 X_start = [-1.5;-0.5;1.9]; X_end = [2.5;1.5;1.9];
% X_start = [-1.5;-0.5;1.5]; X_end = [2.5;1.5;1.5];
[t, Xd, Thd]= DesiredPath(X_start,X_end,Npoints);
plot3(Xd(1,:),Xd(2,:),Xd(3,:),'*r','LineWidth',2); grid on; hold on; 
% drawnow;
 Posed = [Xd ; Thd];
 % Main Lopp
RMSE = [];
Error = [];
f = 0; time = 0; iteration = 0;
Q = [];

DESIRED_position = [];
TRACKED_position = [];
ERROR_position = [];
normErrorPosition = [];

%  for k=1:size(Xd,2)
% for k=1:1  % here for second iteration the alpha,beta gamma vectors and position must be update
%% IK and jacobian
%      posed = Posed(:,k);
%     error = disk(end).pose(1:3)-posed(1:3);
%     RMS = rms(disk(end).pose-posed);

 [A_11,A_12,A_13,A_14,b1a,b2a,b3a,b4a,r1a,r2a,r3a,r4a,delta_1,delta_2,delta_3,delta_4,delta_5,e_1,e_2,e_3,e_4,center_plat,nio_Rot_ni1,o_i,xi,yi]=ik_fourspssp(1,L,l1,l2,alpha,beta,lambda,r_o,O)
 J1=Jacostackedfourspss(delta_1,delta_2,delta_3,delta_4,e_1,e_2,e_3,e_4,xi,yi,o_i,A_11,A_12)
  plotstackedfourspss(A_11,A_12,A_13,A_14,b1a,b2a,b3a,b4a,r1a,r2a,r3a,r4a,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat,O,alpha,beta,lambda,o_i,r_o)
 hold on
[A_21,A_22,A_23,A_24,b1b,b2b,b3b,b4b,r1b,r2b,r3b,r4b,delta_1,delta_2,delta_3,delta_4,delta_5,e_1,e_2,e_3,e_4,center_plat,nio_Rot_ni1,o_i,xi,yi]=ik_fourspssp(2,L,l1,l2,alpha,beta,lambda,r_o,O)
J2=Jacostackedfourspss(delta_1,delta_2,delta_3,delta_4,e_1,e_2,e_3,e_4,xi,yi,o_i,A_21,A_22)
  center_plat1=plotstackedfourspsssecond(A_21,A_22,A_23,A_24,b1b,b2b,b3b,b4b,r1b,r2b,r3b,r4b,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat,O,alpha,beta,lambda,o_i,r_o)
 hold on
 [A_31,A_32,A_33,A_34,b1c,b2c,b3c,b4c,r1c,r2c,r3c,r4c,delta_1,delta_2,delta_3,delta_4,delta_5,e_1,e_2,e_3,e_4,center_plat,nio_Rot_ni1,o_i,xi,yi]=ik_fourspssp(3,L,l1,l2,alpha,beta,lambda,r_o,O)
 J3=Jacostackedfourspss(delta_1,delta_2,delta_3,delta_4,e_1,e_2,e_3,e_4,xi,yi,o_i,A_31,A_32)
 [nio_Rot_ni1a,nio_Rot_ni1b,nio_Rot_ni1c,Rcom,Rcom1,o_i1,o_i2,o_i3]= plotstackedfourspssthird(A_31,A_32,A_33,A_34,b1c,b2c,b3c,b4c,r1c,r2c,r3c,r4c,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat1,O,alpha,beta,lambda,o_i,r_o)
hold off
% figure
[alpha1, beta1, gamma1] = Rot2Angleszyz(nio_Rot_ni1a)
[alpha2, beta2, gamma2] = Rot2Angleszyz(nio_Rot_ni1b)
[alpha, beta, gamma] = Rot2Angleszyz(nio_Rot_ni1c)
%to verify the angles with given input, uncomment the three lines given
%below
% rad2deg(alpha)
% rad2deg(beta)
% rad2deg(gamma)
pose_first=[o_i1;[alpha1;beta1;gamma1]]
pose_second=[o_i2;[alpha2;beta2;gamma2]]
pose_third=[o_i3;[alpha;beta;gamma]]
%  hold on
% [A_i1,A_i2,A_i3,A_i4,b1,b2,b3,b4,r1,r2,r3,r4,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat,nio_Rot_ni1,o_i]=ik_fourspssp(3,L,l1,l2,alpha,beta,lambda,r_o,O)
 Q=[Q;r1a;r2a;r3a;r4a;0;0;r1b;r2b;r3b;r4b;0;0;r1c;r2c;r3c;r4c;0;0;]
%forward Jacobian map
% note here we use the adjoint map, but in decoupled form as in augmented
% rotation and adj spm multipled together, which is infact same as adjoint
% map
JR1=augmentedrotation(eye(3))*Adjointserialparallel(nio_Rot_ni1a*Rcom*o_i2); %confirm once again
JR2=augmentedrotation(nio_Rot_ni1a*Rcom)*Adjointserialparallel(nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*o_i3);
JR3=augmentedrotation(nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1); %just the augmented rotation
J1robot=JR1*inv(J1);
J2robot=JR2*inv(J2);
J3robot=JR3*inv(J3);
Jrobot=[J1robot ;J2robot ;J3robot]
%% plotting
  for k=1:size(Xd,2)
posed = Posed(:,k);
%      posed = Posed(:,k);
    error = pose_third(1:3)-posed(1:3);
     RMS = rms(pose_third-posed);
while (RMS > threshold)  
plot3(Xd(1,:),Xd(2,:),Xd(3,:),'*r','LineWidth',2); hold on; grid on;
        xlabel('x');ylabel('y');zlabel('z');
        
 % pose control
   dQAll = JacInvgeomControl(Jrobot,pose_third,posed,Q,dt) % global coordinates
   % update pose qc=qc+dq % Khalil book pg 134
%    DP = zeros(3,1); DR = eye(3);
   DP1 = zeros(3,1); DR1 = eye(3);
   DP2 = zeros(3,1); DR2 = eye(3);
   DP3 = zeros(3,1); DR3 = eye(3);
   N=3;
%    for c = 2:N+1
%      dX=pinv(Jrobot(6*(c-2)+1:6*(c-1),1:6))*dQAll(6*(c-2)+1:6*(c-1)) %foreach module
%     dX=pinv(Jrobot*dQAll) %foreach module
     dX1=pinv(Jrobot(6*(2-2)+1:6*(2-1),1:6))*dQAll(6*(2-2)+1:6*(2-1)) %foreach module
      dX2=pinv(Jrobot(6*(3-2)+1:6*(3-1),1:6))*dQAll(6*(3-2)+1:6*(3-1))
       dX3=pinv(Jrobot(6*(4-2)+1:6*(4-1),1:6))*dQAll(6*(4-2)+1:6*(4-1))
%       dX1=pinv(JR1)*dQAll(6*(2-2)+1:6*(2-1)) %foreach module
%       dX2=pinv(JR2)*dQAll(6*(3-2)+1:6*(3-1))
%       dX3=pinv(JR3)*dQAll(6*(4-2)+1:6*(4-1))
       
%        
    % update position
%     dp=dX(1:3,:);
%     DP = DP + dp;
    dp1=dX1(1:3,:);
    DP1 = DP1 + dp1;
    dp2=dX2(1:3,:);
    DP2 = DP2 + dp2;
    dp3=dX3(1:3,:);
    DP3 = DP3 + dp3;
    o_i1=o_i1+DP1;
    o_i2=o_i2+DP2;
    o_i3=o_i3+DP3;
    
    % update rotation
%     dth = dX(4:6);
%     dR = angle2rot(dth(1),dth(2),dth(3));
%     DR = DR*dR;
    dth1 = dX1(4:6);
    dR1 = angle2rot(dth1(1),dth1(2),dth1(3));
    DR1 = DR1*dR1;
    dth2 = dX2(4:6);
    dR2 = angle2rot(dth2(1),dth2(2),dth2(3));
    DR2 = DR2*dR2;
    dth3 = dX3(4:6);
    dR3 = angle2rot(dth3(1),dth3(2),dth3(3));
    DR3 = DR3*dR3;
    nio_Rot_ni1a = DR1*nio_Rot_ni1a;
    nio_Rot_ni1b = DR2*nio_Rot_ni1b;
    nio_Rot_ni1c = DR3*nio_Rot_ni1c;
    
     % update the base vector coordinates
       
            
    % update pose
            [alph1,bet1,gamm1] = Rot2Angleszyz(nio_Rot_ni1a);
            [alph2,bet2,gamm2] = Rot2Angleszyz(nio_Rot_ni1b);
            [alph3,bet3,gamm3] = Rot2Angleszyz(nio_Rot_ni1c);
            alpha=[alph1 alph2 alph3]
            beta=[bet1 bet2 bet3]
            lambda=[gamm1 gamm2 gamm3]
            pose_first = [o_i1 ; [alph1;bet1;gamm1] ]
            pose_second = [o_i2 ; [alph2;bet2;gamm2] ]
            pose_third = [o_i3 ; [alph3;bet3;gamm3] ]
%    end
JR1=augmentedrotation(eye(3))*Adjointserialparallel(nio_Rot_ni1a*Rcom*o_i2); %confirm once again
JR2=augmentedrotation(nio_Rot_ni1a*Rcom)*Adjointserialparallel(nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*o_i3);
JR3=augmentedrotation(nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1); %just the augmented rotation
J1robot=JR1*inv(J1);
J2robot=JR2*inv(J2);
J3robot=JR3*inv(J3);
Jrobot=[J1robot ;J2robot ;J3robot]
             [A_11,A_12,A_13,A_14,b1a,b2a,b3a,b4a,r1a,r2a,r3a,r4a,delta_1,delta_2,delta_3,delta_4,delta_5,e_1,e_2,e_3,e_4,center_plat,nio_Rot_ni1,o_i,xi,yi]=ik_fourspssp(1,L,l1,l2,alpha,beta,lambda,r_o,O)
             J1=Jacostackedfourspss(delta_1,delta_2,delta_3,delta_4,e_1,e_2,e_3,e_4,xi,yi,o_i,A_11,A_12)

    plotstackedfourspss(A_11,A_12,A_13,A_14,b1a,b2a,b3a,b4a,r1a,r2a,r3a,r4a,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat,O,alpha,beta,lambda,o_i,r_o)
   [A_21,A_22,A_23,A_24,b1b,b2b,b3b,b4b,r1b,r2b,r3b,r4b,delta_1,delta_2,delta_3,delta_4,delta_5,e_1,e_2,e_3,e_4,center_plat,nio_Rot_ni1,o_i,xi,yi]=ik_fourspssp(2,L,l1,l2,alpha,beta,lambda,r_o,O)
   J2=Jacostackedfourspss(delta_1,delta_2,delta_3,delta_4,e_1,e_2,e_3,e_4,xi,yi,o_i,A_21,A_22)
 
   center_plat1=plotstackedfourspsssecond(A_21,A_22,A_23,A_24,b1b,b2b,b3b,b4b,r1b,r2b,r3b,r4b,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat,O,alpha,beta,lambda,o_i,r_o)
   [A_31,A_32,A_33,A_34,b1c,b2c,b3c,b4c,r1c,r2c,r3c,r4c,delta_1,delta_2,delta_3,delta_4,delta_5,e_1,e_2,e_3,e_4,center_plat,nio_Rot_ni1,o_i,xi,yi]=ik_fourspssp(3,L,l1,l2,alpha,beta,lambda,r_o,O)
    J3=Jacostackedfourspss(delta_1,delta_2,delta_3,delta_4,e_1,e_2,e_3,e_4,xi,yi,o_i,A_31,A_32)

   [nio_Rot_ni1a,nio_Rot_ni1b,nio_Rot_ni1c,Rcom,Rcom1,o_i1,o_i2,o_i3]= plotstackedfourspssthird(A_31,A_32,A_33,A_34,b1c,b2c,b3c,b4c,r1c,r2c,r3c,r4c,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat1,O,alpha,beta,lambda,o_i,r_o)
    
%    JR1=augmentedrotation(eye(3))*Adjointserialparallel(nio_Rot_ni1a*Rcom*o_i2); %confirm once again
% JR2=augmentedrotation(nio_Rot_ni1a*Rcom)*Adjointserialparallel(nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1*o_i3);
% JR3=augmentedrotation(nio_Rot_ni1a*Rcom*nio_Rot_ni1b*Rcom1); %just the augmented rotation
% J1robot=JR1*inv(J1);
% J2robot=JR2*inv(J2);
% J3robot=JR3*inv(J3);
% Jrobot=[J1robot ;J2robot ;J3robot]
%    end
    drawnow; hold off;  pause(0.1); grid on;
%    plotstackedfourspss(A_11,A_12,A_13,A_14,b1a,b2a,b3a,b4a,r1a,r2a,r3a,r4a,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat,O,alpha,beta,lambda,o_i,r_o)
%    center_plat1=plotstackedfourspsssecond(A_21,A_22,A_23,A_24,b1b,b2b,b3b,b4b,r1b,r2b,r3b,r4b,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat,O,alpha,beta,lambda,o_i,r_o)
%    [nio_Rot_ni1a,nio_Rot_ni1b,nio_Rot_ni1c,Rcom,Rcom1,o_i1,o_i2,o_i3]= plotstackedfourspssthird(A_31,A_32,A_33,A_34,b1c,b2c,b3c,b4c,r1c,r2c,r3c,r4c,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat1,O,alpha,beta,lambda,o_i,r_o)

    % update position
%     dp=dX(1:3,:);
%     DP = DP + dp;
%     disk(c).P = disk(c).P + DP;
%     % update rotation
%             dth = dX(4:6);
%             dR = angle2rot(dth(1),dth(2),dth(3));
%             DR = DR*dR;
%             disk(c).R = DR*disk(c).R;
    
%  plotstackedfourspss(A_i1,A_i2,A_i3,A_i4,b1,b2,b3,b4,r1,r2,r3,r4,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat,O,2,alpha,beta,lambda,o_i,r_o)
%  hold on
% plotstackedfourspss(A_i1,A_i2,A_i3,A_i4,b1,b2,b3,b4,r1,r2,r3,r4,delta_1,delta_2,delta_3,delta_4,delta_5,center_plat,O,3,alpha,beta,lambda,o_i)
%  hold on
RMS = rms(pose_third-posed);
error = pose_third(1:3)-posed(1:3);
 iteration = iteration + 1;
 RMSE = [RMSE; iteration RMS];
 Error = [Error; iteration norm(error')];
 time = time + dt;
 light('style','local','position',[0 0 4],'color',[1 1 1])
 TRACKED_position = [TRACKED_position; [time pose_third(1) pose_third(2) pose_third(3)]];
end
disp(['point: ', num2str(k), ',  time: ', num2str(time)]);
%  daspect([1 1 1])
% view(10.126508917753073,39.679461181786593)
view(23.649403451461335,18.035975144939925)
box on
% axis tight
% xlim([-inf inf])
% ylim([-inf inf])
% zlim([-inf inf])
%   drawnow
%   pause(0.1)
%   hold on
%   currFrame= getframe(gcf);
%             writeVideo(vidObj, currFrame);
%      end
  end
%  hold off
%      close(gcf)
%  close(vidObj);



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