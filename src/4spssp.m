syms alpha beta lambda real
a=
Bi=[X_Bi Y_Bi Z_Bi]'; %base coodinate wrt to base frame
m_bi=[x_bi y_bi z_bi]'; % wrt to moving frame
bi=[X_bi Y_bi Z_bi]';
o=[X_o Y_o Z_o]';
B_Rot_m=Rz(alpha)*Ry(beta)*Rz(lambda)   ;%rotation matrix ZYZ convention Euler

bi=B_Rot_m*m_bi+o ;%bi vector at the platform, lower case is platform
