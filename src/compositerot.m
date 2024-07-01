function[R]= compositerot(j,angledeg)
R=[cos(deg2rad(angledeg)*(-1)^j) -sin(deg2rad(angledeg)*(-1)^j) 0
   sin(deg2rad(angledeg)*(-1)^j)  cos(deg2rad(angledeg)*(-1)^j) 0
   0 0 1];
end