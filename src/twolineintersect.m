function point3= twolineintersect(P1,v,Q1,u)
% line1 P1+tv; P1 is point, v is direction
%line2 Q1+su;  Q1 is point, u is direction
if((dot(cross(v,u),cross(v,u)))==0)
    point3  = NaN(1,3,'single');
else
   t=(dot(cross((Q1-P1),u),cross(v,u)))/(dot(cross(v,u),cross(v,u)));
   point3=P1+(t*v);
end
