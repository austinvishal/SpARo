function P=threeplaneintersectionpoint(normal1, normal2,normal3,d1,d2,d3)
A1=cross(normal2,normal3);
A2=cross(normal3,normal1);
A3=cross(normal1,normal2);
P=(-d1*A1-d2*A2-d3*A3)/(dot(normal1,A1));
end