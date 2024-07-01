function Adjspm= Adjointserialparallel(p)
%note this is not the original adjoint map, but a modified one as the
%rotations are taken out as augmented rotation matrix
% Takes T a transformation matrix SE3. 
% Returns the corresponding 6x6 adjoint representation [Adj].
% this map transforms twists from one coordinate frame to another R6--->R6
% [R, p] = TransToRp(T);
% note p has to be expressed in the base coordinates, so must be in proper
% frame of reference
Adjspm = [eye(3), -vectoso3(p); 
      zeros(3,3), eye(3)]; % note here the order is based on the fact that the 
  %twist is [v;omega] 
  %if it is considered as [omega; v], then adjoint will be
%   Adj = [R, VecToso3(p)*R; 
%       zeros(3), R];
end