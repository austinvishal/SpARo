function Adj= Adjointmap(T)
% Takes T a transformation matrix SE3. 
% Returns the corresponding 6x6 adjoint representation [Adj].
% this map transforms twists from one coordinate frame to another R6--->R6
[R, p] = TransToRp(T);
Adj = [R, zeros(3); 
      VecToso3(p)*R, R]; % note here the order is based on the fact that the 
  %twist is [omega;v] 
  %if it is considered as [v; omega], then adjoint will be
%   Adj = [R, VecToso3(p)*R; 
%       zeros(3), R];
end