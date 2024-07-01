function [alpha, beta, gamma] = Rot2Angleszyz(R)

    alpha = atan2(R(2,3),R(1,3));
    beta = atan2(sqrt(1 - R(3,3)^2),R(3,3));
    gamma = atan2(R(3,2),-R(3,1));
    
end