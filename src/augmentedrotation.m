function [Raug]=augmentedrotation(R)
Raug=[R zeros(3,3);
    zeros(3,3) R];
end