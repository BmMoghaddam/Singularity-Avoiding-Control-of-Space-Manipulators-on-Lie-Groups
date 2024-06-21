function [x] = vee(X)
%Form the vector from the skew symmetric matrix
% x=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
x=[X(1:3,4);X(3,2); X(1,3);X(2,1)];
end

