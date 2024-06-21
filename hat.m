function [x] = hat(X)
%Form the vector from the skew symmetric matrix
x=[skewsym(X(4:6)) X(1:3); 0 0 0 0];
% x=[X(3,2); X(1,3);X(2,1)];
end

