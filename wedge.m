function [x] = wedge(X)
%Form the vector from the skew symmetric matrix
% x=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
v=X(1:3);
w=X(4:6);
x=[skewsym(w) v;zeros(1,4)];
end

