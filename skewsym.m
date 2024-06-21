function [X] = skewsym(x)
%Form the skew symmetric matrix
% if length(x(1,:))==3
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
% elseif length(x(1,:))==2
%     X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
% elseif length(x(1,:))==1
%     X=x;
% end
end

