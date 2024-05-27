function [Adj] = Adjoint(g)
% returns the adjoint of the given transformation
    R=zeros(3,3);     R=g(1:3,1:3);
    p=zeros(3,1);   p=g(1:3,4);
    Adj=[R skewsym(p)*R;  zeros(3) R];
end