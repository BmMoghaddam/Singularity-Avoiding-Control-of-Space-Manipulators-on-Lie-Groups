function [Ad_partial] = Ad_partial(j,k,i,Xi_m,q_m,iota0)

if j>i && i>=k
    Ad_partial=-Ad_frak(j,i+1,Xi_m,q_m)*adjoint_low(Xi_m(:,i),iota0)*Ad_frak(i,k,Xi_m,q_m);
elseif j==i && i>=k
    Ad_partial=-adjoint_low(Xi_m(:,i),iota0)*Ad_frak(i,k,Xi_m,q_m);
else
    Ad_partial=zeros(6);
end

end