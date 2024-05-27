function [Ad_partial_par] = Ad_partial_par(j,k,i,s,Xi_m,q_m,iota0)

if j>=s && s>i && i>=k 
    Ad_partial_par=-Ad_partial(j,i+1,s,Xi_m,q_m,iota0)*adjoint_low(Xi_m(:,i),iota0)*Ad_frak(i,k,Xi_m,q_m);
elseif j>i && i>=s && s>=k
    Ad_partial_par=-Ad_frak(j,i+1,Xi_m,q_m)*adjoint_low(Xi_m(:,i),iota0)*Ad_partial(i,k,s,Xi_m,q_m,iota0);
elseif j==i && i>=k && s>=k && s<=i
    Ad_partial_par=-adjoint_low(Xi_m(:,i),iota0)*Ad_partial(i,k,s,Xi_m,q_m,iota0);
else
    Ad_partial_par=zeros(6);
end

end