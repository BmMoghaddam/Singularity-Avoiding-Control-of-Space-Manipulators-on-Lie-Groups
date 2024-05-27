function [Ad_frak] = Ad_frak(i,j,Xi_m,q_m)
Ad_frak=eye(6,6);

if i>=j
    g_temp=eye(4);
    for k=j:i
        g_temp=g_Matrix(Xi_m(1:6,k),-q_m(k))*g_temp;
    end
%     elseif i==j
%         g_temp=g_temp*g_Matrix(Xi_m(1:6,i),-q_m(i));
    Ad_frak=Adjoint(g_temp);
end

end