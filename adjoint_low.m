function [adjoint_low] = adjoint_low(V,iota0)
% returns the adjoint of the given transformation

if length(V)==6
    % Extract linear and Angular portion
    v=V(1:3);
    w=V(4:6);
    % Form matrix elements
    w_tilde=skewsym(w);
    v_tilde=skewsym(v);

    adjoint_low=[w_tilde v_tilde;zeros(3) w_tilde];

elseif length(V)==3
    V_tot=iota0*V;
    % Extract linear and Angular portion
    v=V_tot(1:3); %v(all(v == 0,2),:)=[]; % removes row if the entire column is zero
    w=V_tot(4:6); %w(all(w == 0,2),:)=[]; % removes row if the entire column is zero

%     % Form matrix elements
%     w_tilde=skewsym(w);
%     v_tilde=skewsym(v);

    adjoint_low=[0 -w(3) v(2);w(3) 0 -v(1);0 0 0];
end
end