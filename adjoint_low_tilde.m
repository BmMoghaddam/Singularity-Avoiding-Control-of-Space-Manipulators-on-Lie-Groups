function [adjoint_low_tilde] = adjoint_low_tilde(P)
% returns the adjoint of the given transformation

    % Extract linear and Angular portion
    f=P(1:3);
    tau=P(4:6);
    % Form matrix elements
    f_tilde=skewsym(f);
    tau_tilde=skewsym(tau);

    adjoint_low_tilde=[zeros(3) f_tilde; f_tilde tau_tilde];

end