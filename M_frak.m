function [M_frak0,M_frak] =M_frak(M_curly0,M_curly,Ad_gbar_inv)
% Compute M_frak for ith body

M_frak0=M_curly0;
M_frak=zeros(length(squeeze(M_curly(:,1,1))),6,6);

% Calculate M_frak
for i=1:length(squeeze(M_curly(:,1,1)))
%     Adi_inv=inverse(Adjoint());
    M_frak(i,:,:)=Ad_gbar_inv(:,:,i)'*squeeze(M_curly(i,:,:))*Ad_gbar_inv(:,:,i);
end