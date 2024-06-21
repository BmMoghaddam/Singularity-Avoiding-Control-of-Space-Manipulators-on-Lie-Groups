function [M_curly0, M_curly]= M_curly_ee(m0,I0,mm,Im,Ad_gcm_inv,Ad_gcm0_inv)
% Calculates the Inertia matrix in the joint frames
n=length(mm);

M_curly=zeros(n,6,6);
% M_curly0=transpose(Ad_gm_inv(i,:,:)*)

% form set of M in the joint frames
M_curly0(:,:)=Ad_gcm0_inv'*[m0*eye(3) zeros(3);zeros(3) I0]*Ad_gcm0_inv;

for i=1:n
    M_curly(i,:,:)=Ad_gcm_inv(:,:,i)'*[mm(i)*eye(3) zeros(3);zeros(3) Im(:,:,i)]*Ad_gcm_inv(:,:,i);
end


% M_curly(n,:,:)=Ad_gcm_inv(:,:,n)'*[mm(n)*eye(3) zeros(3); zeros(3) Im(1:3,1:3,n)]*Ad_gcm_inv(:,:,n);
