function diag_M =diagonalize(M_frak0, M_frak)
% Compute M_frak for ith body
n=length(M_frak(:,1,1));

% Initiate
diag_M=zeros(6*(n+1),6*(n+1));

diag_M(1:6,1:6)=M_frak0;

% Calculate diag{M_frak}
for i=2:(n+1)
    diag_M(6*(i-1)+1:6*i,6*(i-1)+1:6*i)=M_frak(i-1,:,:);
end

end