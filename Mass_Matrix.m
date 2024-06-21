function [M0,M0m,Mm]=Mass_Matrix(I0,Im,m0,mm,Tij,Ti0,q_m,Xi_m)
% Compute the Mass Matrix of the multibody system.
%   Output:
%   M0      Spacecraft mass matrix 
%   Mm      Manipulator mass matrix

n=length(q_m);

%Initiate
Mm=zeros(6,6,n,'like',I0);

% Calculate L matrix
for i=1:n
    for j=1:n
        if i>j
            Ad_temp=eye(6);
            for k=i:j
                Ad_temp=Ad_temp*Adjoint(g_matrix(-Xi_m(k),q_m(k)));
            end
            L(6*(i-1)+1:6*(i-1)+6,6*(j-1)+1:6*(j-1)+6)=Ad_temp;
        elseif i==j
            L(6*(i-1)+1:6*(i-1)+6,6*(j-1)+1:6*(j-1)+6)=zeros(6);
        elseif i<j
            L(6*(i-1)+1:6*(i-1)+6,6*(j-1)+1:6*(j-1)+6)=zeros(6);
        end
    end
end

%Backwards recursion
for i=n:-1:1
    %Initialize M 
    Mm(1:6,1:6,i)=[Im(1:3,1:3,i),zeros(3,3);zeros(3,3),mm(i)*eye(3)];
    for j=1:n-i
        Mm(1:6,1:6,i)=Mm_tilde(1:6,1:6,i)+Tij(1:6,1:6,i+j,i)'*Mm(1:6,1:6,i+j)*Tij(1:6,1:6,i+j,i);
        % Conjugation of the Inertia matrix by the translation matrix corresponding to CG
    end
end

% Spacecraft M tilde
M0=[I0,zeros(3,3);zeros(3,3),m0*eye(3)];
for j=1:n
    M0=M0+Ti0(1:6,1:6,j)'*Mm(1:6,1:6,j)*Ti0(1:6,1:6,j);
end


end