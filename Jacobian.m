function [J]=Jacobian_0(robot,q)
% Compute the geometric Jacobian of the point rp
%   	rp  point of interest in inertial frame 
%       i   link number

% Initiate
J=zeros(6,robot.vehicle.b + robot.n);
% Jm=zeros(6,robot.n);

% Vehicle portion
J0=[robot.vehicle.iota_0];

% ____________ Manipulator Jacobian _____________ %

j=1;
Jm(1:6,j)=robot.joints(j).xi;%[eye(3),zeros(3,3);SkewSym(rL(1:3,j)-rp),eye(3)]*pm(1:6,j);
for j=2:robot.n %Iterate through all joints
    temp=simplify(Adjoint(g(robot.joints(j-1).xi,q(j-1)))*robot.joints(j).xi);%[eye(3),zeros(3,3);SkewSym(rL(1:3,j)-rp),eye(3)]*pm(1:6,j);
    Jm(1:6,j)=temp;
end

Jm=simplify(Jm);

J=[J0 Jm(1:6,1:j)];

end