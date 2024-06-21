% ************************************************************************%
%                                                                         %
%                   Relative output regulation of                         %
%                   space manipulators on Lie groups                      %
%                                                                         %
%                                                                         %
% Developed by:     Borna Monazzah Moghaddam                              %
%                   Autonomous Space Robotics and Mechatronics Laboratory %
% Supervised by:    Robin Chhabra,                                        %
%                   Carleton University, Ottawa, Canada.                  %
%                                                                         %
% Initiated: 2022 August                                                  %
%                                                                         %
% Edited:                                                                 %
% ************************************************************************%

clc
close all
clear


% -------------------------------------------- Import toolboxes

% Selfmade classes


% ******************* Initiate Constants and Dimensions **************** %%

% ***************************** Create Robot **************************** %

sc_size=1; %m % size of the spacecraft cube

% **************************** get iiwa7 info

robot = importrobot('iiwa7.urdf');
robot.DataFormat = 'column';

% **************************** Initiate states

q_m=[0;0;0;0;0;0;0];
q_dot_m=[0;0;0;0;0;0;0];
V_I0=[0;0;0;0;0;0];

% End-Effector
out=sim('EE_iiwa7.slx');

% **************************** Prepare properties in workspace                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    o be fed to simulink model

n=7; %number of joints in the manipulator

w=transpose([   0 0 1; ...
                0 1 0; ...
                0 0 1; ... % shoulder ball joint
                0 -1 0;... %elbow 
                0 0 1; ...
                0 1 0; ...
                0 0 1  ... %wrist joint
                            ]); %vector of rotation of each joint in inertial frame in initial pose

% **************************** Initiate Iota the inclusion map of the base
iota0=eye(6,6);

% **************************** Joint positions for internal
rho(1:3,1)=out.R.signal1.data(1:3,4,1);%+[0;0;0.5*sc_size];
rho(1:3,2)=out.R.signal2.data(1:3,4,1);%+[0;0;0.5*sc_size];
rho(1:3,3)=out.R.signal3.data(1:3,4,1);%+[0;0;0.5*sc_size];
rho(1:3,4)=out.R.signal4.data(1:3,4,1);%+[0;0;0.5*sc_size];
rho(1:3,5)=out.R.signal5.data(1:3,4,1);%+[0;0;0.5*sc_size];
rho(1:3,6)=out.R.signal6.data(1:3,4,1);%+[0;0;0.5*sc_size]; rho(2,6)=0;
rho(1:3,7)=out.R.signal7.data(1:3,4,1);%+[0;0;0.5*sc_size];


% **************************** form the overall twist matrix
Xi_m_matrix=zeros(6*n,n);
Xi_m_temp=zeros(6,n);
for i=1:n
    v(1:3,i)=-cross(w(1:3,i),rho(1:3,i));
    Xi_m_temp(1:6,i)=[v(1:3,i);w(1:3,i)];
    Xi_m_matrix(6*(i-1)+1:6*(i-1)+6,i)=[v(1:3,i);w(1:3,i)];
end

Xi_m(:,:)=Xi_m_temp;


% **************************** Set the initial poses relative to spacecraft
R(1:3,1:3,1)=out.R.signal1.data(1:3,1:3,1);
R(1:3,1:3,2)=out.R.signal2.data(1:3,1:3,1);
R(1:3,1:3,3)=out.R.signal3.data(1:3,1:3,1);
R(1:3,1:3,4)=out.R.signal4.data(1:3,1:3,1);
R(1:3,1:3,5)=out.R.signal5.data(1:3,1:3,1);
R(1:3,1:3,6)=out.R.signal6.data(1:3,1:3,1);
R(1:3,1:3,7)=out.R.signal7.data(1:3,1:3,1);

g_bar=zeros(4,4,n);
for i=1:n
    g_bar(1:4,1:4,i)=[R(1:3,1:3,i) rho(1:3,i); 0 0 0 1];
end

R_bar_rel(1:3,1:3,1)=g_bar(1:3,1:3,1);
for i=2:n
    R_bar_rel(1:3,1:3,i)=g_bar(1:3,1:3,i-1)'*g_bar(1:3,1:3,i);
end

% Set the initial poses of CoM of bodies in joint frames
g_cm=zeros(4,4,n);
g_cm0(1:4,1:4)=[R_bar_rel(:,:,1) rho(:,1); 0 0 0 1]; % ee
for i=2:n
    g_cm(1:4,1:4,i-1)=g_bar(:,:,i-1)\g_bar(:,:,i);%[R_bar_rel(:,:,i) R(1:3,1:3,i-1)'*(rho(:,i)-rho(:,i-1)); 0 0 0 1];
end

g_cm(1:4,1:4,n)=[eye(3) [0; 0; -sc_size/2]; 0 0 0 1]; %spacecraft


% Set the Adjoint of initial poses of CoM of bodies in joint frames
Ad_gcm_inv=zeros(6,6,n);
Ad_gbar_inv=zeros(6,6,n);

Ad_gcm0_inv=inv(Adjoint(g_cm0(:,:)));

for i=1:n
    Ad_gcm_inv(:,:,i)=inv(Adjoint(g_cm(:,:,i)));
    Ad_gbar_inv(:,:,i)=inv(Adjoint(g_bar(:,:,i)));
end

%Xi_m=0;
% mu=zeros(6,1)';
% mu_t=zeros(6,1)';

% End-Efffector
m0=robot.Bodies{1,n+1}.Mass;

% arm + spacecraft in reverse
mm=zeros(1,n);
mm(n)=10;%kg spacecraft

for i=2:n
    mm(n+1-i)=robot.Bodies{1,i}.Mass;
end

% ----------------- inertia matrices
I0(1:3,1:3)=[robot.Bodies{1,8}.Inertia(1) robot.Bodies{1,8}.Inertia(6) robot.Bodies{1,8}.Inertia(5); ...
    robot.Bodies{1,8}.Inertia(6) robot.Bodies{1,8}.Inertia(2) robot.Bodies{1,8}.Inertia(4); ...
    robot.Bodies{1,8}.Inertia(5) robot.Bodies{1,8}.Inertia(4) robot.Bodies{1,8}.Inertia(3)];

%spacecraft
Im(1:3,1:3,n)=eye(3)*1.66667; %for cube with density 0.8
%arm
for i=2:n
    Im(1:3,1:3,n-i+1)=[robot.Bodies{1,i}.Inertia(1) robot.Bodies{1,i}.Inertia(6) robot.Bodies{1,i}.Inertia(5); ...
    robot.Bodies{1,i}.Inertia(6) robot.Bodies{1,i}.Inertia(2) robot.Bodies{1,i}.Inertia(4); ...
    robot.Bodies{1,i}.Inertia(5) robot.Bodies{1,i}.Inertia(4) robot.Bodies{1,i}.Inertia(3)];
end


% calculate Inertia matrices in the joint frames
[M_curly0,M_curlym]=M_curly_ee(m0,I0,mm,Im,Ad_gcm_inv,Ad_gcm0_inv);

% ************** Initiate forces

f_0=[0;0;0;0;0;0];
f_m=[0;0;0;0;0;0;0];
f_e=[0;0;0;0;0;0];



% Form the math matrix diagonalized in the base frame

[M_frak0,M_frak] =M_frak(M_curly0,M_curlym,Ad_gbar_inv);

diag_M =diagonalize(M_frak0, M_frak);


m0_frak=diag_M(1:6,1:6);
m_diag=diag_M(7:end,7:end);

% ----------------- inertia matrices for simscape model
% I0(1:3,1:3)=M_curly0(4:6,4:6);%Ad_gcm_inv(:,:,1)'*[m0*eye(3) zeros(3);zeros(3) [robot.Bodies{1,8}.Inertia(1) robot.Bodies{1,8}.Inertia(6) robot.Bodies{1,8}.Inertia(5); ...
%     robot.Bodies{1,8}.Inertia(6) robot.Bodies{1,8}.Inertia(2) robot.Bodies{1,8}.Inertia(4); ...
%     robot.Bodies{1,8}.Inertia(5) robot.Bodies{1,8}.Inertia(4) robot.Bodies{1,8}.Inertia(3)]]*Ad_gcm_inv(:,:,1);
% I0(1:3,1:3)=temp(4:6,4:6);

%spacecraft 
% temp=Ad_gcm_inv(:,:,n)'*[mm(n)*eye(3) zeros(3); zeros(3) Im(1:3,1:3,n)]*Ad_gcm_inv(:,:,n);
% Im(1:3,1:3,n)=temp(4:6,4:6);
%arm
% for i=1:n
%     Im(1:3,1:3,i)=M_curlym(i,4:6,4:6);
% end

mu=zeros(6,1);
mu_t=zeros(6,1);

g_I0=eye(4);

% *************************** Set Target parameters

rho_0t=[0.24;0.26;0.25]; %m
V_0t=[-0.01;-0.01;0];%[-0.0005;-0.0005;0];
w_t=[0;0;-0.05];

mt=10;
M_t=[eye(3)*mt zeros(3); zeros(3) eye(3)*80];

mu_t=M_t*[V_0t;w_t];

% *************************** Set Controller parameters

K_p=5*[0.1 0 0 0 0 0;0 0.1 0 0 0 0; 0 0 0.1 0 0 0; 0 0 0 0.01 0 0;
    0 0 0 0 0.01 0;0 0 0 0 0 0.01];%0.05*eye(6);
K_d=5*[0.7*eye(3) zeros(3);zeros(3) 0.5*eye(3)];
K_i=5*[0.01*eye(3) zeros(3);zeros(3) 0.001*eye(3)];
% 
% 
% % P=M0*V_I0+M0\M0m*q_dot_m; mu=P;
% q_m=[0.2;1.2;0.3;1.8;0.5;1.3;0.6];
% % q_dot_m=0.1*[0.2;1;0.3;1;0.5;0;0];%[0;0;0;0;0;0;0];
% f_m=[0;0;0;0;0;0;0];
% 
% M_temp=sim('M0_initiate',0.01);
% M0=M_temp.M0.data(:,:,1);%iota0'*M_frak0*iota0;
% M0m=M_temp.M0m.data(:,:,1);
% % M0m=M_temp.M0m.data(:,:,1);
% P0=M0*V_I0+M0\M0m*q_dot_m;
% mu=P0;