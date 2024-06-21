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

% -------------------------------------------- Import toolboxes

% Selfmade classes


% *********************** Initiate Forces & States **************** %%

% ***************************** Create Robot **************************** %

% **************************** Initiate states

q_m=[0.2;1.2;0.3;2.5;0.5;1.5;0.6]; %[0.2;1.5;0.3;1.8;0.5;0.3;0.6]; %[0.2;1.2;0.3;1.8;0.5;1.3;0.6];
q_dot_m=[0;0;0;0;0;0;0];% q_dot_m=0.1*[0.2;1;0.3;1;0.5;0;0];
V_I0=[-0.001;-0.001;0.0;-0.001;0.001;0.002];

g_I0=eye(4);

% **************************** Prepare properties in workspace                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    o be fed to simulink model

n=7; %number of joints in the manipulator

% ************** Initiate forces

f_0=[0;0;0;0;0;0];
f_m=[0;0;0;0;0;0;0];
f_e=[0;0;0;0;0;0];


% *************************** Set Target parameters

% rho_0t=[0.24;0.26;0.25]; %m
% V_0t=[-0.01;-0.01;0];%[-0.0005;-0.0005;0];
% w_t=[0;0;-0.05];
rho_It= [-0.3;-0.3;-0.1];%rho_0t=[0.45;0.46;0.25]; % [0.05;0.05;0.05];
V_It=[0.01;0.01;0]; %[0;0;0];-0.002z
w_t=-[0;0;0.01];

% V_t=[V_0t;w_0t];
g_It=[eye(3) rho_It;0 0 0 1];
% temp=(Adjoint(g_It))*[V_Itb;w_tb];
% V_It=temp(1:3);
% w_t=temp(4:6);

mt=10;
M_t_body=[eye(3)*mt zeros(3); zeros(3) eye(3)*0.416667];
M_t=(((Adjoint(g_It))'\M_t_body)/(Adjoint(g_It)));
mu_t=inv((Adjoint(g_It))')*M_t_body*[V_It;w_t]; %momentum of CoM of target based on M_t in body transformed to inertia times V in inertia
% inv(Adjoint([eye(3) rho_0t;0 0 0 1]))'*
% inv(Adjoint([eye(3) rho_0t-[0.25;0.25;0.25];0 0 0 1]))'*
% *************************** Set Controller parameters

K_p=100*[0.1 0 0 0 0 0;0 0.1 0 0 0 0; 0 0 0.1 0 0 0; 0 0 0 0.01 0 0; %50
    0 0 0 0 0.01 0;0 0 0 0 0 0.01];%0.05*eye(6);
K_d=10*[1.3*eye(3) zeros(3);zeros(3) 0.8*eye(3)];
K_i=5*[0.1*eye(3) zeros(3);zeros(3) 0.01*eye(3)]; %5
K_w=200*eye(7);
K_wd=2*eye(7);


% P=M0*V_I0+M0\M0m*q_dot_m; mu=P;

% f_m=[0;0;0;0;0;0;0];

M_temp=sim('M0_initiate',0.01);
M0=M_temp.M0.data(:,:,1);%iota0'*M_frak0*iota0;
M0m=M_temp.M0m.data(:,:,1);
% M0m=M_temp.M0m.data(:,:,1);
P0=M0*V_I0+(M0\M0m)*q_dot_m;
mu=P0;

rise_time=1;
fall_distance=0.01;

% q_dot_m=0.01*q_dot_m;
% V_I0=0.01*V_I0;