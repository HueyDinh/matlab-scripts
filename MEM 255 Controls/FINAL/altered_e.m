%%% Symbols Definitions
syms j1 j2 m1 m2 l1 l2 eccen k c s;

%%% Symbols Value Definitions (Not Substituted Yet)
j1=33;
j2=10;
m2=20;
m1=6*m2;
l1=0.5;
l2=0.5;
eccen=0; %The Only Change
k=1000;
c=2;
G = [1 3 3 1];

%%% Derived Quantities
M11 = (m1*m2)/(m1+m2)*(l1+l2)^2+j1+j2;
M12 = (m1*m2)/(m1+m2)*l2*(l1+l2)+j2;
M22= (m1*m2)/(m1+m2)*l2^2+j2;
M = [M11 M12;
     M12 M22];
C = [0 0;
     0 c];
K = [0 0;
     0 k];
B_T = [1 0]'; 
B_f = [-m2/(m1+m2)*l2-(m2/(m1+m2)-eccen)*l1 -m2/(m1+m2)*l2]';


%%
%%% State Space Construction (All Variants of A, B, and C(M) Matrices)

% A Matrix
ssA = double([zeros(2) eye(2);
            -M\K -M\C])
% B Matrix - Full Control
ssB = double([zeros(2);
            M\[B_T B_f]])
% B Matrix - Torquer Only
ssB_T = double([zeros(2,1);
              M\B_T])
% B Matrix - Thruster Only
ssB_f = double([zeros(2,1);
              M\B_f])
% C Matrix (Realistic, 1 Sensor for Theta Only)
          ssC = [1 0 0 0];
% D Matrix
ssD = 0;

%%
%%% Both Controls ss - Part 4a
ss_full_ctrl = ss(ssA,ssB,ssC,ssD);
% Controlability Matrix
ctrb_full = ctrb(ss_full_ctrl)
% Number of Uncontrollable States
full_ctrl_unctrl = length(ssA) - rank(ctrb_full)

%%
%%% Only Torquer ss - Part 4b
ss_T = ss(ssA,ssB_T,ssC,ssD);
% Controlability Matrix
ctrb_T = ctrb(ss_T)
% Number of Uncontrollable States
T_unctrl = length(ssA) - rank(ctrb_T)

%%
%%% Only Thruster ss - Part 4c
ss_f = ss(ssA,ssB_f,ssC,ssD);
% Controlability Matrix
ctrb_f = ctrb(ss_f)
% Number of Uncontrollable States
f_unctrl = length(ssA) - rank(ctrb_f)

%%
%%% Observability Matrix - Part 4d
obs = obsv(ss_full_ctrl)
% Number of Unobservable States
full_ctrl_unobs = length(ssA) - rank(obs)
