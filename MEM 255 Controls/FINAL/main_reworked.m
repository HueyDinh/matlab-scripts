%%% Symbols Definitions
syms j1 j2 m1 m2 l1 l2 eccen k c s;

%%% Symbols Value Definitions (Not Substituted Yet)
j1=33;
j2=10;
m2=20;
m1=6*m2;
l1=0.5;
l2=0.5;
eccen=0.2;
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
%%% Frequency Domain Representation - Part 1b

% Dynamics Matrix (Systems of two 2nd order ODE)
dynamics = M*s^2+C*s+K
% Non-Homogenous Matrix
non_homo = [B_T B_f]
% f to theta TF
f_theta = subs(det([non_homo(:,2) dynamics(:,2)])/det(dynamics))
% T to psi TF
T_psi = subs(det([dynamics(:,1) non_homo(:,1)])/det(dynamics))

%%
%%% State Space Construction (All Variants of A, B, and C(M) Matrices)

% A Matrix - Part 1
ssA = double([zeros(2) eye(2);
            -M\K -M\C])
% B Matrix - Full Control - Part 1a, 3a
ssB = double([zeros(2);
            M\[B_T B_f]])
% B Matrix - Torquer Only - Part 3b
ssB_T = double([zeros(2,1);
              M\B_T])
% B Matrix - Thruster Only - Part 3c
ssB_f = double([zeros(2,1);
              M\B_f])
% C Matrix (Realistic, 1 Sensor for Theta Only) - Part 1a
          ssC = [1 0 0 0];
% C Matrix - Observa All Output for Part 6 and Part 8 Plotting
ssC_full = eye(4);
% D Matrix
ssD = 0;
% Full State Feedback A Matrix - Part 8a
ssAG = double(ssA  - ssB_T*G)

%%
%%% Both Controls ss - Part 3a
ss_full_ctrl = ss(ssA,ssB,ssC,ssD);
% Controlability Matrix
ctrb_full = ctrb(ss_full_ctrl)
% Number of Uncontrollable States
full_ctrl_unctrl = length(ssA) - rank(ctrb_full)

%%
%%% Only Torquer ss - Part 3b
ss_T = ss(ssA,ssB_T,ssC,ssD);
% Controlability Matrix
ctrb_T = ctrb(ss_T)
% Number of Uncontrollable States
T_unctrl = length(ssA) - rank(ctrb_T)

%%
%%% Only Thruster ss - Part 3c
ss_f = ss(ssA,ssB_f,ssC,ssD);
% Controlability Matrix
ctrb_f = ctrb(ss_f)
% Number of Uncontrollable States
f_unctrl = length(ssA) - rank(ctrb_f)

%%
%%% Observability Matrix - Part 3d
obs = obsv(ss_full_ctrl)
% Number of Unobservable States
full_ctrl_unobs = length(ssA) - rank(obs)

%%
%%% Stability and Oscillation Mode Analysis - Part 2

% Eigenvalue and Eigenvector of A - Part 2a
[V,E] = eig(ssA)
% Naural Frequency and Damping Ratio Associated with Each Poles - Part 2b
[wn,zeta] = damp(ss_full_ctrl)

%%
%%% ss Representation for P6, Thruster Only, Full C Matrix
ss_p6 = ss(ssA,ssB_f,ssC_full,ssD);
% Impulse Response Data
[outs_6,t]=impulse(ss_p6);
% Impulse Response Plotting

%%% Theta
figure;
plot(t,outs_6(:,1));
title("System's Pitch Angle $\theta$ response to Unit Impulse Thrust Input","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant)");
xlim([0 40]);
grid; grid minor;

%%% Psi
figure;
plot(t,outs_6(:,2));
title("System's Hinge Angle $\psi$ response to Unit Impulse Thrust Input","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\psi (radiant)");
xlim([0 70]);
grid; grid minor;

%%% Theta Dot
figure;
plot(t,outs_6(:,3));
title("System's Pitch Angle Velocity $\dot{\theta}$ response to Unit Impulse Thrust Input","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("$\dot{\theta}$ (radiant/s)","Interpreter","latex");
xlim([0 70]);
grid; grid minor;

%%% Psi Dot
figure;
plot(t,outs_6(:,4));
title("System's Hinge Angle Velocity $\dot{\psi}$ response to Unit Impulse Thrust Input","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("$\dot{\psi}$ (radiant/s)","Interpreter","latex");
xlim([0 70]);
grid; grid minor;

%%
%%% Full State Feedback Stability Evaluation - Part 8
ss_p8 = ss(ssAG,ssB_f,ssC_full,ssD);
%%% Closed Loop Eigenvalue and Eigenvectors - Part 8a
[VG, EG] = eig(ssAG)

%%% Natural Frequency and Damping Ratios of All Vibration Modes - Part 8a,8b
[wn_G, zeta_g, poles_G] = damp(ss_p8)
[outs_8,t_8] = impulse(ss_p8);

%%
%%% Theta - Full State Feedback, Unit Impulse Thruster
figure;
plot(t_8,outs_8(:,1));
title("System's Pitch Angle $\theta$ response to Unit Impulse Thrust Input, Full State Feedback","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant)");
grid; grid minor;

%%% Psi - Full State Feedback, Unit Impulse Thruster
figure;
plot(t_8,outs_8(:,2));
title("System's Hinge Angle $\psi$ response to Unit Impulse Thrust Input, Full State Feedback","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\psi (radiant)");
xlim([0 70]);
grid; grid minor;

%%% Theta Dot - Full State Feedback, Unit Impulse Thruster
figure;
plot(t_8,outs_8(:,3));
title("System's Pitch Angle Velocity $\dot{\theta}$ response to Unit Impulse Thrust Input, Full State Feedback","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("$\dot{\theta}$ (radiant/s)","Interpreter","latex");
xlim([0 120]);
grid; grid minor;

%%% Psi Dot - Full State Feedback, Unit Impulse Thruster
figure;
plot(t_8,outs_8(:,4));
title("System's Hinge Angle Velocity $\dot{\psi}$ response to Unit Impulse Thrust Input, Full State Feedback","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("$\dot{\psi}$ (radiant/s)","Interpreter","latex");
xlim([0 70]);
grid; grid minor;


