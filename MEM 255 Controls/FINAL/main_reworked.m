%Symbols Definitions
syms j1 j2 m1 m2 l1 l2 eccen k c s;

%Symbols Value Definitions (Not Substituted Yet)
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

%Derived Quantities
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

%Frequency Domain Representation
dynamics = M*s^2+C*s+K;
non_homo = [B_T B_f];
%%% f to theta TF
f_theta = subs(det([non_homo(:,2) dynamics(:,2)])/det(dynamics));
%%% T to psi TF
T_psi = subs(det([dynamics(:,1) non_homo(:,1)])/det(dynamics));

%State Space Construction
ssA = double([zeros(2) eye(2);
            -M\K -M\C]);
ssB = double([zeros(2);
            M\[B_T B_f]]);
ssB_T = double([zeros(2,1);
              M\B_T]);
ssB_f = double([zeros(2,1);
              M\B_f]);
ssC = [1 0 0 0];
ssC_theta_phi = [1 0 0 0: 0 1 0 0];
ssC_full = eye(4);
ssD = 0;
ssAG = double(ssA  - ssB_T*G);

%%%Both Controls (Theta Output Only)
ss_full_ctrl = ss(ssA,ssB,ssC,ssD);
%%%%%Controlability
ctrb_full = ctrb(ss_full_ctrl);
full_ctrl_unctrl = length(ssA) - rank(ctrb_full);
%%%%%Observability
obs = obsv(ss_full_ctrl);
full_ctrl_unobs = length(ssA) - rank(obs);

%%%Only Thruster
ss_f = ss(ssA,ssB_f,ssC,ssD);
%%%%%Controlability
ctrb_f = ctrb(ss_f);
f_unctrl = length(ssA) - rank(ctrb_f);

%%%Only Torquer
ss_T = ss(ssA,ssB_T,ssC,ssD);
%%%%%Controlability
ctrb_T = ctrb(ss_T);
T_unctrl = length(ssA) - rank(ctrb_T);

%Stability and Oscillation Mode Analysis
[V,E] = eig(ssA);
[wn,zeta] = damp(ss_full_ctrl);

% ss Rep for P6;
ss_p6 = ss(ssA,ssB_f,ssC_full,ssD);
[outs_6,t]=impulse(ss_p6);
figure;
plot(t,outs_6(:,1));
title("System's Pitch Angle $\theta$ response to Unit Impulse Thrust Input","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant)");
xlim([0 40]);
grid; grid minor;

figure;
plot(t,outs_6(:,2));
title("System's Hinge Angle $\psi$ response to Unit Impulse Thrust Input","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant)");
xlim([0 70]);
grid; grid minor;

figure;
plot(t,outs_6(:,3));
title("System's Pitch Angle Velocity $\dot{\theta}$ response to Unit Impulse Thrust Input","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant/s)");
xlim([0 70]);
grid; grid minor;

figure;
plot(t,outs_6(:,4));
title("System's Hinge Angle Velocity $\dot{\psi}$ response to Unit Impulse Thrust Input","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant/s)");
xlim([0 70]);
grid; grid minor;

%Full State Feedback Stability Evaluation
ss_p8 = ss(ssAG,ssB_f,ssC_full,ssD);
[VG, EG] = eig(ssAG);
[wn_G, zeta_g, poles_G] = damp(ss_p8);
[outs_8,t_8] = impulse(ss_p8);

figure;
plot(t_8,outs_8(:,1));
title("System's Pitch Angle $\theta$ response to Unit Impulse Thrust Input, Full State Feedback","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant)");
grid; grid minor;

figure;
plot(t_8,outs_8(:,2));
title("System's Hinge Angle $\psi$ response to Unit Impulse Thrust Input, Full State Feedback","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant)");
xlim([0 70]);
grid; grid minor;

figure;
plot(t_8,outs_8(:,3));
title("System's Pitch Angle Velocity $\dot{\theta}$ response to Unit Impulse Thrust Input, Full State Feedback","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant/s)");
xlim([0 120]);
grid; grid minor;

figure;
plot(t_8,outs_8(:,4));
title("System's Hinge Angle Velocity $\dot{\psi}$ response to Unit Impulse Thrust Input, Full State Feedback","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant/s)");
xlim([0 70]);
grid; grid minor;

%Problem 8 Step Input
[outs_8s,t_8s] = step(ss_p8);
figure;
plot(t_8s,outs_8s(:,1));
title("System's Pitch Angle $\theta$ response to Unit Step Thrust Input, Full State Feedback","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant)");
grid; grid minor;

figure;
plot(t_8s,outs_8s(:,2));
title("System's Hinge Angle $\psi$ response to Unit Step Thrust Input, Full State Feedback","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant)");
xlim([0 70]);
grid; grid minor;

figure;
plot(t_8s,outs_8s(:,3));
title("System's Pitch Angle Velocity $\dot{\theta}$ response to Unit Step Thrust Input, Full State Feedback","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant/s)");
grid; grid minor;

figure;
plot(t_8s,outs_8s(:,4));
title("System's Hinge Angle Velocity $\dot{\psi}$ response to Unit Step Thrust Input, Full State Feedback","Interpreter","latex");
xlabel("Time (seconds)");
ylabel("\theta (radiant/s)");
xlim([0 70]);
grid; grid minor;

step(ss_p8)

