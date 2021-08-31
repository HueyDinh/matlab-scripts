%% Symbols Definitions
syms j1 j2 m1 m2 l1 l2 eccen k c gam s;

%%% Symbol Values - e will be Provided Later

j1=33;
j2=10;
m2=20;
m1=6*m2;
l1=0.5;
l2=0.5;
k=1000;
c=2;

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
B_f = [-m2/(m1+m2)*l2-(m2/(m1+m2)-eccen)*l1 -m2/(m1+m2)*l2]';

%% Closed Loop TF - Part 7

% TF from Tc to theta Construction
diff = M*s^2+C*s+K;
tf_c = det([[1 0]',diff(:,2)])/det(diff)*gam;
% Closed-loop Transfer Function from Reference Signal to Pitch Angle
closed = simplify(tf_c/(1+tf_c));
% Substitution - Gamma not Substituted
closed = subs(closed);
pretty(closed)
%% gamma = 500 Substitution
closed = simplify(subs(closed,gam,500));
pretty(closed)
cl_tf = tf([50,7,3500]*500,[1800,421,235500,3500,1750000]);
%%% Poles and Damping Characteristics
[wn,zeta,poles]=damp(cl_tf)

%% Routh Table (Symbolic) - Part 7
row_1 = [1 (8420000+2000*gam)/72000 140000/72000*gam 0 0];
row_2 = [16840/72000 280*gam/72000 0 0 0];
row_3 = -1/row_2(1)*[det([row_1(1) row_1(2);row_2(1) row_2(2)]) det([row_1(1) row_1(3);row_2(1) row_2(3)]) det([row_1(1) row_1(4);row_2(1) row_2(4)])];
row_4 = -1/row_3(1)*[det([row_2(1) row_2(2);row_3(1) row_3(2)]) det([row_2(1) row_2(3);row_3(1) row_3(3)])];
row_5 = -1/row_4(1)*[det([row_3(1) row_3(2);row_4(1) row_4(2)])];
pretty(row_1)
pretty(row_2)
pretty(row_3)
pretty(simplify(row_4))
pretty(simplify(row_5))

%% State Space Construction - Part 5
ssA1 = [zeros(2) eye(2);
      -M\K -M\C];
ssB1 = [zeros(2,1);
      M\B_f];
ssC1 = [1 0 0 0];

% Symbolic Substitutions
ssA1 = subs(ssA1);
ssB1 = subs(ssB1);
ssC1 = subs(ssC1);
ssD1 = 0;

%% Controllability Matrix with Substitution (Except for Epsilon);
ctrb_ss_1 = subs([ssB1 ssA1*ssB1 ssA1^2*ssB1 ssA1^3*ssB1]);
pretty(ctrb_ss_1)

%% Controllability Matrix Evaluation at Solution 1 Epsilon = 2/7
eccen=2/7;
pretty(subs(ctrb_ss_1))
% Number of Uncontrollable States
num_unctr_1 = length(ssA1) - rank(subs(ctrb_ss_1))

%% Controllability Matrix Evaluation at Solution 1 Epsilon = -23/130
eccen=-23/130;
pretty(subs(ctrb_ss_1))
% Number of Uncontrollable States
num_unctr_2 = length(ssA1) - rank(subs(ctrb_ss_1))
