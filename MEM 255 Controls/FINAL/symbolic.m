%Parameters
syms j1 j2 m1 m2 l1 l2 eccen k c s gam;

%Flags
thrust_off = false;
torque_off = true;

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
if ~thrust_off && ~torque_off
    B_matrix = [B_T B_f];
elseif thrust_off && ~torque_off
    B_matrix = B_T;
elseif ~thrust_off && torque_off
    B_matrix = B_f;
else
    B_matrix = [0 0]';
end
[~,num_col_B_mat] = size(B_matrix);

%System Dynamics
%%%Phase Variable - Input1 = T, Input2 = f; Output1 = theta; Output2 = phi;
ssA1 = [zeros(2) eye(2);
      -M\K -M\C];
ssB1 = [zeros(2,num_col_B_mat);
      M\B_matrix];
ssC1 = [1 0 0 0];
ssD1 = 0;

%TF from Tc to theta Construction
diff = M*s^2+C*s+K;
tf = det([[1 0]',diff(:,2)])/det(diff)*gam;
closed = simplify(tf/(1+tf));

%Parameter Values
j1=33;
j2=10;
m2=20;
m1=6*m2;
l1=0.5;
l2=0.5;
eccen = -23/130;
k=1000;
c=2;

%Substitutions
ssA1 = subs(ssA1);
ssB1 = subs(ssB1);
ssC1 = subs(ssC1);
ssD1 = 0;
closed = subs(closed);
[num, den] = numden(closed)
diff=subs(diff);
pretty(det(diff))
pretty(closed)

%Controllability Matrix;
ctrb_ss_1 = [ssB1 ssA1*ssB1 ssA1^2*ssB1 ssA1^3*ssB1];
pretty(ctrb_ss_1);
num_unctr = length(ssA1) - rank(subs(ctrb_ss_1))

%Observability Matrix;
obsv_ss_1 = [ssC1; ssC1*ssA1; ssC1*ssA1^2; ssC1*ssA1^3];
pretty(obsv_ss_1);
num_unobsv = rank(ssA1) - rank(subs(obsv_ss_1));

