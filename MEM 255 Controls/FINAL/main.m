%Parameters Values
j1=33;
j2=10;
m2=20;
m1=6*m2;
l1=0.5;
l2=0.5;
eccen=0.2;
k=1000;
c=2;

%Flags
thrust_off = false;
torque_off = false;

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
%%%State Space Object 1
ss_1 = ss(ssA1,ssB1,ssC1,ssD1);
%%%Equvalent TF;
tf_1 = tf(ss_1);
%%%System Poles
poles1 = pole(ss_1);
%%%Controllability Matrix
ctrb_ss_1 = ctrb(ss_1);
num_unctrb_ss_1 = length(ssA1) - rank(ctrb_ss_1);
%%%Observability Matrix
obs_ss_1 = obsv(ss_1);
num_unobs_ss_1 = length(ssA1) - rank(obs_ss_1);
%%Step Input (Make Sure to Disable Torquer)
step(ss_1);
