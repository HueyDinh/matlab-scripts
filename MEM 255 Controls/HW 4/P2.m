num = [1 3];
den = [1 3 2];
sys = tf(num,den);
[A,B,C,D] = tf2ss(num,den)

ss_sys=ss(A,B,C,D);
[vect,~]=eig(A);
ss2=ss2ss(ss_sys,vect^-1)