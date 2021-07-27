num = [1 3];
den = [1 3 2];
sys = tf(num,den);
[A,B,C,D] = tf2ss(num,den)
