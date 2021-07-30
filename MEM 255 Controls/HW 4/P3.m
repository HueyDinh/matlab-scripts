num=[1 5];
den=[2 4.8 9.6 16];
[A, B, C, D]=tf2ss(num,den)
sys=ss(A,B,C,D);
t2=linspace(0,5000,100000)';
u=2*sin(0.01*t2);
c=lsim(sys,u,t2);

figure;
opt = stepDataOptions('StepAmplitude',2);
step(sys,opt);
title("System Step Response (Amplitude 2)","interpreter","latex");
xlabel("Time");
ylabel("c(t)");
grid();

figure;
plot(t2,c)
title("$e(t)=2\sin(0.01t)$ System Response","interpreter","latex");
xlabel("Time (seconds)");
ylabel("c(t)");
grid();