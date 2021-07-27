num=[1 5];
den=[2 4.8 9.6 16];
[A, B, C, D]=tf2ss(num,den);
sys=ss(A,B,C,D);
t=linspace(0,5000,100000)';
u=2*sin(0.01*t);
c=lsim(sys,u,t);

figure;
opt = stepDataOptions('StepAmplitude',2);
step(sys,opt)

figure;
plot(t,c)
