% P3.7
d_ratio = 1/sqrt(10);
nat_freq = sqrt(10);

tset =  -log(0.02*sqrt(1-d_ratio^2))/(d_ratio*nat_freq);
limit = tset*1.2;
t = 0:0.001:limit;

x_1 = @(t) 2;
x_ser_1 = arrayfun(x_1,t);

x_2 = @(t) -2*exp(-t)/sqrt(9/10)*cos(3*t-atan(1/3));
x_ser_2 = arrayfun(x_2,t);

x_2_1 = @(t) -2*exp(-t)*cos(3*t);
x_ser_2_1 = arrayfun(x_2_1,t);

x_2_2 = @(t) -2/3*exp(-t)*sin(3*t);
x_ser_2_2 = arrayfun(x_2_2,t);

x_ser = arrayfun(@(t) x_1(t) + x_2(t),t);

figure;
hold on
title("P3.6 System Response Break-down");
plot(t,x_ser_1,'DisplayName',"Constant Response (x=2)");
plot(t,x_ser_2,'DisplayName',"Total Damped Oscillation Response");
plot(t,x_ser_2_1,'DisplayName',"Cosine Term")
plot(t,x_ser_2_2,'DisplayName',"Sine Term")
plot(t,x_ser,'DisplayName',"Overall System Step Response");
xlim([0 limit])
ylim([-2.5 3.5]);
xlabel("Time (seconds)");
ylabel("Response Amplitude");
legend("Location",'southeast');
grid; grid minor;

%test
sys = tf([20],[1 2 10]);
step(sys);
comb_ser = arrayfun(@(t) x_2_1(t) + x_2_2(t),t);
plot(t,comb_ser,'DisplayName',"Sum of sinusoidal Response");
