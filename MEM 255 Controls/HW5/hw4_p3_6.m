% P3.6
tset_2 = 1/(-1)*log(0.02);
tset_3 = 1/(-2)*log(0.02);
limit = max(tset_2,tset_3)*1.2;
t = 0:0.001:limit;

x_1 = @(t) 2;
x_ser_1 = arrayfun(x_1,t);

x_2 = @(t) -4*exp(-t);
x_ser_2 = x_2(t);

x_3 = @(t) 2*exp(-2*t);
x_ser_3 = x_3(t);

x_ser = arrayfun(@(t) x_1(t) + x_2(t) + x_3(t),t);

figure;
hold on
title("P3.6 System Response Break-down");
plot(t,x_ser_1,'DisplayName',"Constant Response (x=2)");
plot(t,x_ser_2,'DisplayName',"Exponential Response (Pole = -1)");
plot(t,x_ser_3,'DisplayName',"Exponential Response (Pole = -2)");
plot(t,x_ser,'DisplayName',"Overall System Step Response");
xlim([0 limit])
ylim([-4.5 2.5]);
xlabel("Time (seconds)");
ylabel("Response Amplitude");
legend("Location",'southeast');
grid; grid minor;

%test
% sys = tf([4],[1 3 2]);
% step(sys);
