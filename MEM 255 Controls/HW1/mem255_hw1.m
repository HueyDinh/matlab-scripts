gamma_0=0.25;
gamma_list =linspace(0,2,10000);
init = [2 0]';
threshold =init(1)*0.0001;
t_span = [0 100];
[t,y] = ode45(@(t,y) p1(t,y,gamma_0),t_span,init);
tau = negligible(t,y(:,1),threshold);
% temp_funct = @(gamma) tau_gamma_pair(@p1,t_span,init,gamma,threshold);
% [tau_series,gamma_series] = arrayfun(temp_funct,gamma_list);

f1=figure;
plot1 = plot(t,y(:,1),'r');
axis tight;

function dy = p1(t,y,gamma)
dy = [y(2);-gamma*y(2)-y(1)];
end

function t = negligible(t,y,threshold)
index = abs(y) < threshold;
t_list=t(index);
t=t_list(1);
end

function [tau,gamma] = tau_gamma_pair(handle,t_span,init,gamma,threshold)
[t,y] = ode45(@(t,y) handle(t,y,gamma),t_span,init);
tau = negligible(t,y(:,1),threshold);
end