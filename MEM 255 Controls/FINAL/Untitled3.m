syms x
ans = vpasolve(1-sin(x)^2-(cos(x)+sin(x)/(pi-x))^2==0,x,[pi 3*pi/2])