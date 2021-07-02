poles = roots([1 7 3 2 1 0 3]);
zeros = roots([1 2 4 1 0 3]);
figure;
plot(poles,"x");
hold on;
plot(zeros,"o");
hold off;
title('Transfer function poles and zeros visualization.');
xlabel("Real Part");
ylabel("Imaginary Part");
yline(0);
xline(0);


