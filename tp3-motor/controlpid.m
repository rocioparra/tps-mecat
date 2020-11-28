close all;
s = tf('s');
P = k/(s*tau+1);

mp = 0.1; ts = 1.5;
xi = -log(mp) / sqrt( pi^2 + log(mp)^2 );
w0 = 4/xi/ts;

ki = 1*w0^2/k*tau


kp = 1*(2*xi*w0*tau-1)/k
C = kp + ki/s;

L = P*C;
T = L/(1+L);
step(T);
% [y, t] = step(T); 
% p = plot(t, y);
grid on;

tm = 0.36;
ap = 1.08;

axes = gca;
datatip(axes.Children(1).Children(2), tm, ap);