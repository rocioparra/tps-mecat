close all;
syms k tau k kp ki real;
syms s;

P = k/(s*tau+1);
C = kp + ki/s;

L = P*C;
T = simplify(L/(1+L));