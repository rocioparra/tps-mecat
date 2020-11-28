close all;

data = csvread("lazocerrado.csv")/12/34.014*60.0;
data = data(2:end);

Ts = 0.001;
t = 0:Ts:Ts*(length(data)-1);

p = plot(t, data);
grid on;
hold on;

t0 = 0.272;

[s, t2] = step(T);
s = s(t2<2-t0); t2 = t2(t2<2-t0);
plot(t2+t0, s*(290-200)+200);

xlabel('Tiempo (s)');
ylabel('Velocidad angular (rpm)');
legend('Medición', 'Modelo');
