close all;

data = csvread("lazoabierto.csv")/12/34.014*60.0;

Ts = 0.001;
t = 0:Ts:Ts*(length(data)-1);

t0_plot = 0.5;
tf_plot = 1;
t_plot = t(t>=t0_plot & t <=tf_plot)-t0_plot;
data_plot = data(t>=t0_plot & t <=tf_plot);
p = plot(t_plot, data_plot, '.-');
xlabel('Tiempo (s)');
ylabel('Velocidad angular (rpm)');
grid on;

t0 = 0.596; d0 = 0;
tfinal = 0.746; dfinal = 290.2;
datatip(p, t0-t0_plot, d0);
datatip(p, tfinal-t0_plot, dfinal);

% find point where signal rises to 1/e
d63 = d0+(dfinal-d0)*(1-exp(-1));
t63 = 0.613;
datatip(p, t63-t0_plot, d63);

tau = t63 - t0;

vueltas2rpm = 60/12/34.014; % sabemos que el valor final son 290rpm
rpm2vueltas = 1/vueltas2rpm;

vfinal = 12.0; v0 = 0; 
k = ((dfinal-d0)*vueltas2rpm) / (vfinal-v0);

hold on;
tt = t(t>=t0 & t<tf_plot);
plot(tt-t0_plot, rpm2vueltas*(vfinal-v0)*k*(1-exp(-(tt-t0)/tau))+d0);
legend('Medición', 'Modelo');
