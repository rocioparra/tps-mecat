close all;
device = serialport("COM3", 115200);
%configureTerminator(device, "CR");
tstart = tic;

data = [];
while toc(tstart) <= 4
    data = [data str2double(readline(device))];
end

t = linspace(0, 4, length(data));
plot(t, data/12/34.014*60.0);
ylim([0, 300]);
csvwrite("lazocerrado_0_290.csv", data);
clear device;