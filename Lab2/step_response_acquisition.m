clear all;
close all;
s = serialport('COM3', 115200, 'Parity', 'None');
s.configureTerminator("LF");
y = zeros(100, 1);
u = zeros(100, 1);
for k=1:600
    txt = s.readline();
    eval(char(txt'));
    y(k) = Y;
    u(k) = U;
    disp(char(txt'));
end
clear s;