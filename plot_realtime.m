close all;
s = serialport('COM3', 115200, 'Parity', 'None');
s.configureTerminator("LF");

u_line = animatedline("Color", "g");
y_zad_line = animatedline("Color", "b");
y_line = animatedline("Color", "r");

for k = 1:6000
    txt = s.readline();
    eval(char(txt'));
    y(k) = Y;
    u(k) = U;
    y_zad(k) = Y_zad;
    disp(char(txt'));
    addpoints(u_line, k, u(k));
    addpoints(y_zad_line, k, y_zad(k));
    addpoints(y_line, k, y(k));
    drawnow;
end