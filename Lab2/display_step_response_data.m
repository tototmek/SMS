D = 40;
fprintf("{")
s(1:D) = 0;
for i=1:D
    fprintf("%.2f, ", y(i));
    s(i) = y(i);
end
fprintf("}")
plot(s);