function plot_from_file(filename, start, stop)
    data = load(filename);
    y = data.y(start:stop);
    u = data.u(start:stop);
    y_zad = data.y_zad(start:stop);
    E = sum((y_zad  -  y).^2);
    plot_output(y, u, E, y_zad);
end