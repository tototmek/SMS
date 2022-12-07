function plot_output(y, u, E, y_zad)
    margin = 0.1;
    ymax = max(max(y_zad), max(y));
    ymin = min(min(y_zad),min(y));
    umax = max(u);
    umin = min(u);
    dy = ymax - ymin;
    du = max(u) - min(u);
    ymin = ymin - dy * margin;
    ymax = ymax + dy * margin;
    umin = umin - du * margin;
    umax = umax + du * margin;
    kmax = size(y, 2);

    set(groot,'defaultAxesTickLabelInterpreter','latex'); 
    figure
    subplot(2,1,1);
    stairs(y_zad);
    hold on;
    stairs(y);
    ylim([ymin, ymax]);
    xlim([0, kmax]);
    ylabel('$y$', Interpreter='latex');
    xlabel('$k$', Interpreter='latex');
    legend({'$y^{zad}$', '$y$'}, Interpreter='latex', Location="southeast");
    title(append(strrep(strrep(sprintf("$E=%.2e", E), ".", ","), "e+0", "\cdot 10^{"), "}$"), Interpreter="latex");
    subplot(2,1,2);
    stairs(u, Color="#EDB120");
    ylim([umin, umax]);
    xlim([0, kmax]);
    ylabel('$u$', Interpreter='latex');
    xlabel('$k$', Interpreter='latex');
    legend({'$u$'}, Interpreter='latex', Location="southeast");
end