function generate_step_plot(filename, time_before, time_after)
    data = load(filename, "y_zad");
    y_zad = data.y_zad;
    start = locate_step(y_zad);
    plot_from_file(filename, start - time_before, start + time_after);
end