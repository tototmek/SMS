function autogenerate_plots()
    time_before = 5;
    time_after = 300;
    directory = uigetdir;
    files = dir(fullfile(directory, "*.mat"));
    for k = 1:length(files)
        name = files(k).name;
        generate_step_plot(name, time_before, time_after);
        disp(append("Generating:", name(1:end-4), ".eps"));
        print(append("plots/", name(1:end-4)), "-depsc", "-r400");
        %close;
    end
end