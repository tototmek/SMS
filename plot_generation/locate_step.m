function t0 = locate_step(x)
    t0 = 0;
    for k = 2:size(x, 2)
        if (x(k-1) < x(k))
            t0 = k;
            break;
        end
    end
end