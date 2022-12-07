function matrix_to_ccode(M)
i_max = size(M, 1);
j_max = size(M, 2);
fprintf("{")
for i = 1:i_max
    fprintf("{")
    for j = 1:j_max-1
        fprintf("%f,", M(i, j))
    end
    fprintf("%f", M(i, j_max))  
    fprintf("}")
    if i < i_max
        fprintf(",")
    end
end
fprintf("};")
end