function [R_total, t_total] = get_total_transformation(R, t)
    N=size(R,1);
    R_total = cell(N, 1); R_total{1} = R{1};
    t_total = cell(N, 1); t_total{1} = t{1};
    for i=2:N,
        R_total{i} = R{i} * R_total{i-1};
        t_total{i} = R{i} * t_total{i-1}+ t{i};
    end
end