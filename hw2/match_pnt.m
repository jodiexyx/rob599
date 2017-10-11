function [match, min_dist] = match_pnt(p_t, p_s)
    ns = size(p_s,2);
    match = zeros(ns,1);
    min_dist = zeros(ns,1);
    for i=1:ns,
        dist = sqrt(sum((p_t-p_s(:,i)).^2));
        [min_dist(i),match(i)] = min(dist);
    end
end