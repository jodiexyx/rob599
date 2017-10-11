function [R_total, t_total] = icp_3d(p_t, p_s)
N = 2000;
p_t = downsample(p_t, N);
p_s = downsample(p_s, N);

%%
eps = 1e-8;
k = 0;
R_total = eye(3);
t_total = zeros(3, 1);
while 1
    %% Update p2, the current source cloud
    p2 = R_total' * (p_s - t_total);
    
    %% Get p1, the matching points from the target cloud
    [match, min_dist] = match_pnt(p_t, p2);
    p1 = p_t(:, match);
    d = min_dist/std(min_dist);
    w = exp(-d)/sum(exp(-d));
    
    %% Centroids of p1 and p2
    mu1 = p1*w;
    mu2 = p2*w;
    
    %% Center the two clouds
    p1_bar = (p1 - mu1).*w';
    p2_bar = (p2 - mu2).*w';
    
    %% Estimate the rotation and translation
    [U, ~, V] = svd(p1_bar * p2_bar');
    R = V*U';
    t = mu2-R*mu1;
    
    %% Update R_total and t_total
    t_total = t_total + t;
    R_total = R_total*R;

    %% Terminate when [R, t] is very close to [I, 0]
    delta = norm([R - eye(3), t]);
    if delta > sqrt(eps)
        pause(0.02)
        k = k + 1;
    else
        break
    end
end

end