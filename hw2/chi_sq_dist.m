function d = chi_sq_dist(h1, h2)
d = sum((h1 - h2).^2 ./ (h1 + h2 + eps)) / 2;
end