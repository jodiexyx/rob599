function p_ = downsample(p, N)
idx = int64(linspace(1, size(p, 2), N));
p_ = p(:, idx);
end