function codeword = get_codeword(feature_all, n_c)
rng(0);
[idx,codeword] = kmeans(feature_all, n_c);
end