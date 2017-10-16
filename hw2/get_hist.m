function h = get_hist(kdtree, feature, n_c)
    idx=knnsearch(kdtree,feature);
    h=zeros(1,n_c);
    for i=1:length(idx),
        h(idx(i)) = h(idx(i))+1;
    end
    h = h/length(idx);
end