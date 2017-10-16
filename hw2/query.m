function [d, idx] = query()
    kitti = load('kitti_bow.mat');
    hist_train = kitti.hist_train;
    kdtree = kitti.kdtree;
    n_c = kitti.n_c;
    img_q = rgb2gray(imread('query.png'));
    feature_q = SURF(img_q);
    h = get_hist(kdtree, feature_q, n_c);
    d = zeros(size(hist_train,1),1);
    for i=1:size(hist_train,1),
        d(i) = chi_sq_dist(h,hist_train(i,:));
    end
    [~,idx] = min(d);
end