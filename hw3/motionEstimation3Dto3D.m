function [R,t] = motionEstimation3Dto3D(It0_left, It0_right, It1_left, It1_right)
    %
    % 3D to 3D motion estimation. t0 and t1 refer to the time frames t and
    % t+1, respectively.
    %
    % read calibration parameters (intrinsic and extrinsic) from the
    % calibration file
    calibname = 'calib.txt';
    T = readtable(calibname, 'Delimiter', 'space', 'ReadRowNames', true, 'ReadVariableNames', false);
    B = table2array(T);
    Pleft = vertcat(B(1,1:4), B(1,5:8), B(1,9:12)); % left camera [3x4] projection matrix
    Pright = vertcat(B(2,1:4), B(2,5:8), B(2,9:12)); % right camera [3x4] projection matrix
    %
    % calculate correspondences between It0_left, It0_right stereo pair
    [MTt0L, MTt0R, FT0L] = detectFeatureMatches(It0_left, It0_right); 
    % calculate correspondences between It1_left, It1_right stereo pair
    [MTt1L, MTt1R, FT1L] = detectFeatureMatches(It1_left, It1_right); 
    % calculate correspondences between It0_left and It1_left (loc_ft0,loc_ft1)
    
    indexPairs = matchFeatures(FT0L,FT1L);
    MTt0L=MTt0L(indexPairs(:,1));
    MTt0R=MTt0R(indexPairs(:,1));
    MTt1L=MTt1L(indexPairs(:,2));
    MTt1R=MTt1R(indexPairs(:,2));
    
    loc_ft0_SP=MTt0L;
    loc_ft1_SP=MTt1L;
    
    N=size(loc_ft0_SP,1);
    loc_ft0=zeros(N,2);
    loc_ft1=zeros(N,2);
    for i=1:N
        loc_ft0(i,:)=loc_ft0_SP(i).Location;
        loc_ft1(i,:)=loc_ft1_SP(i).Location;
    end
    
    % calculate 3D point cloud Wt0 from It0_left, It0_right stereo pair
    Wt0=get_3d_point(MTt0L, MTt0R, Pleft, Pright);
    % calculate 3D point cloud Wt1 from It1_left, It1_right stereo pair   
    Wt1=get_3d_point(MTt1L, MTt1R, Pleft, Pright);
    
    % compute R_total, t_total using ransac
    [R,t] = estimateTransform(Wt0,Wt1,loc_ft0,loc_ft1, Pleft);
%EOF
end 
function [R,t] = estimateTransform(Wt0,Wt1,loc_ft0,loc_ft1, Pleft)
    %
    %   Find the R,t between source and target point clouds (Wt0 and wt1, respectively)
    %   using a set of corresponding points whose locations are given by loc_ft0,loc_ft1. 
    %   RANSAC method is used.
    %
    % OUTPUTS:
    %   The estimated R and t
    %
    % Extract the RANSAC coefficients
    %   s: smallest number of points required to fit the model
    s = 3;
    %   w: Percentage number of inliers desired 
    w = 0.2;
    %   p: Accuracy in fitted model desired
    p = 0.99;
    % Number of iterations required to find rotation matrix using RANSAC
    N = floor(log(1-p)/log(1-w^s));
    %
    coeff.numPtsToSample = 3;   %size(Wt0,1); % number of correspondences to sample needed to fit a model
    coeff.iterNum = 2000;       % N;         % number of iterations to run the RANSAC sampling loop
    coeff.thDist = 1.5;         %2.0;        % inlier distance threshold; for us this is in pixels

    coeff.source_feat_locs = loc_ft0; % 2D locations of the features in the source t0 image
    coeff.target_feat_locs = loc_ft1; % 2D locations of the featuers in the target t1 image
    coeff.camera_projective_mat = Pleft;    % intrinsic/projective camera matrix
    coeff.randomseed = 2.8;    % intrinsic/projective camera matrix
    
    
    % find those inliers!
    [R,t] = ransac3D(Wt0, Wt1, coeff);
    %
    % EOF
end

function [R,t]=ransac3D(Wt0, Wt1, coeff)
    rng(coeff.randomseed);
    R=zeros(3);
    t=zeros(3,1);
    bestInlierNum=0;
    
    P=coeff.camera_projective_mat;
    j_t0=coeff.source_feat_locs;
    j_t1=coeff.target_feat_locs;
    
    N=size(Wt0,1);
    
    for i=1:coeff.iterNum
        
        idx=randsample(N, coeff.numPtsToSample);
        
        Ws_t0=Wt0(idx,:);
        Ws_t1=Wt1(idx,:);
        
        centroid_0=mean(Ws_t0);
        centroid_1=mean(Ws_t1);
        
        H=Ws_t1'*Ws_t0;
        [U,~,V]=svd(H);
        cur_R=V*U';
        cur_t= -cur_R*centroid_1' + centroid_0'; 
        T=[cur_R,cur_t;0,0,0,1];
        
        ptw1=P*T*[Wt1';ones(1,N);];
        jw1=ptw1(1:2,:)./repmat(ptw1(3,:),[2,1]);
        ptw0=P*inv(T)*[Wt0';ones(1,N);];
        jw0=ptw0(1:2,:)./repmat(ptw0(3,:),[2,1]);
        
        dist=[j_t0'-jw1, j_t1'-jw0];
        dist=sqrt(sum(dist.^2));
        cur_inlier=sum(dist<coeff.thDist);
        
        if cur_inlier>bestInlierNum
            bestInlierNum=cur_inlier;
            R=cur_R;
            t=cur_t;
        end
        
    end

end

function point_cloud=get_3d_point(matchedPointsL, matchedPointsR, Pleft, Pright)
    N=size(matchedPointsL, 1);
    point_cloud=zeros(N,3);
    
    %For all point correspondences
    for i = 1:N
        % For all of the matched/corresponding points between ImageL and ImageR
        %
        % Perform linear triangulation
        % YOU NEED TO IMPLEMENT THIS FUNCTION
        % point_cloud = linear_triangulation(point_left, point_right);
        %
        %
        A=zeros(4,4);
        
        xl=matchedPointsL(i).Location;
        xr=matchedPointsR(i).Location;
        
        A(1,:)=xl(1)*Pleft(3,:)-Pleft(1,:);
        A(2,:)=xl(2)*Pleft(3,:)-Pleft(2,:);
        A(3,:)=xr(1)*Pright(3,:)-Pright(1,:);
        A(4,:)=xr(2)*Pright(3,:)-Pright(2,:);
        
        [~,~,V]=svd(A);
        X=V(:,4);
        
        point_cloud(i,:)=X(1:3)./X(4);
        
    end
end

%