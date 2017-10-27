function point_cloud = gen_pointcloud(Ileft, Iright, Pleft, Pright)
%
% performs image triangulation on stereo pair given by Ileft and Iright

%
% extract and match SURF features from the left and right images
[matchedPointsL, matchedPointsR] = detectFeatureMatches(Ileft, Iright); 

N=size(matchedPointsL,1);

%A=zeros(4*N, 4);
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

%
% Visualization Code
scatter3(point_cloud(:,1), point_cloud(:,2), point_cloud(:,3))

% EOF
end