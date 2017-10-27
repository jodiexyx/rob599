function [D] = genDisparityMap(I1, I2, min_d, max_d, w_radius)
%
% INPUTS:
% I1 = left stereo image
% I2 = right stereo image
% maxdisp = determines the region of interested to search in the right image.
%           It can be thought of as the 'shift' of the right image
% w_radius = half of window size
%

%non-pad
    ws = 2*w_radius+1;
    [M,N,color]=size(I1);

    I1_db = im2double(I1);
    I2_db = im2double(I2);

    min_SAD_map = ones(M,N,max_d-min_d+1).*(M*N*color*256);
    D = zeros(M,N);
    
    for k = min_d:max_d
        dif_img = I1_db;
        dif_img(:, (1+k):N, :)=abs(I1_db(:,(1+k):N,:)-I2_db(:,(1):(N-k),:));
        
        SAD_map=zeros(M,N);
        for c=1:color
            SAD_map = SAD_map+conv2(dif_img(:,:,c), ones(ws), 'same');
        end
        
        for i=1:M
            for j=1:N
                if SAD_map(i,j) < min_SAD_map(i,j)
                    min_SAD_map(i,j) = SAD_map(i,j);
                    D(i,j) = k; 
                end
            end
        end
        
    end


%padarray
%{
    ws = 2*w_radius+1;
    [M,N,color]=size(I1);

    I1_db = padarray(im2double(I1), [0,max_d], 'both');
    I2_db = padarray(im2double(I2), [0,max_d], 'both');

    min_SAD_map = ones(M,N,max_d-min_d+1).*(M*N*color*256);
    D = zeros(M,N);
    
    for k = min_d:max_d
        dif_img = I1_db;
        dif_img(:, (1+k):(N+2*max_d), :)=abs(I1_db(:,(1+k):(N+2*max_d),:)-I2_db(:,(1):(N+2*max_d-k),:));
        
        SAD_map=zeros(M,N+2*max_d);
        for c=1:color
            SAD_map = SAD_map+conv2(dif_img(:,:,c), ones(ws), 'same');
        end
        
        for i=1:M
            for j=1:N
                if SAD_map(i,j+max_d) < min_SAD_map(i,j)
                    min_SAD_map(i,j) = SAD_map(i,j+max_d);
                    D(i,j) = k; 
                end
            end
        end
        
    end
%}
    
%     visualize disparity map
    figure;
    imagesc(D,[min_d max_d]);
    colormap(gray);
    colorbar;
    axis image;

end