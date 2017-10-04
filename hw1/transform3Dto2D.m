function pnt_2D = transform3Dto2D(ws),
    % inputs: ws is the dimension of the world space, ws x ws
    % init hourglass points with scale = 100, grid_grain = 10
    [hourglass_points] = gen_hourglass(100,10); % hourglass_points is 441*4
    trans = eye(4);
    %1. Translate the rectangle by 250 mm along the world x axis and 250 mm along the world yaxis.
    T1 = [1,0,0,250;0,1,0,250;0,0,1,0;0,0,0,1];
    %2. Rotate the rectangle 105 degrees counterclockwise about the world y axis and scale by afactor of 50 in the world x direction and 0.25 in the world z direction.
    T2_rot = [cos(105/180*pi),0,sin(105/180*pi),0;0,1,0,0;-sin(105/180*pi),0,cos(105/180*pi),0;0,0,0,1];
    T2_scale = [50,0,0,0;0,1,0,0;0,0,0.25,0;0,0,0,1];
    T2 = T2_scale * T2_rot;
    %3. Translate the rectangle by 350 mm along the current y axis.
    T3 = [1,0,0,0;0,1,0,350;0,0,1,0;0,0,0,1];
    %4. Rotate the rectangle by 45 degrees counterclockwise about the current x axis.
    ref0 = T3*T2*T1*[0;0;0;1];
    n = T3*T2_rot*T1*[1;0;0;0];
    K = [0, -n(3), n(2); n(3), 0, -n(1); -n(2), n(1), 0];
    R = eye(3) + sin(0.25*pi) * K + (1 - cos(0.25*pi)) * K^2;
    T4 = [eye(3), ref0(1:3); zeros(1, 3), 1] * [R, zeros(3, 1); zeros(1, 3), 1] * [eye(3), -ref0(1:3); zeros(1, 3), 1];
    %5. Project from the fixed 3D world frame to 2D image plane defined by the perspective matrixP and intrinsic matrix I.
    trans = T4*T3*T2*T1;
    trans_points = trans * hourglass_points';
    trans_points = [1,0,0,0;0,1,0,0;0,0,1,0] * trans_points;
    world_pix_mat = trans_points./trans_points(3,:);
    max_col = max(world_pix_mat,[],2);
    min_col = min(world_pix_mat,[],2);
    scale = (ws-1)./(max_col-min_col);
    pnt_2D = (world_pix_mat-min_col).*scale;
    pnt_2D(3,:)=1;
end