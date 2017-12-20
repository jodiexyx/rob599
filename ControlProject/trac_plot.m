load TestTrack.mat
track_left_boundary = TestTrack.bl;
track_right_boundary = TestTrack.br;
track_center = TestTrack.cline;
track_orientation = TestTrack.theta;
Nobs = 25;
Xobs = generateRandomObstacles(Nobs,TestTrack);


close
figure(1)
set(gcf,'units','points','position',[100,100,800,600])
%plot(track_center(1,:),track_center(2,:),'^','MarkerSize', 5)
hold on
%plot(track_right_boundary(1,:),track_right_boundary(2,:)) % ,'>','MarkerSize', 2
%plot(track_left_boundary(1,:),track_left_boundary(2,:)) %,'<','MarkerSize', 2

for i=1:Nobs
    x=[Xobs{i}(:,1);Xobs{i}(1,1)];
    y=[Xobs{i}(:,2);Xobs{i}(1,2)];
    plot(x,y)
    hold on
end

%generate extra point

N=size(track_center, 2);

left_bound = TestTrack.bl;
right_bound = TestTrack.br;
center_line = TestTrack.cline;
fixed_point = zeros(1, N);

for i=1:Nobs
    obs_center = mean(Xobs{i}, 1)';

    while(1)
        point_dis = sum((repmat(obs_center, 1, N)-center_line).^2, 1)+999999999.*fixed_point;

        [dis_value, idx] = sort(point_dis);
    
        obs_dis_gate = max(sum((repmat(obs_center, 1, 4)-Xobs{i}').^2, 1));
        
        if(dis_value(1)<obs_dis_gate*2)
            %remove point
            left_bound = [left_bound(:, 1:(idx(1)-1)), left_bound(:, (idx(1)+1):N)];
            right_bound = [right_bound(:, 1:(idx(1)-1)), right_bound(:, (idx(1)+1):N)];
            center_line = [center_line(:, 1:(idx(1)-1)), center_line(:, (idx(1)+1):N)];
            fixed_point = [fixed_point(1:(idx(1)-1)), fixed_point((idx(1)+1):N)];
            N=N-1;
        else
            break
        end
    
    end
    idx_idx_1 = 1;
    idx_idx_2 = 2;
    while((obs_center-center_line(:, idx(idx_idx_1)))'*(obs_center-center_line(:, idx(idx_idx_2)))>0)
        idx_idx_2 = idx_idx_2+1;
    end
    
    min_idx = min(idx(idx_idx_1), idx(idx_idx_2));
    max_idx = max(idx(idx_idx_1), idx(idx_idx_2));
    
    rate = norm(obs_center-center_line(:, min_idx))/norm(center_line(:, max_idx)-center_line(:, min_idx));
    
    new_bl = left_bound(:, min_idx) + rate*(left_bound(:, max_idx)-left_bound(:, min_idx));
    new_br = right_bound(:, min_idx) + rate*(right_bound(:, max_idx)-right_bound(:, min_idx));
    left_center = new_bl + 0.05*(new_br-new_bl);
    right_center = new_bl + 0.95*(new_br-new_bl);
    
    new_center = left_center;
    
    if norm(left_center-obs_center)<norm(right_center-obs_center)
        new_center = right_center;
    end
    
    left_bound = [left_bound(:, 1:min_idx), new_bl, left_bound(:, (min_idx+1):N)];
    right_bound = [right_bound(:, 1:min_idx), new_br, right_bound(:, (min_idx+1):N)];
    center_line = [center_line(:, 1:min_idx), new_center, center_line(:, (min_idx+1):N)];
    fixed_point = [fixed_point(1:min_idx), 1, fixed_point((min_idx+1):N)];
    N = N + 1;
end

%change start/end point to fixed
%{
left_bound = [left_bound(:, 1), left_bound];
right_bound = [right_bound(:, 1), right_bound];
center_line = [[287; -176], center_line];
fixed_point = [1, fixed_point];
N = N + 1;
%}

fixed_point(1:2)=1;
fixed_point(end)=1;

%calculate path cost
GRID = 11;
GRID_EDGE_RATE = 0.2;

path_cost = ones(N, GRID, GRID)*Inf;
path_from = ones(N, GRID, GRID, 2)*Inf;

path_cost(1, 1, :) = 0;
path_from(1, 1, :) = 0;

path_cost(2, 1, 1) = 0;
path_from(2, 1, 1, :) = [1;1];


for i=3:N
    for j=1:GRID
        if fixed_point(i)==1
            if j==1
                current_point = center_line(:, i);
            else
                break
            end
        else
            current_point = get_grid_point(left_bound(:, i), right_bound(:, i), j, GRID, GRID_EDGE_RATE);
        end
        
        for k=1:GRID
            if fixed_point(i-1)==1
                if k==1
                    prev_point = center_line(:, i-1);
                else
                    break
                end
            else
                prev_point = get_grid_point(left_bound(:, i-1), right_bound(:, i-1), k, GRID, GRID_EDGE_RATE);
            end

            for l=1:GRID
                if fixed_point(i-2)==1
                    if l==1
                        old_point = center_line(:, i-2);
                    else
                        break
                    end
                else
                    old_point = get_grid_point(left_bound(:, i-2), right_bound(:, i-2), l, GRID, GRID_EDGE_RATE);
                end
                
                %valid transfer here
                %{
                if ((current_point-prev_point)'*(prev_point-old_point)/norm(current_point-prev_point)/norm(prev_point-old_point))<0.3
                    continue
                end
                %}
                if path_cost(i, j, k) > (path_cost(i-1, k, l) + norm(current_point-prev_point))
                    path_cost(i, j, k) = path_cost(i-1, k, l) + norm(current_point-prev_point);
                    path_from(i, j, k, :) = [k, l];
                end
            end
        end
    end
end

%give out path
path_line = zeros(2, N);
[~,current_pos] = min(path_cost(N, :));

current_pos_2d = [mod(current_pos-1, GRID)+1, ceil(current_pos/GRID)];

for i=N:-1:1
    if fixed_point(i) == 1
        path_line(:, i) = center_line(:, i);
        current_pos_2d(:) = path_from(i, 1, current_pos_2d(2), :);
    else
        path_line(:, i) = get_grid_point(left_bound(:, i), right_bound(:, i), current_pos_2d(1), GRID, GRID_EDGE_RATE);
        current_pos_2d(:) = path_from(i, current_pos_2d(1), current_pos_2d(2), :);
    end
end

%bezier curve for obs
fixed_point(1:2)=0;
fixed_point(end)=0;

current_idx = 1;
INTERP_NUM = 5;
while(current_idx <= length(path_line))
    if fixed_point(current_idx) == 1
        P0 = path_line(:, current_idx-1);
        P1 = path_line(:, current_idx);
        P2 = path_line(:, current_idx+1);
       
        P0_l = left_bound(:, current_idx-1);
        P0_r = right_bound(:, current_idx-1);
        P2_l = left_bound(:, current_idx+1);
        P2_r = right_bound(:, current_idx+1);
       
        T=linspace(0,1,INTERP_NUM+2);
        
        inter_line = zeros(2, INTERP_NUM);
        inter_lb = zeros(2, INTERP_NUM);
        inter_rb = zeros(2, INTERP_NUM);
        
        
        for i=2:(INTERP_NUM+1)
           t = T(i); 
           
           now_point = (1-t)^2*P0 +2*t*(1-t)*P1 + (t)^2*P2;
           now_l = P0_l + t*(P2_l-P0_l);
           now_r = P0_r + t*(P2_r-P0_r);
           
           inter_line(:, i-1) = now_point;
           inter_lb(:, i-1) = now_l;
           inter_rb(:, i-1) = now_r;
        end
       
        left_bound = [left_bound(:, 1:(current_idx-1)), inter_lb, left_bound(:, (current_idx+1):N)];
        right_bound = [right_bound(:, 1:(current_idx-1)), inter_rb, right_bound(:, (current_idx+1):N)];
        path_line = [path_line(:, 1:(current_idx-1)), inter_line, path_line(:, (current_idx+1):N)];
        fixed_point = [fixed_point(1:(current_idx-1)), zeros(1, INTERP_NUM), fixed_point((current_idx+1):N)];

        N = N - 1 + INTERP_NUM;
        assert(N==size(left_bound, 2));
        
        current_idx = current_idx + INTERP_NUM;
       
   else
       current_idx = current_idx+1;
   end
end



%print new path
plot(path_line(1,:),path_line(2,:),'^','MarkerSize', 5)
plot(left_bound(1,:),left_bound(2,:)) % ,'>','MarkerSize', 2
plot(right_bound(1,:),right_bound(2,:)) %,'<','MarkerSize', 2


TestTrack = struct();
TestTrack.bl = left_bound;
TestTrack.br = right_bound;
TestTrack.cline = path_line;
TestTrack.theta = zeros(1, N);

save('TestTrack_out.mat', 'TestTrack', 'Xobs');

function grid_point = get_grid_point(lb, rb, grid_id, GRID, GRID_EDGE_RATE)
    grid_point = lb + ((grid_id-1)/(GRID-1)*(1-2*GRID_EDGE_RATE)+GRID_EDGE_RATE) * (rb-lb);
end
