
Kr = 2;
Kt = 3;


for i = 1:Nobs
    ini_mark = 1;
    ob_center(i,:) = sum(Xobs{i})/4;
    ob_radius(i) = sqrt( (ob_center(i,1) - Xobs{i}(1,1))^2 + (ob_center(i,2) - Xobs{i}(1,2))^2) * Kr;
    for j = 1:246
        ob_track_distance(i,j) = sqrt( (ob_center(i,1) - track_center(1,j))^2 + (ob_center(i,2) - track_center(2,j))^2);
           if ob_track_distance(i,j) < ob_radius(i)
               translation = ob_radius(i) / ob_track_distance(i,j) * Kt;
%                x_tras = abs(translation * sin(track_orientation(j)));
%                y_tras = abs(translation * cos(track_orientation(j)));
%                x_direction = sign( ob_center(i,1) - track_center(1,j));
%                y_direction = sign( ob_center(i,2) - track_center(2,j));
%                track_center(1,j) = track_center(1,j) - x_tras * x_direction;
%                track_center(2,j) = track_center(2,j) + y_tras * y_direction;
              


               number_of_center_inrange(i,ini_mark) = j;
               ini_mark = ini_mark + 1;
           end
    end
end


% close
 figure(2)
set(gcf,'units','points','position',[100,100,800,600])
plot(track_center(1,:),track_center(2,:),'^','MarkerSize', 5)
hold on
plot(track_right_boundary(1,:),track_right_boundary(2,:)) % ,'>','MarkerSize', 2
plot(track_left_boundary(1,:),track_left_boundary(2,:)) %,'<','MarkerSize', 2

for i=1:Nobs
    x=[Xobs{i}(:,1);Xobs{i}(1,1)];
    y=[Xobs{i}(:,2);Xobs{i}(1,2)];
    plot(x,y)
    hold on
end
