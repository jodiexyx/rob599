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