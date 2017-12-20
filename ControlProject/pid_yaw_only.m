clc;
clear all;
load ('TestTrack.mat');
bl = TestTrack.bl;
br = TestTrack.br;
cline = TestTrack.cline;
%[U,x1]= outputforwat([287,5,-176,0,2,0],cline(:,2),[0,2000]')

%plot(bl(1,:),bl(2,:),'b');
%hold on;
%plot(br(1,:),br(2,:),'b');
%hold on;

plot(bl(1,:),bl(2,:),'k');
hold on;
plot(br(1,:),br(2,:),'k');
hold on;

t = 1:1:246;
x_c = cline(1,:);
y_c = cline(2,:);

tq = 1:0.05:246;
cline_nx = interp1(t,x_c,tq,'spline');
cline_ny = interp1(t,y_c,tq,'spline');
cline_nw = [cline_nx;cline_ny];

%plot(x,y,'b');
plot(cline_nx,cline_ny,'d');
u = [];
x0 = [287,5,-176,0,2,0];
x_c = x0;
[m,n] = size(cline_nx);
p = 0;
w = 10; %size of window
pro_pool = zeros(1,w);
a = 1;
b = 0;
c = 1;
flag = 0;%just start
count = 0;

while p <= 1000
    M = [x_c(1)*ones(1,n);x_c(3)*ones(1,n)]-[cline_nw(1,:);cline_nw(2,:)];
    [~,index] = sort(sum(M.*M));
    x_c
%   norm([x0(1);x0(3)]-cline_nw(:,2))%test
    p = index(1) + 20%p??????????
    x_obj = cline_nw(:,p);%current objective point
    [theta,~] = cart2pol(x_obj(1)-x_c(1),x_obj(2)-x_c(3));
    F = 300;% just set F this way
    pro = theta - x_c(5);%minus yaw angle(negative for clockwise)
    pro_pool = [pro_pool(1,2:end), pro];
    int = sum(pro_pool)/w;
    dif = pro_pool(end)-pro_pool(end-1);
    delta = a*pro + b*int + c*dif;
    delta = max(delta,-0.5);
    delta = min(delta,0.5);
    u_new = [delta; F];
    u_newp = repmat(u_new,1,30);
    u = [u, u_newp];
    Y = forwardIntegrateControlInput(u');
    x_c = Y(end,:);
    count = count + 1;
end

ROB599_ControlsProject_part1_input = u';
Y =forwardIntegrateControlInput(ROB599_ControlsProject_part1_input);
plot(Y(:,1),Y(:,3),'d','MarkerSize', 2)
hold on;
