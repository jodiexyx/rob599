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
cline_nw = cline;%no interpolation

%plot(x,y,'b');
plot(cline_nw(1,:),cline_nw(2,:),'d');
u = [];
x0 = [287,5,-176,0,2,0];
x_c = x0;
[m,n] = size(cline_nw);
p = 0;
w = 10; %size of window
pro_pool = zeros(1,w);
a = 1;
b = 0;
c = 1;
flag = 0;%just start
count = 0;
F = 1000;

sum_grad = [];
k = 3;

for i = 1:n-k-1
    test_c = cline_nw(:,i+2:i+k+1);
    grad = (test_c(2,:)-cline_nw(2,i)*ones(1,k))./(test_c(1,:)-cline_nw(1,i)*ones(1,k));
    dif_grad = diff(grad);
    sum_i = sum(abs(dif_grad));
    sum_grad = [sum_grad, sum_i];
end

[max_s,idx] = max(sum_grad)
[min_s,idx] = min(sum_grad)
