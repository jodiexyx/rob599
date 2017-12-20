clc;
clear all;

tic

load ('TestTrack_out.mat');
bl = TestTrack.bl;
br = TestTrack.br;
cline = TestTrack.cline;
%[U,x1]= outputforwat([287,5,-176,0,2,0],cline(:,2),[0,2000]')

plot(bl(1,:),bl(2,:),'k');
hold on;
plot(br(1,:),br(2,:),'k');
hold on;

N=size(bl, 2);

t = 1:1:N;
x_c = cline(1,:);
y_c = cline(2,:);

tq = 1:0.05:246;
cline_nx = interp1(t,x_c,tq,'spline');
cline_ny = interp1(t,y_c,tq,'spline');
cline_nw = cline;%no interpolation

%enlong the final part
enlong_step = 5;
for i=1:enlong_step
    cline_final = cline_nw(:,end);
    cline_final_b_1 = cline_nw(:,end-1);
    cline_final_a_1 = cline_final*2-cline_final_b_1;
    cline_nw = [cline_nw, cline_final_a_1];
end

%plot(x,y,'b');
plot(cline_nw(1,:),cline_nw(2,:),'d');
u = [];
x0 = [287,5,-176,0,2,0];
x_c = x0;
[m,n] = size(cline_nw);
p = 0;
w = 10; %size of window
pro_pool = zeros(1,w);
a = 2;%P*
b = 1;%I*
c = 1;%D*
flag = 0;%just start
count = 0;
F = 0;
k = 4;

u0 = zeros(2,1);
u_c = zeros(2,1);

last_cali_time = 0;
cali_inter = 500;

last_all_cali_time = 0;
all_cali_inter = 10000;

u_before_cali=zeros(2,1);
x_cali = x0;
u_after_cali=-1;

while p <= n-2
    M = [x_c(1)*ones(1,n);x_c(3)*ones(1,n)]-[cline_nw(1,:);cline_nw(2,:)];
    [min_dis,index] = sort(sum(M.*M));
%   norm([x0(1);x0(3)]-cline_nw(:,2))%test
    p_new = index(1) + 2;%objective index
    
    if p_new ~= p
        p=p_new
    else
        p=p_new;
    end
    
    x_obj = cline_nw(:,p);%current objective point
    
    %{
    test_c = cline_nw(:,p:p+k-1);
    grad = (test_c(2,:)-x_c(3)*ones(1,k))./(test_c(1,:)-x_c(1)*ones(1,k));
    dif_grad = diff(grad);
    sum_c = sum(abs(dif_grad));%sum of degree change(of 6 points after)
    sum_grad_max = 660;
    sum_grad_min = 0.03;
    %}
    
    %F = -0.01*sum_c^2 + 500 + 10000/(x_c(2)+0.01);%input F (better be a function of v and sum_c)*
    F = 100;
    F = max(F,-10000);
    F = min(F,5000);
    
    [theta,~] = cart2pol(x_obj(1)-x_c(1),x_obj(2)-x_c(3));
    pro = theta - x_c(5);%minus yaw angle(negative for clockwise)
    pro_pool = [pro_pool(1,2:end), pro];
    int = sum(pro_pool)/w;
    dif = pro_pool(end)-pro_pool(end-1);
    delta = a*pro + b*int + c*dif;
    delta = max(delta,-0.5);
    delta = min(delta,0.5);
    u_new = [delta; F];
    
    %calculate step num
    velocity = x_c(2);
    %allow_step = max(min(round(min_dis(1)/velocity/0.01/2), 20), 1);
    allow_step = 20;
    
    u_newp = repmat(u_new,1,allow_step);%30,50...* number of input every loop(can be a function)
    for i=1:allow_step
       u_newp(1, i)=interp1([0, allow_step],[u_c(1), u_new(1)],i);
       u_newp(2, i)=interp1([0, allow_step],[u_c(2), u_new(2)],i);
    end
    
    u = [u, u_newp];
    %Y = forwardIntegrateControlInput(u');
    %Y = forwardIntegrateControlInput([u_c';u_newp'], x_c);

    if size(u_after_cali, 1) == 1
        u_after_cali = u_newp;
    else
        u_after_cali = [u_after_cali,u_newp];
    end
    Y = forwardIntegrateControlInput([u_c';u_newp'], x_c);
    
    if length(u)-last_cali_time>cali_inter
        last_cali_time = length(u);
        
        if length(u)-last_all_cali_time>all_cali_inter
            last_all_cali_time  = length(u);
            Y = forwardIntegrateControlInput([u0';u']);
        else
            Y = forwardIntegrateControlInput([u_before_cali(:,end)';u_after_cali'], x_cali);
        end
        x_cali = Y(end, :);
        u_before_cali = [u_before_cali, u_after_cali];
        u_after_cali = -1;

    end
    %{
    if mod(p, 40) == 0
        Y = forwardIntegrateControlInput([zeros(1,2);u']);
    else
        Y = forwardIntegrateControlInput([zeros(1,2);u_newp'], x_c);
    end
    %}
    x_c = Y(end,:);
    u_c = u(:,end);
    count = count + 1;
    %x_c(2)
end

ROB599_ControlsProject_part1_input = [u0';u'];
toc

Y =forwardIntegrateControlInput(ROB599_ControlsProject_part1_input);
plot(Y(:,1),Y(:,3),'d','MarkerSize', 2)
hold on;

if exist('Xobs', 'var')
    for i=1:length(Xobs)
        x=[Xobs{i}(:,1);Xobs{i}(1,1)];
        y=[Xobs{i}(:,2);Xobs{i}(1,2)];
        plot(x,y)
        hold on
    end
    
end


%scatter(cline_nw(1,:),cline_nw(2,:),'d')