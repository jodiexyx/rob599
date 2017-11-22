%1.1
Ac = [0,0.310625;0,0];
Bc = [0;1];
dt = 0.01;
A = Ac*dt + eye(2);
B = Bc*dt;

%1.2
PredHorizon = 10;
Ndec=2*(PredHorizon+1)+PredHorizon;

%1.3
Aeq=zeros(2+2*PredHorizon, Ndec);
beq=zeros(2+2*PredHorizon, 1);
Aeq(1:2,1:2)=eye(2);
beq(1)=-1.25-(-1);

for i=1:PredHorizon
    Aeq((2*i+1):(2*i+2),(2*i-1):(2*i)) = A;
    Aeq((2*i+1):(2*i+2),(2*i+1):(2*i+2)) = -eye(2);
    Aeq((2*i+1):(2*i+2),2*PredHorizon+i+2)=B;
end

%1.4
ut = @(t)-0.3175*sin(pi*t/10-pi/2);
Au = eye(Ndec);
Al = eye(Ndec);
Aineq = [Au;-Al];
bu = zeros(Ndec,1);
bu(1:(2+2*PredHorizon)) = 0.5;
bl = zeros(Ndec,1);
bl(1:(2+2*PredHorizon)) = -0.5;
for j = (3+2*(PredHorizon)):Ndec
    bu(j) = 10 - ut((j-2-2*PredHorizon)*dt);
    bl(j) = -10 - ut((j-2-2*PredHorizon)*dt);
end
bineq = [bu;-bl];

%1.5
x_normal = zeros(1002,2);
x_normal(1,1) = -1;
for i=1:1001
    x_normal(i+1,:) = (A*x_normal(i,:)'+B*ut((i-1)*dt))';
end

x = zeros(1001,2);
x(1,1) = -1.25;
Aeq_ = Aeq;
Aineq_ = Aineq;
beq_ = beq;
bineq_ = bineq;
for i = 1:1000
    beq_(1) = x(i,1) - x_normal(i,1);
    beq_(2) = x(i,2) - x_normal(i,2);
    
    bu_ = zeros(Ndec,1);
    bl_ = zeros(Ndec,1);
    bu_(1:(2+2*(PredHorizon))) = 0.5;
    bl_(1:(2+2*(PredHorizon))) = -0.5;
    for j = (3+2*(PredHorizon)):Ndec
        bu_(j) = 10 - ut((i+j-3-2*PredHorizon)*dt);
        bl_(j) = -10 - ut((i+j-3-2*PredHorizon)*dt);
    end
    bineq_ = [bu_;-bl_];
    H = zeros(32,32);
    H(1:22,1:22) = 100*eye(22);
    c = zeros(32,1);
    z = quadprog(H,c,Aineq_,bineq_,Aeq_,beq_);
    
    x(i+1,:) = z(3:4)'+x_normal(i+1,:);
end