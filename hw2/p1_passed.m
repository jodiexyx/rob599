%parameter definition
ms=400;
A=[0 -1;0 0];
B=[0;-1/ms];
F=[1;0];
C=[1 0];

%1.1 determine the observability matrix (O) and its rank (r)
O=[C; C*A]
r=rank(O)
%1.2 Pole placement Design
%1.2.1 Feedback Design. Find gain matrix, K, to place poles at -1,-1 
K=[0 1]/([B A*B])*(A*A+2*A+eye(2))
%1.2.2 Observer Design. Find gain matrix, G, to place poles at -3,-3 
G=(A*A+6*A+9*eye(2))/([C;C*A])*[0;1]

%1.2.3 Euler simulate to find the trajectories for 
%the system (x) and observer (x_hat)
t=0:1/100:10;
x_1=[0.1;0.1];
x_hat_1=[0;0];

w = 0; v = 0;
AA = [A -B*K; G*C A-G*C-B*K];
BB = [w; 0; G*v];
[~,xx] = ode45(@(~,xx) AA*xx+BB, t, [x_1;x_hat_1]); 
xx = xx';

x=xx(1:2,:);
x_hat=xx(3:4,:);

%1.3 LQR Design
%1.3.1 Feedback Design. use LQR to find feedback gains K_lqr
%note to use the lqr_LTV function with a constant A, B matrices, 
%enter @(i)A, @(i)B as the arguments AFun, BFun
Q=100*eye(2);
R=0.00005;

[K_lqr, ~] = lqr_LTV(@(i)A, @(i)B,Q,R,t);


%1.3.2 Observer Design. use LQR to find optimal observer G_lqr
%note to use the lqr_LTV function with a constant A, B matrices, 
%enter @(i)A, @(i)B as the arguments AFun, BFun
Qo=eye(2);
Ro=1;
[G_lqr, ~] = lqr_LTV(@(i)A', @(i)C',Qo,Ro,t);
G_lqr = cellfun(@transpose,G_lqr,'un',0);

%1.3.3 Euler Simulate
x0=[x_1;x_hat_1];
w=normrnd(0,0.0005,1,1001);v=normrnd(0,0.0005,1,1001);
tResult=[]; xResult=[];

for index = 2:numel(t)
% index=2;
    
    AA = [A' -C'*G_lqr{index}'; K_lqr{index}'*B' A'-K_lqr{index}'*B'-C'*G_lqr{index}'];
    BB = [w(index); 0; K_lqr{index}'*v(index)];
    t_mini = t(index-1:index);
    [tt,xx] = ode45(@(~,xx) AA*xx+BB, t_mini, x0); 
    
    xResult = cat(1, xResult, xx(end,:));
    x0 = xResult(end, :);
end
xResult = xResult';
xResult = cat(2, [x_1;x_hat_1], xResult);

x_lqr_1=xResult(2,:);
x_lqr_2=xResult(1,:);
x_lqr=[x_lqr_1;x_lqr_2];
x_hat_lqr_1=xResult(4,:);
x_hat_lqr_2=xResult(3,:);
x_hat_lqr=[x_hat_lqr_1;x_hat_lqr_2];

%1.4 Comparison
%Look at the observer error (x-x_hat) and (x_lqr-x_hat_lqr)

%which observer converges faster? enter 'acker' or 'lqr'
%apostrophes included!
faster_convergence='acker'

%which observer is less noisy? enter 'acker' or 'lqr'
%apostrophes included!
less_noisy='lqr'

function [K, P] = lqr_LTV(AFun,BFun,Q,R,tSpan)
    nSteps = length(tSpan);

    P{nSteps} = zeros(size(Q));
    K{nSteps} = zeros(length(R),length(Q));
    
    for i = nSteps-1:-1:1
        A_ = AFun(i+1);
        B_ = BFun(i+1);
        P_ = P{i+1};
        
        P{i} = P_ + (tSpan(i+1)-tSpan(i)) * ( P_*A_ + A_'*P_ - P_*B_*(R\(B_'*P_)) + Q);
        K{i} = R\(B_'*P_);
    end
end