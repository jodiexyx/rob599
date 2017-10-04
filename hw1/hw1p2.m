%%
A=[-3 0 0;0 -1 -3;0 1 -2];

%2.1
[V,D] = eig(A);
l1=D(1,1);
l2=D(2,2);
l3=D(3,3);

%List eigenvectors such that v1 is the eigenvector for eigenvalue l1.
%Eigenvectors should be in column vector format.
v1=V(:,1);
v2=V(:,2);
v3=V(:,3);

%2.2
%Write out the close form equation using the symbolic variable t as time
syms t
x= expm(A*t)*[5;0;0];

%2.3
f=@(t,x) A*x;
tspan=[0:0.1:10];
x0=[0;1;2];clc


[T,Y]=ode45(f,tspan,x0);

%2.4 
sol24= sum(Y*[1;0;0]*0.1);