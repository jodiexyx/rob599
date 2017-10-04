%parameter definition
m=1500;
n=4.8;
r=0.4;
g=9.81;
C_r=0.01;
rho=1.3;
C_d=0.32;
a=2.4;
theta_e=2*pi/180;
v_e=20;

%you may enter equations, but do not change the names of the variables given. 
%The auto-grader uses these variable names to check your answers!

%Part 1.1 Determine equilibrium engine torque
u_e=(m*g*C_r*sign(v_e)+1/2*rho*C_d*a*v_e^2+m*g*sin(theta_e))*r/n;

%Part 1.2.1 Determine the A,B,F matrices for the linearized system
A=-rho*C_d*a*v_e/m;
B=n/(m*r);
F=-g*cos(theta_e);

%Part 1.2.2 Determine feedback gains of linear system to place pole at -1
k=inv(B)*(A+1);

%Part 1.2.3 Determine the steady state error after 10 seconds
tspan=[0:0.01:10];
x0=-1;
[t,x1]=ode45(@(t,x) [(A-B*k)*x+F*0], tspan, x0);
[t,x2]=ode45(@(t,x) [(A-B*k)*x+F*3/180*pi], tspan, x0);
sse=x2(1001)-x1(1001);

%Part 1.3.1 Determine A_I,B_I,F_I matrices of the linear system with integral action
%the subscript I is just used to indicate these matrices and vectors apply 
A_I=[A,0;1,0];
B_I=[B;0];
F_I=[F;0];

%Part 1.3.2 Place poles of the system with integral action at -1,-2
k_I=[0,1]*inv([B_I A_I*B_I])*(A_I^2+3*A_I+2*eye(2));

%Part 1.3.3 with integral action determine the steady state error after 10
%seconds
z0=[-1;0];
[t,z]=ode45(@(t,z) [(A_I-B_I*k_I)*z+F_I*3/180*pi], tspan, z0);
sse_with_integral_action=z(1001);