Ac = [0 0.310625; 0 0];
Bc = [0;1];
dt = 0.01;

A = Ac*dt + eye(2)
B = Bc*dt

ds=0.1;

Ndec=11*2+10

Aeq=zeros(2+10*2, Ndec);
beq=zeros(2+10*2, 1);

%initial
Aeq(1,1)=1;
beq(1)=-1.25-(-1);

Aeq(2,2)=1;
beq(2)=0;

%dynamics
for k=0:9
    Aeq((3+k*2):(3+k*2+1), (k*2+1):(k*2+2))=A;
    Aeq((3+k*2):(3+k*2+1), (3+k*2):(3+k*2+1))=-eye(2);
    Aeq((3+k*2):(3+k*2+1), 23+k)=B;
end

Aineq=zeros(Ndec*2, Ndec);
bineq=zeros(Ndec*2, 1);

for k=0:10
    %xi<=0.5
    Aineq(2*k+1, 2*k+1)=1;
    bineq(2*k+1)=0.5;
    %yi<=0.5
    Aineq(2*k+2, 2*k+2)=1;
    bineq(2*k+2)=0.5;
    %xi>=-0.5
    Aineq(Ndec+2*k+1, 2*k+1)=-1;
    bineq(Ndec+2*k+1)=-(-0.5);
    %yi>=-0.5
    Aineq(Ndec+2*k+2, 2*k+2)=-1;
    bineq(Ndec+2*k+2)=-(-0.5);
end

ut=-0.3175*sin(pi*0/10-pi/2);

for k=0:9
    
    %ui<=10-ut
    Aineq(22+k+1, 22+k+1)=1;
    bineq(22+k+1)=10-ut;
    %ui>=-10-ut
    Aineq(Ndec+22+k+1, 22+k+1)=-1;
    bineq(Ndec+22+k+1)=-(-10-ut); 
end
    
