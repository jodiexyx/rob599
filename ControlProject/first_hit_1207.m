%% 
clc; clear

%% read track
load TestTrack.mat
track_left_boundary = TestTrack.bl;
track_right_boundary = TestTrack.br;
track_center = TestTrack.cline;
track_orientation = TestTrack.theta;

%% plot track
close
figure(1)
set(gcf,'units','points','position',[100,100,800,600])
plot(track_center(1,:),track_center(2,:),'--','MarkerSize', 5)
hold on
plot(track_right_boundary(1,:),track_right_boundary(2,:)) % ,'>','MarkerSize', 2
plot(track_left_boundary(1,:),track_left_boundary(2,:)) %,'<','MarkerSize', 2
title('Test Track')
s_input = [];
f_input = [];

% steer: [-0.5,0.5], Fx: [-10000,5000]
N1 = 400;
steer_1 = -0.005*ones(N1,1);
force_1 = 5000*ones(N1,1);
s_input = [s_input; steer_1];
f_input = [f_input; force_1];

N2 = 400;
steer_2 = -0.005*ones(N2,1);
force_2 = -5000*ones(N2,1);
s_input = [s_input; steer_2];
f_input = [f_input; force_2];

N3 = 1950;
steer_3 = -0.025*ones(N3,1);
force_3 = 300*ones(N3,1);
s_input = [s_input; steer_3];
f_input = [f_input; force_3];

N4 = 1500;
steer_4 = 0.*ones(N4,1);
force_4 = 100*ones(N4,1);
s_input = [s_input; steer_4];
f_input = [f_input; force_4];

N5 = 800;
steer_5 = 0.025*ones(N5,1);
force_5 = 100*ones(N5,1);
s_input = [s_input; steer_5];
f_input = [f_input; force_5];

N6 = 700;
steer_6 = -0.04*ones(N6,1);
force_6 = 100*ones(N6,1);
s_input = [s_input; steer_6];
f_input = [f_input; force_6];

N7 = 800;
steer_7 = 0.04*ones(N7,1);
force_7 = 100*ones(N7,1);
s_input = [s_input; steer_7];
f_input = [f_input; force_7];

N8 = 200;
steer_8 = 0*ones(N8,1);
force_8 = 100*ones(N8,1);
s_input = [s_input; steer_8];
f_input = [f_input; force_8];

N9 = 550;
steer_9 = -0.035*ones(N9,1);
force_9 = 100*ones(N9,1);
s_input = [s_input; steer_9];
f_input = [f_input; force_9];

N10 = 200;
steer_10 = 0*ones(N10,1);
force_10 = 100*ones(N10,1);
s_input = [s_input; steer_10];
f_input = [f_input; force_10];

N11 = 750;
steer_11 = -0.028*ones(N11,1);
force_11 = 100*ones(N11,1);
s_input = [s_input; steer_11];
f_input = [f_input; force_11];

N = 100;
steer = -0.1*ones(N,1);
force = 100*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 800;
steer = 0*ones(N,1);
force = 150*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 200;
steer = 0.035*ones(N,1);
force = -1000*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 200;
steer = 0.5*ones(N,1);
force = 500*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 500;
steer = 0.01*ones(N,1);
force = 500*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 280;
steer = 0.03*ones(N,1);
force = 600*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 500;
steer = -0.045*ones(N,1);
force = 100*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 500;
steer = -0.06*ones(N,1);
force = 100*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 100;
steer = 0.03*ones(N,1);
force = -2500*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 315;
steer = 0.15*ones(N,1);
force = 500*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 1200;
steer = 0*ones(N,1);
force = 500*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];


N = 200;
steer = 0.03*ones(N,1);
force = -4000*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 194;
steer = 0.1*ones(N,1);
force = 500*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 700;
steer = 0*ones(N,1);
force = 5000*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 200;
steer = 0.002*ones(N,1);
force = -5000*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];

N = 200;
steer = 0.1*ones(N,1);
force = 500*ones(N,1);
s_input = [s_input; steer];
f_input = [f_input; force];







ROB599_ControlsProject_part1_input =[s_input f_input];
x0=[287,5,-176,0,2,0];

Y =forwardIntegrateControlInput(ROB599_ControlsProject_part1_input, x0);
plot(Y(:,1),Y(:,3),'d','MarkerSize', 2)

trajectory = [Y(:,1),Y(:,3)];
checkTrajectory(trajectory,ROB599_ControlsProject_part1_input)



