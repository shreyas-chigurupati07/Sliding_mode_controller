clc; clear ; close all

%Initialising the points given for the drone.
p0 = [0,0,0];
p1 = [0,0,1];
p2 = [1,0,1];
p3 = [1,1,1];
p4 = [0,1,1];
p5 = [0,0,1];

tr1 = [];
tr2 = [];
tr3 = [];
tr4 = [];
tr5 = [];


x_traj1 = [];
y_traj1 = [];
z_traj1 = [];

vel1 = [];
vel2 = [];
vel3 = [];
vel4 = [];
vel5 = [];

%Initialising the time for the trajectory generation
t0 = 0;
tf = 15;
tfP = 5;
syms t

% Qunitic Trajectory generation 
%Point 0 to 1
qp1 = QunticTraj(t0,tfP,p0,p1)
qp1_d = diff(qp1,t)
qp1_dd = diff(qp1,t,2)
tfP = 15;

%Point 1 to 2
qp2 = QunticTraj(t0,tfP,p1,p2)
qp2_d = diff(qp2,t)
qp2_dd = diff(qp2,t,2)

%Point 2 to 3
qp3 = QunticTraj(t0,tfP,p2,p3)
qp3_d = diff(qp3,t)
qp3_dd = diff(qp3,t,2)

%Point 3 to 4
qp4 = QunticTraj(t0,tfP,p3,p4)
qp4_d = diff(qp4,t)
qp4_dd = diff(qp4,t,2)

%Point 4 to 5
qp5 = QunticTraj(t0,tfP,p4,p5)
qp5_d = diff(qp5,t)
qp5_dd = diff(qp5,t,2)

t1 = 0:0.5:5;
t2 = 0:0.1:15;

for i = 1:length(t1)
    tr_1 = [0, 0, 6*t1(i)^5/3125 - 3*t1(i)^4/125 + 2*t1(i)^3/25];
    tr1 = [tr1;tr_1];
    
    qp1_d = [0, 0, 0.2400*t1(i)^2 - 0.0960*t1(i)^3 + 0.0096*t1(i)^4];
    vel1 = [vel1;qp1_d];

end

for i = 1:length(t2)

    tr_2 = [4664065662093477*t2(i)^5/590295810358705651712 - t2(i)^4/3375 + (2*t2(i)^3)/675, 0, 1];
    tr2 = [tr2;tr_2];

    
    qp2_d = [0.0089*t2(i)^2 - 0.0012*t2(i)^3 + 3.9506e-05*t2(i)^4, 0, 0];
    vel2 = [vel2;qp2_d];

    tr_3   = [1, 4664065662093477*t2(i)^5/590295810358705651712 - t2(i)^4/3375 + 2*t2(i)^3/675, 1];
    tr3 = [tr3;tr_3];

    qp3_d =     [0, 0.0089*t2(i)^2 - 0.0012*t2(i)^3 + 3.9506e-05*t2(i)^4, 0];
    vel3 =[vel3;qp3_d];

    tr_4 =  [- (4664065662093477*t2(i)^5)/590295810358705651712 + t2(i)^4/3375 - (2*t2(i)^3)/675 + 1, 1, 1];
    tr4 = [tr4;tr_4];

    qp4_d = [- 0.0089*t2(i)^2 + 0.0012*t2(i)^3 - 3.9506e-05*t2(i)^4, 0, 0];
    vel4 = [vel4;qp4_d];

    tr_5 = [0, - (4664065662093477*t2(i)^5)/590295810358705651712 + t2(i)^4/3375 - (2*t2(i)^3)/675 + 1, 1];
    tr5 = [tr5; tr_5];

    qp5_d = [0, - 0.0089*t2(i)^2 + 0.0012*t2(i)^3 - 3.9506e-05*t2(i)^4, 0];
    vel5 = [vel5;qp5_d];
end

time = [0:0.1057:65];
x_traj1 = [tr1(:,1);tr2(:,1);tr3(:,1);tr4(:,1);tr5(:,1)];
y_traj1 = [tr1(:,2);tr2(:,2);tr3(:,2);tr4(:,2);tr5(:,2)];
z_traj1 = [tr1(:,3);tr2(:,3);tr3(:,3);tr4(:,3);tr5(:,3)];

dtime=65;
[T,X]=ode45(@ode_quadrotor,[0:0.1:65],[0;0;0;0;0;0;0;0;0;0;0;0]);

%Plotting the desired trajectories
p = plot3(tr1(:,1),tr1(:,2),tr1(:,3));
p.LineWidth = 1.5;
hold on;
plot3(tr2(:,1),tr2(:,2),tr2(:,3));
p.LineWidth = 1.5;

p = plot3(tr3(:,1),tr3(:,2),tr3(:,3));
p.LineWidth = 1.5;

p = plot3(tr4(:,1),tr4(:,2),tr4(:,3));
p.LineWidth = 1.5;

x = plot3(tr5(:,1),tr5(:,2),tr5(:,3));
x.LineWidth = 1.5;
xlabel('X')
ylabel('Y')
zlabel('Z')
hold off;

figure('Name','States XYZ')
subplot(2,2,1)
plot(time,x_traj1,'b','LineWidth',2);
xlabel('t');
ylabel('X');

subplot(2,2,2)
plot(time,y_traj1,'b','LineWidth',2);
xlabel('t');
ylabel('Y');

subplot(2,2,3)
plot(time,z_traj1,'b','LineWidth',2);
xlabel('t');
ylabel('Z');



function r = QunticTraj(t0,tf,p0,pf)
    syms t
    A = [1 t0 t0^2 t0^3 t0^4 t0^5;
            0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
            0 0 2 6*t0 12*t0^2 20*t0^3;
            1 tf tf^2 tf^3 tf^4 tf^5;
            0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
            0 0 2 6*tf 12*tf^2 20*tf^3];
        
    b = [inv(A)*[p0(1);0;0;pf(1);0;0], inv(A)*[p0(2);0;0;pf(2);0;0], inv(A)*[p0(3);0;0;pf(3);0;0]];
       
    a = [1 t t^2 t^3 t^4 t^5];
    
    r = a*b;
end
