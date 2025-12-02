close all;
clear;
clc;

Link = MDH_Table_Build();
Link = Transimation_Matrix_Build(Link);

th1 = 89.6218;
th2 = 273.1024;
th3 = 16.6555;
d4 = 0;
th5 = 0;
th6 = 70.2421;
th7 = 270.3782;

q_prev=[th1,th2,th3,d4,th5,th6,th7];

q0 = [151,151,151,0,0,0];
q1 = [450,450,450,0,0,180];
v_max = [1000,1000,1000,1000,1000,1000];
A = [1000,1000,1000,1000,1000,1000];
dt =0.01;

x = 151:1:450;
y = 151:1:450;
z = 151:1:450;

[q,~,~]=Tplanning(q0, q1, v_max, A, dt);

step = length(q);

Forward_kinematics(Link,q_prev,0);
pause(1);
% view(0,0);
% hold on;
for i = 1:step
    T = [cosd(q(i,6)),-sind(q(i,6)),  0,  q(i,1);
         sind(q(i,6)), cosd(q(i,6)),  0,  q(i,2);
             0,             0,        1,  q(i,3);
             0,             0,        0,     1  ];
    Q = Geometric_Inverse_Kinematics(Link,T);
    [q_best,~,~] = Select_Optimal_Solution(Q,q_prev)
    if i < step
        Forward_kinematics(Link,q_best,1);
        plot3(x,y,z,'r','LineWidth',1);
        hold on;
    else
        Forward_kinematics(Link,q_best,0);
    end
end