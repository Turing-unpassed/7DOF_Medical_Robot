close all;
clear;
clc;

Link = MDH_Table_Build();
Link = Transimation_Matrix_Build(Link);

th1=45.0000; 
th2=41.7585;
th3=117.2310;
d4=0;
th5=90.0000;
th6=248.9896;
th7=135;

q_prev=[th1,th2,th3,d4,th5,th6,th7];

q0 = [150,150,150,0,0,0];
q1 = [450,450,450,360,360,180];
v_max = [1000,1000,1000,1000,1000,1000];
A = [1000,1000,1000,1000,1000,1000];
dt =0.01;

% R = [1    0   0;
%      0    1   0;
%      0    0   1];
% [alpha, beta, gamma] = ZYZ_EulerAngle_Solution(R)
% R = [0    0   -1;
%      1    0    0;
%      0   -1    0];
% [alpha, beta, gamma] = ZYZ_EulerAngle_Solution(R)


[q,~,~]=Tplanning(q0, q1, v_max, A, dt);

step = length(q);

% t = 0:1:step-1;
% figure(1);
% plot(t,q(:,6));

Forward_kinematics(Link,q_prev,0);
pause(2);
% view(0,0);
% hold on;
for i = 1:step
    r11 = cosd(q(i,4))*cosd(q(i,5))*cosd(q(i,6)) - sind(q(i,4))*sind(q(i,6));
    r21 = sind(q(i,4))*cosd(q(i,5))*cosd(q(i,6)) + cosd(q(i,4))*sind(q(i,6));
    r31 = -sind(q(i,5))*cosd(q(i,6));
    r12 = -cosd(q(i,4))*cosd(q(i,5))*sind(q(i,6)) - sind(q(i,4))*cosd(q(i,6));
    r22 = -sind(q(i,4))*cosd(q(i,5))*sind(q(i,6)) + cosd(q(i,4))*cosd(q(i,6));
    r32 = sind(q(i,5))*sind(q(i,6));
    r13 = cosd(q(i,4))*sind(q(i,5));
    r23 = sind(q(i,4))*sind(q(i,5));
    r33 = cosd(q(i,5));

    T = [r11    r12    r13    q(i,1);
         r21    r22    r23    q(i,2);
         r31    r32    r33    q(i,3);
          0      0      0        1  ];

    Q = Geometric_Inverse_Kinematics(Link,T);
    [q_best,~,~] = Select_Optimal_Solution(Q,q_prev);
    q_prev = q_best;
    if i < step
        Forward_kinematics(Link,q_best,1);
        plot3(q(:,1),q(:,2),q(:,3),'r','LineWidth',1);
        hold on;
    else
        Forward_kinematics(Link,q_best,0);
    end
end
