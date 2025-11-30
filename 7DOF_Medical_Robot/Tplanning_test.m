close all;
clear;
clc;


Link = MDH_Table_Build();
Link = Transimation_Matrix_Build(Link);
% 采样时间间隔 (50ms)
dt = 0.05;

% 初始位置 [度数/毫米]
q0 = struct('th1', 0, 'th2', 0, 'th3', 0, 'd4', 0, 'th5', 0, 'th6', 0, 'th7', 0);

% 目标位置 [度数/毫米]
qf = struct('th1', 30, 'th2', 30, 'th3', 30, 'd4', 10, 'th5', 30, 'th6', 30, 'th7', 30);

% 各关节最大速度 [度/秒 或 mm/秒]
v_max = struct('th1', 20, 'th2', 20, 'th3', 20, 'd4', 10, 'th5', 20, 'th6', 20, 'th7', 20);

% 各关节最大加速度 [度/秒² 或 mm/秒²]
a_max = struct('th1', 25, 'th2', 25, 'th3', 25, 'd4', 15, 'th5', 25, 'th6', 25, 'th7', 25);

[Q1,V1,ACC1]=Tplanning(q0.th1, qf.th1, v_max.th1, a_max.th1, dt);
[Q2,V2,ACC2]=Tplanning(q0.th2, qf.th2, v_max.th2, a_max.th2, dt);
[Q3,V3,ACC3]=Tplanning(q0.th3, qf.th3, v_max.th3, a_max.th3, dt);
[Q4,V4,ACC4]=Tplanning(q0.d4, qf.d4, v_max.d4, a_max.d4, dt);
[Q5,V5,ACC5]=Tplanning(q0.th5, qf.th5, v_max.th5, a_max.th5, dt);
[Q6,V6,ACC6]=Tplanning(q0.th6, qf.th6, v_max.th6, a_max.th6, dt);
[Q7,V7,ACC7]=Tplanning(q0.th7, qf.th7, v_max.th7, a_max.th7, dt);

% 取最长的轨迹长度作为总步数
N = max([length(Q1), length(Q2), length(Q3), length(Q4), length(Q5), length(Q6), length(Q7)]);

% 插值各关节轨迹，确保所有关节在相同时间内到达目标位置
for i = 1:N
    Q = [
        (i <= length(Q1)) * Q1(min(i, length(Q1))) + (i > length(Q1)) * qf.th1;
        (i <= length(Q2)) * Q2(min(i, length(Q2))) + (i > length(Q2)) * qf.th2;
        (i <= length(Q3)) * Q3(min(i, length(Q3))) + (i > length(Q3)) * qf.th3;
        (i <= length(Q4)) * Q4(min(i, length(Q4))) + (i > length(Q4)) * qf.d4;
        (i <= length(Q5)) * Q5(min(i, length(Q5))) + (i > length(Q5)) * qf.th5;
        (i <= length(Q6)) * Q6(min(i, length(Q6))) + (i > length(Q6)) * qf.th6;
        (i <= length(Q7)) * Q7(min(i, length(Q7))) + (i > length(Q7)) * qf.th7;
    ];
    if i < N
        Forward_kinematics(Link, Q, 1);
    else
        Forward_kinematics(Link, Q, 0);
    end
end