close all;
clear;
clc;

Link = MDH_Table_Build();
Link = Transimation_Matrix_Build(Link);

nPts = 200;

t = linspace(0,2*pi,nPts);

scale = 10;                % tune if required for your robot workspace
cx = 300; cy = 100; cz = 100; % center position in workspace (mm)

Y = scale * 16 * (sin(t)).^3;
Z = scale * (13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t));
X = cx + zeros(size(t));   % constant height; change if you want 3D variation

q_prev = zeros(1,7);

p=[]';

for k = 1:nPts

    P = [X(k); Y(k)+cy; Z(k)+cz];
    R = eye(3);
    T07 = [R, P; 0 0 0 1];
    p = [p,P];
    Q = Geometric_Inverse_Kinematics(Link, T07);
    [q_best,~,~] = Select_Optimal_Solution(Q,q_prev);
    q_prev = q_best;

    if k < nPts
        Forward_kinematics(Link,q_best,1);
        scatter3(p(1,:),p(2,:),p(3,:),10,'r','filled','o');
    else
        Forward_kinematics(Link,q_best,0);
    end
end
