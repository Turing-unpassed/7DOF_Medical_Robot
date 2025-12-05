function J = Jocobian_Build(link,q)
%微分变换建立关节空间的雅可比矩阵
%   Detailed explanation goes here

J = zeros(6,7);  %预分配雅可比矩阵空间
T = zeros(4,4,8);  %预分配变换矩阵空间

T01 = @(theta) [cosd(theta)  -sind(theta)   0      0;
               sind(theta)    cosd(theta)   0      0;
                   0             0          1     153.7;
                   0             0          0      1];

T12 = @(theta) [cosd(theta)  -sind(theta)   0      0;
                   0             0         -1      0;
               sind(theta)    cosd(theta)   0      0;
                   0             0          0      1];

T23 = @(theta) [cosd(theta)  -sind(theta)   0     250.35;
               sind(theta)    cosd(theta)   0      0;
                   0             0          1      0;
                   0             0          0      1];

T34 = @(d) [ 0 -1  0  0;
             0  0  1  d;
            -1  0  0  0;
             0  0  0  1];

T45 = @(theta) [cosd(theta)  -sind(theta)   0      0;
                  0               0         1     84.8;
                -sind(theta) -cosd(theta)   0      0;
                   0             0          0      1];

T56 = @(theta) [cosd(theta)  -sind(theta)   0      0;
                  0               0         1      0;
                sind(theta)  -cosd(theta)   0      0;
                   0             0          0      1];

T67 = @(theta) [cosd(theta)  -sind(theta)   0      0;
                  0               0         1      0;
                -sind(theta) -cosd(theta)   0      0;
                   0             0          0      1];            
 
T77 = eye(4);
T1 = T01(q(1));                     % T01(q1)
T2 = T12(q(2));                     % T12(q2)
T3 = T23(q(3));                     % T23(q3)
T4 = T34(q(4));                   % T34(q4)
T5 = T45(q(5));                     % T45(q5)
T6 = T56(q(6));                     % T56(q6)
T7 = T67(q(7));                     % T67(q7)
T8 = T77;                          % T77

T07 = T1*T2*T3*T4*T5*T6*T7;    % 末端相对基坐标系的变换矩阵

T = cat(3,T1,T2,T3,T4,T5,T6,T7,T8);   %将变换矩阵按顺序存入T矩阵

for i = 1:7
    Tin = eye(4);                       %初始化为单位矩阵
    for j = i+1:8
        Tin = Tin*T(:,:,j);           %计算各个关节相对基坐标系的变换矩阵
    end
    n = Tin(1:3,1);
    o = Tin(1:3,2);
    a = Tin(1:3,3);
    p = Tin(1:3,4);
    x = cross(p,n);
    y = cross(p,o);
    z = cross(p,a);
    if(link(i+1).type == 'R')        %转动关节
        J(1,i) = x(3);                              %线速度雅可比矩阵
        J(2,i) = y(3);
        J(3,i) = z(3);
        J(4,i) = n(3);                              %角速度雅可比矩阵
        J(5,i) = o(3);
        J(6,i) = a(3);
    else                             %移动关节
        J(1,i) = n(3);                              %线速度雅可比矩阵
        J(2,i) = o(3);
        J(3,i) = a(3);
        J(4,i) = 0;                                 %角速度雅可比矩阵
        J(5,i) = 0;
        J(6,i) = 0;
    end

end
R = zeros(6,6);
r = T07(1:3,1:3);
R(1:3,1:3) = r;
R(4:6,4:6) = r;
J = R*J;   %将雅可比矩阵转换到基坐标系下

end