function Link = Transimation_Matrix_Build(link)
%根据DH表建立每个关节相对上一个关节的齐次变换矩阵
%   Detailed explanation goes here
%                 ​           cosθi              −sinθi​​                  0​​​             ai−1
%       i-1-----          ​sinθi*​cosαi−1       cosθi​*cosαi−1         −sinαi−1       −di​*sinαi−1​
%            |    =       sinθi​*sinαi−1       cosθi​*sinαi−1          cosαi−1        ​​di​*cosαi−1​​​
%            |i               0                   0                    0               1​
syms alpha_var a d theta

T = [    cosd(theta)                   -sind(theta)                  0                a;
    sind(theta)*cosd(alpha_var)   cosd(theta)*cosd(alpha_var)   -sind(alpha_var)  -d*sind(alpha_var);
    sind(theta)*sind(alpha_var)   cosd(theta)*sind(alpha_var)    cosd(alpha_var)   d*cosd(alpha_var);
              0                           0                        0                1          ];

for i=1:4                     %建立包含关节变量的齐次变换矩阵
    if(i>1)
        parami.alpha_var = link(i-1).alpha;
        parami.a = link(i-1).a;
        parami.d = link(i).d;
        Ti = subs(T,parami);
        link(i).T = Ti;
    else
        param0.alpha_var = 0;
        param0.a = 0;
        param0.d = 0;
        param0.theta = 0;
        T0 = subs(T,param0);
        link(i).T = T0;
    end
end

param5.alpha_var = link(4).alpha;
param5.a = link(4).a;
param5.theta = link(5).theta;
T5 = subs(T,param5);
link(5).T = T5;                   

for k = 6:8
    paramk.alpha_var = link(k-1).alpha;
    paramk.a = link(k-1).a;
    paramk.d = link(k).d;
    Tk = subs(T,paramk);
    link(k).T = Tk;
end

param9.alpha_var = 0;
param9.a = 0;
param9.d = 80;
param9.theta = 0;
T9 = subs(T,param9);
link(9).T = T9;

Link = link;
end