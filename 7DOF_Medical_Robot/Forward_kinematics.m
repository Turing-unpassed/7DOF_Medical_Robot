function T = Forward_kinematics(Link,Q,fcla)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

radius    = 25;  %25
len       = 60;  %60
joint_col = 0;

% T45 = Link(6).T;
% l = symvar(T45);
% T45 = double(subs(T45,l,0));
for i=2:8        %给每个关节变量替换为实际值
    if(i==5)
        q = symvar(Link(i).T);
        Link(i).T = subs(Link(i).T,q,150+Q(i-1));
    else
        q = symvar(Link(i).T);
        Link(i).T = subs(Link(i).T,q,Q(i-1));
    end
end

for i = 1:9
    Link(i).T = double(Link(i).T);       %将符号矩阵转换为实值矩阵
end

for i=2:9
    Link(i).T=Link(i-1).T*Link(i).T;   %得到每个关节在base坐标系下的齐次变换矩阵
    DrawLink(Link(i-1).T(1:3,4),Link(i).T(1:3,4),'b',2);
end

for i=1:8
    DrawJoint(Link(i).T, radius,len, joint_col);
%     DrawFrame(Link(i).T);
end

DrawFrame(Link(1).T);
DrawFrame(Link(8).T);

axis([-300,600,-300,600,-300,600]);
xlabel('x');
ylabel('y'); 
zlabel('z');
grid on;
drawnow;

% T1=Link(5).T*T45;
% T2=Link(8).T;
T = Link(8).T;
%判断是否清空坐标系
if(fcla)
    cla;
end
hold on

end