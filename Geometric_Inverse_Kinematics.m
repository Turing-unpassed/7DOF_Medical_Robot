function Q = Geometric_Inverse_Kinematics(link,T07)
%几何代数法求解前三轴关节变量，Z-Y-Z欧拉角求解后三轴
%                   q[1,1] --------------q[1,2]
%                     |                     |
%                 |       |             |        |
%               q[2,1]  q[2,2]        q[2,3]  q[2,4]    
%                 |       |             |        |
%                 |       |             |        |
%              q[3,1]   q[3,2]        q[3,3]   q[3,4]    每列为一组解，前三轴共四组解
%                 |       |             |        |
%              |     | 
%           angle1  angle2   ----    ------    angle8   
%         每列为一组解，共8组
%   Detailed explanation goes here
p = T07(1:3,4);
d4=0;
d = 150+d4;

z = p(3)-link(2).d;
l = sqrt(p(1)^2+p(2)^2-link(3).d^2);

q(1,1) = 90 + atan2d(p(2),p(1)) - atan2d(l,link(3).d);  %th1_1
q(1,2) = 90 + atan2d(p(2),p(1)) - atan2d(-l,link(3).d); %th1_2

x1 = (p(1)-link(3).d*cosd(q(1,1)-90))/cosd(q(1,1));
x2 = (p(1)-link(3).d*cosd(q(1,2)-90))/cosd(q(1,2));

q(3,1) = asind(-(x1^2+z^2-link(3).a^2-d^2)/(2*link(3).a*d));  %th3_1
q(3,2) = 180-q(3,1);                                            %th3_2
q(3,3) = asind(-(x2^2+z^2-link(3).a^2-d^2)/(2*link(3).a*d));  %th3_3
q(3,4) = 180-q(3,3);                                            %th3_4

th1=90+q(3,1);
th2=90+q(3,2);
th3=90+q(3,3);
th4=90+q(3,4);

k11 = link(3).a+d*cosd(th1);
k12 = link(3).a+d*cosd(th2);
k13 = link(3).a+d*cosd(th3);
k14 = link(3).a+d*cosd(th4);

k21 = d*sind(th1);
k22 = d*sind(th2);
k23 = d*sind(th3);
k24 = d*sind(th4);

r1 = sqrt(k11^2+k21^2);
r2 = sqrt(k12^2+k22^2);
r3 = sqrt(k13^2+k23^2);
r4 = sqrt(k14^2+k24^2);

q(2,1) = atan2d(z/r1,x1/r1)-atan2d(k21,k11);    %th2_1
q(2,2) = atan2d(z/r2,x1/r2)-atan2d(k22,k12);    %th2_2
q(2,3) = atan2d(z/r3,x2/r3)-atan2d(k23,k13);    %th2_3
q(2,4) = atan2d(z/r4,x2/r4)-atan2d(k24,k14);    %th2_4

Link = Transimation_Matrix_Build(link);
D = symvar(Link(5).T);
Link(5).T = subs(Link(5).T,D,d);
T04 = zeros(4,4,4);
T05 = zeros(4,4,4);
T57 = zeros(4,4,4);
for i = 1:2
    q2 = symvar(Link(2).T);
    T2 = subs(Link(2).T,q2,q(1,i));
    for j = 2*i-1:2*i
        q3 = symvar(Link(3).T);
        T3 = subs(Link(3).T,q3,q(2,j));
        q4 = symvar(Link(4).T);
        T4 = subs(Link(4).T,q4,q(3,j));
        T04(:,:,j) = double(Link(1).T*T2*T3*T4*Link(5).T);  %根据前三轴计算T04
    end
end
th = symvar(Link(6).T);
Link(6).T = double(subs(Link(6).T,th,0));
alpha = zeros(2,4);
beta = zeros(2,4);
gamma = zeros(2,4);
for i = 1:4
    T05(:,:,i) = T04(:,:,i)*Link(6).T;   %得到T05
    %T07 = T05*T57
    T57(:,:,i) = T05(:,:,i)\T07;
    [alpha(:,i), beta(:,i), gamma(:,i)] = ZYZ_EulerAngle_Solution(T57(:,:,i));  %zyz欧拉角求解后三轴
end
alpha = alpha - 180;   %从欧拉角得到后三轴真实角度
gamma = gamma - 180;

alpha = NormalizeAngle(alpha);  %将得到的角度归一化到0-360°
beta = NormalizeAngle(beta);
gamma = NormalizeAngle(gamma);

Q1= [q(1,1),q(2,1),q(3,1),d4,alpha(1,1) ,beta(1,1),gamma(1,1) ];  
Q2= [q(1,1),q(2,1),q(3,1),d4,alpha(2,1) ,beta(2,1),gamma(2,1) ];
Q3= [q(1,1),q(2,2),q(3,2),d4,alpha(1,2) ,beta(1,2),gamma(1,2) ];
Q4= [q(1,1),q(2,2),q(3,2),d4,alpha(2,2) ,beta(2,2),gamma(2,2) ];
Q5= [q(1,2),q(2,3),q(3,3),d4,alpha(1,3) ,beta(1,3),gamma(1,3) ];
Q6= [q(1,2),q(2,3),q(3,3),d4,alpha(2,3) ,beta(2,3),gamma(2,3) ];
Q7= [q(1,2),q(2,4),q(3,4),d4,alpha(1,4) ,beta(1,4),gamma(1,4) ];
Q8= [q(1,2),q(2,4),q(3,4),d4,alpha(2,4) ,beta(2,4),gamma(2,4) ];
Q=[Q1;Q2;Q3;Q4;Q5;Q6;Q7;Q8];  %得到8组解，每行为一组解
end   