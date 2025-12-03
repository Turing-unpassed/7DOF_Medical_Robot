function Q = Geometric_Inverse_Kinematics(Link,T07)
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
length = 300;
step = 0.1;

Q = NaN(8,7,length/step+1);  %预分配空间，最多11组解 
T04 = zeros(4,4,4);
T05 = zeros(4,4,4);
T57 = zeros(4,4,4);
alpha = zeros(2,4);
beta = zeros(2,4);
gamma = zeros(2,4);

T00 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

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

for d4 = 0:step:length
    d = 150+d4;
    thetad = atan2d(Link(6).d,d);
    l = sqrt(d^2 + Link(6).d^2);

    % 每次迭代重置并准备索引
    q = zeros(3,4);
    idx = fix(d4/step+1);

    z = p(3)-Link(2).d;

    q(1,1) = atan2d(p(2),p(1));  %th1_1
    q(1,2) = q(1,1)+180; %th1_2

    x1 = p(1)/cosd(q(1,1));
    x2 = p(1)/cosd(q(1,2));

    % 检查 acos 的定义域，若越界则跳过本次 d4
    s1 = (x1^2 + z^2 - Link(3).a^2 - l^2) / (2*Link(3).a*l);
    s2 = (x2^2 + z^2 - Link(3).a^2 - l^2) / (2*Link(3).a*l);
    if ~isfinite(s1) || ~isfinite(s2) || abs(s1) > 1 || abs(s2) > 1
        Q(:,:,idx) = NaN(8,7);
        continue
    end

    theta1 = acosd(s1);
    theta2 = acosd(s2);
    q(3,1) = theta1 - thetad - 90;  %th3_1
    q(3,2) = -theta1 - thetad - 90; %th3_2
    q(3,3) = theta2 - thetad - 90;  %th3_3
    q(3,4) = -theta2 - thetad - 90; %th3_4

    k1 = (x1^2 + z^2 + Link(3).a^2 - l^2) / (2*Link(3).a*sqrt(x1^2+z^2));
    k2 = (x2^2 + z^2 + Link(3).a^2 - l^2) / (2*Link(3).a*sqrt(x2^2+z^2));
    if ~isfinite(k1) || abs(k1) > 1 || ~isfinite(k2) || abs(k2) > 1
        Q(:,:,idx) = NaN(8,7);
        continue
    end
    phi1 = acosd(k1);
    phi2 = acosd(k2);
    if theta1
        q(2,1) = atan2d(z,x1) - phi1;    %th2_1
        q(2,2) = atan2d(z,x1) + phi1;    %th2_2
    else
        q(2,1) = atan2d(z,x1) + phi1;    %th2_1
        q(2,2) = atan2d(z,x1) - phi1;    %th2_2
    end

    if theta2
        q(2,3) = atan2d(z,x2) - phi2;    %th2_3
        q(2,4) = atan2d(z,x2) + phi2;    %th2_4
    else
        q(2,3) = atan2d(z,x2) + phi2;    %th2_3
        q(2,4) = atan2d(z,x2) - phi2;    %th2_4
    end
    % 若计算出的任何角为非实数或 NaN，视为不可达
    if any(~isfinite(q(:))) || ~isreal(q)
        Q(:,:,idx) = NaN(8,7);
        continue
    end

    for i = 1:2
        T1 = T01(q(1,i));
        for j = 2*i-1:2*i
            T2 = T12(q(2,j));
            T3 = T23(q(3,j));
            T04(:,:,j) = T00*T1*T2*T3*T34(d);  %根据前三轴计算T04
        end
    end

    for i = 1:4
        T05(:,:,i) = T04(:,:,i)*T45(0);   %得到T05
        %T07 = T05*T57
        T57(:,:,i) = T05(:,:,i)\T07;
        [alpha(:,i), beta(:,i), gamma(:,i)] = ZYZ_EulerAngle_Solution(T57(:,:,i));  %zyz欧拉角求解后三轴
    end

    alpha = alpha - 180;   %从欧拉角得到后三轴真实角度
    gamma = gamma - 180;

    alpha = NormalizeAngle(alpha);  %将得到的角度归一化到0-360°
    beta = NormalizeAngle(beta);
    gamma = NormalizeAngle(gamma);
    q = NormalizeAngle(q);

    Q1= [q(1,1),q(2,1),q(3,1),d4,alpha(1,1) ,beta(1,1),gamma(1,1) ];
    Q2= [q(1,1),q(2,1),q(3,1),d4,alpha(2,1) ,beta(2,1),gamma(2,1) ];
    Q3= [q(1,1),q(2,2),q(3,2),d4,alpha(1,2) ,beta(1,2),gamma(1,2) ];
    Q4= [q(1,1),q(2,2),q(3,2),d4,alpha(2,2) ,beta(2,2),gamma(2,2) ];
    Q5= [q(1,2),q(2,3),q(3,3),d4,alpha(1,3) ,beta(1,3),gamma(1,3) ];
    Q6= [q(1,2),q(2,3),q(3,3),d4,alpha(2,3) ,beta(2,3),gamma(2,3) ];
    Q7= [q(1,2),q(2,4),q(3,4),d4,alpha(1,4) ,beta(1,4),gamma(1,4) ];
    Q8= [q(1,2),q(2,4),q(3,4),d4,alpha(2,4) ,beta(2,4),gamma(2,4) ];
    Qi=[Q1;Q2;Q3;Q4;Q5;Q6;Q7;Q8];  %得到8组解，每行为一组解
    Q(:,:,idx)=Qi;  %将不同d4下的解存入Q矩阵中

end

end