close all;
clear;
clc;

Link = MDH_Table_Build();
Link = Transimation_Matrix_Build(Link);

th1=225.0000; 
th2=138.2415;
th3=3.8072;
d4=0;
th5=270.0000;
th6=127.9514;

th7=135.0000;

q_prev=[th1,th2,th3,d4,th5,th6,th7];



% 全局参数
z_height = 150;
v_max_val = 1000; % mm/s
a_max_val = 800; % mm/s^2
dt = 0.1;
q_total = []; % 总轨迹容器

% 正方形边点定义
p1 = [150, 150, z_height]; % 左下
p2 = [150, 450, z_height]; % 左上
p3 = [450, 450, z_height]; % 右上
p4 = [450, 150, z_height]; % 右下

% 储存位置信息
sq_pts = [p1; p4; p3; p2; p1];

% 各段直线的切向角度
sq_angles = [0, 90, 180, 270, 360]; % 最后一个360是为了闭合时平滑回到0

% 正方形运动
for i = 1:4
    % 提取位置信息
    curr_p = sq_pts(i, :);
    next_p = sq_pts(i+1, :);
    
    % 目标姿态
    target_alpha = sq_angles(i);
    curr_orient = [target_alpha, 180, 0];
    
    % A. 原地旋转对准方向 
    if i == 1
        last_orient = [0, 180, 0]; 
    else
        % 上一段结束时的姿态
        last_orient = [sq_angles(i-1), 180, 0];
        if i==5 % 回到起点
            last_orient(1) = 360;
        end
    end
    
    if i > 1
        % 在 curr_p 处，从上一段角度转到当前段角度
        prev_ang = sq_angles(i-1);
        curr_ang = sq_angles(i);
        
        % 构造旋转起止点
        q_rot_start = [curr_p, prev_ang, 180, 0];
        q_rot_end   = [curr_p, curr_ang, 180, 0];
        
        % 旋转速度/加速度 (使用较小值以平稳旋转)
        v_rot_vec = repmat(100, 1, 6);
        a_rot_vec = repmat(100, 1, 6);
        
        [q_rot, ~, ~] = Tplanning(q_rot_start, q_rot_end, v_rot_vec, a_rot_vec, dt);
        q_total = [q_total; q_rot];
    end
    
    % B. 直线运动
    % 构造 6D 向量
    q_start = [curr_p, curr_orient];
    q_end = [next_p, curr_orient];
    
    % 速度和加速度向量 (6维)
    % 确保所有分量为正，避免 Tplanning 报错
    v_vec = repmat(v_max_val, 1, 6);
    a_vec = repmat(a_max_val, 1, 6);
    
    [q_line, ~, ~] = Tplanning(q_start, q_end, v_vec, a_vec, dt);
    q_total = [q_total; q_line];
end

% 2. 叶形弧线

% 先在 p1 处调整姿态，准备画弧
% 弧线1在 p1 处的切线方向：圆心在正上方，切线水平向右 (0度)
% 上一段结束时角度是 270度。需要转到 0 度 (360度)
q_rot_start = [p1, 270, 180, 0];
q_rot_end   = [p1, 360, 180, 0];
v_rot_vec = repmat(100, 1, 6);
a_rot_vec = repmat(100, 1, 6);
[q_rot, ~, ~] = Tplanning(q_rot_start, q_rot_end, v_rot_vec, a_rot_vec, dt);
q_total = [q_total; q_rot];

% 画弧线 1
center_arc1 = p2; % (150, 450)
radius_arc = 300;
q_arc1 = PlanArcSegment(center_arc1, radius_arc, -pi/2, 0, z_height, v_max_val, a_max_val, dt);
q_total = [q_total; q_arc1];

% 此时到达 p3 (450, 450)。切线方向 90度 (垂直向上)
% 弧线2: 以 p4(右下 450, 150) 为圆心，半径 300，从 p3 画回 p1
% 起点 p3(450, 450) -> 相对 p4 是 (0, 300) -> 90度
% 终点 p1(150, 150) -> 相对 p4 是 (-300, 0) -> 180度
% 逆时针画弧 (90 -> 180)

% 弧线1结束时，切线是垂直向上 (90度)。
% 弧线2在 p3 开始时，圆心在正下方，切线水平向左 (180度)。
% 需要在 p3 处从 90度 转到 180度。
q_rot_start = [p3, 90, 180, 0];
q_rot_end   = [p3, 180, 180, 0];
v_rot_vec = repmat(100, 1, 6);
a_rot_vec = repmat(100, 1, 6);
[q_rot, ~, ~] = Tplanning(q_rot_start, q_rot_end, v_rot_vec, a_rot_vec, dt);
q_total = [q_total; q_rot];

% 画弧线 2
center_arc2 = p4; % (450, 150)
q_arc2 = PlanArcSegment(center_arc2, radius_arc, pi/2, pi, z_height, v_max_val, a_max_val, dt);
q_total = [q_total; q_arc2];

q = q_total;

% --- 修改结束 ---

step = length(q);

Forward_kinematics(Link,q_prev,0);
pause(5);

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

% 辅助函数：生成圆弧段轨迹
% 输入: center, radius, theta_start, theta_end, z_h
% 输出: q_seg (N x 6)
function q_seg = PlanArcSegment(center, radius, theta_start, theta_end, z_h, v_max, a_max, dt)
    arc_len = radius * abs(theta_end - theta_start);
    
    q0_v = [0, 0, 0, 0, 0, 0];
    q1_v = [arc_len, 0, 0, 0, 0, 0];
    v_v = [v_max, 1, 1, 1, 1, 1];
    a_v = [a_max, 1, 1, 1, 1, 1];
    [S_traj, ~, ~] = Tplanning(q0_v, q1_v, v_v, a_v, dt);
    s = S_traj(:, 1);
    
    N = length(s);
    q_seg = zeros(N, 6);
    
    % 映射
    % s = R * |theta - theta_start|
    % theta = theta_start + sign * (s/R)
    direction = sign(theta_end - theta_start);
    theta_t = theta_start + direction * (s / radius);
    
    q_seg(:, 1) = center(1) + radius * cos(theta_t);
    q_seg(:, 2) = center(2) + radius * sin(theta_t);
    q_seg(:, 3) = z_h;
    
    % 姿态跟随切向
    % 切向角 phi = theta + 90 (逆时针) 或 theta - 90 (顺时针)
    % 这里统一用 atan2 计算切向向量
    for k = 1:N
        % 计算当前点切向向量
        % d/dt [R cos, R sin] = [-R sin, R cos] * dtheta/dt
        % 向量方向 [-sin(theta), cos(theta)] (逆时针)
        % 若顺时针则相反
        vx = -sin(theta_t(k)) * direction;
        vy = cos(theta_t(k)) * direction;
        phi = atan2d(vy, vx);
        
        % Alpha = phi
        q_seg(k, 4) = phi; 
        q_seg(k, 5) = 180; % Beta 朝下
        q_seg(k, 6) = 0;
    end
end

