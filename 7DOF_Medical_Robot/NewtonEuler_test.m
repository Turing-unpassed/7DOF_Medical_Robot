clear; clc; close all;
%纯AI生成版本，好像能用？
% =========================================================================
% 1. 加载动力学参数
% =========================================================================
[mass_list, mass_center_list, inertia_tensor_list] = KineticPara_Load();

% =========================================================================
% 2. 建立机器人模型 (用于获取 DH 参数)
% =========================================================================
Link = MDH_Table_Build();
% 提取关节类型列表 (0: Revolute, 1: Prismatic)
% Link(2)对应J1, Link(8)对应J7
joint_types = zeros(7, 1);
for k = 1:7
    if strcmp(Link(k+1).type, 'P')
        joint_types(k) = 1;
    end
end

% 注意：这里不需要调用 Transimation_Matrix_Build，因为我们在循环中手动计算数值矩阵
% 且 Transimation_Matrix_Build 生成的是符号矩阵，计算慢且不方便

% =========================================================================
% 3. 轨迹规划 (参考 EndPoint_TrajectoryPlanning.m)
% =========================================================================
% 初始关节角度 (用于逆解的初始猜测)
th1=45.0000; 

th2=41.7585;

th3=117.2310;

d4=0;

th5=90.0000;

th6=248.9896;

th7=135;
q_prev_initial = [th1, th2, th3, d4, th5, th6, th7];

% 笛卡尔空间规划参数
q0 = [150,150,150, 0, 0, 0];
q1 = [450, 450, 450, 360, 360, 180];
v_max = [1000, 1000, 1000, 1000, 1000, 1000];
A = [1000, 1000, 1000, 1000, 1000, 1000];
dt = 0.01;

% 生成笛卡尔轨迹
[Q_cart, ~, ~] = Tplanning(q0, q1, v_max, A, dt);
step = size(Q_cart, 1);

% =========================================================================
% 4. 逆运动学求解关节角度
% =========================================================================
Q_joint = zeros(step, 7);
q_prev = q_prev_initial;

% 为了逆解需要 Link 结构体包含 T 字段 (符号或数值均可，Geometric_Inverse_Kinematics 需要)
% 我们先生成一次符号的，供逆解函数使用
Link_IK = Transimation_Matrix_Build(Link);

% --- 仿真初始化 ---
figure('Name', 'Robot Motion Simulation');
Forward_kinematics(Link_IK, q_prev, 0);
pause(1);

fprintf('正在进行逆运动学求解与仿真...\n');
for i = 1:step
    % 构建末端变换矩阵
    % 参考 EndPoint_TrajectoryPlanning.m 中的构建方式
    r11 = cosd(Q_cart(i,4))*cosd(Q_cart(i,5))*cosd(Q_cart(i,6)) - sind(Q_cart(i,4))*sind(Q_cart(i,6));
    r21 = sind(Q_cart(i,4))*cosd(Q_cart(i,5))*cosd(Q_cart(i,6)) + cosd(Q_cart(i,4))*sind(Q_cart(i,6));
    r31 = -sind(Q_cart(i,5))*cosd(Q_cart(i,6));
    r12 = -cosd(Q_cart(i,4))*cosd(Q_cart(i,5))*sind(Q_cart(i,6)) - sind(Q_cart(i,4))*cosd(Q_cart(i,6));
    r22 = -sind(Q_cart(i,4))*cosd(Q_cart(i,5))*sind(Q_cart(i,6)) + cosd(Q_cart(i,4))*cosd(Q_cart(i,6));
    r32 = sind(Q_cart(i,5))*sind(Q_cart(i,6));
    r13 = cosd(Q_cart(i,4))*sind(Q_cart(i,5));
    r23 = sind(Q_cart(i,4))*sind(Q_cart(i,5));
    r33 = cosd(Q_cart(i,5));

    T_end= [r11    r12    r13    Q_cart(i,1);
            r21    r22    r23    Q_cart(i,2);
            r31    r32    r33    Q_cart(i,3);
            0      0      0        1  ];
    Q_sols = Geometric_Inverse_Kinematics(Link_IK, T_end);
    [q_best, ~, ~] = Select_Optimal_Solution(Q_sols, q_prev);
    
    if any(isnan(q_best))
        warning('第 %d 步无解，使用上一步结果', i);
        q_best = q_prev;
    end
    
    Q_joint(i, :) = q_best;
    q_prev = q_best;

    % --- 机器人仿真绘图 ---
    if i < step
        Forward_kinematics(Link_IK, q_best, 1);
        % 绘制末端轨迹
        plot3(Q_cart(:,1), Q_cart(:,2), Q_cart(:,3), 'r', 'LineWidth', 1);
        hold on;
    else
        Forward_kinematics(Link_IK, q_best, 0);
    end
end

% =========================================================================
% 5. 计算关节速度和加速度 (数值微分)
% =========================================================================
% 转换为 SI 单位进行微分
% 角度 -> 弧度, 长度(J4) -> 米
Q_joint_SI = Q_joint;

% 【关键修复】使用 unwrap 处理角度跳变 (例如 180 -> -180)
% unwrap 只能处理弧度，所以先转弧度再 unwrap
Q_joint_SI(:, [1 2 3 5 6 7]) = unwrap(deg2rad(Q_joint(:, [1 2 3 5 6 7]))); 

% 移动关节单位转换 mm -> m
for k = 1:7
    if joint_types(k) == 1
        Q_joint_SI(:, k) = Q_joint(:, k) / 1000;
    end
end

% 【新增修复】对关节位置进行平滑处理，消除微小的 IK 跳变和数值噪声
% 这可以有效抑制由于逆解不连续导致的速度/加速度脉冲
for k = 1:7
    Q_joint_SI(:, k) = smoothdata(Q_joint_SI(:, k), 'gaussian', 20);
end

dQ_joint_SI = zeros(size(Q_joint_SI));
ddQ_joint_SI = zeros(size(Q_joint_SI));

% 一阶差分计算速度
dQ_joint_SI(1:end-1, :) = diff(Q_joint_SI) / dt;
dQ_joint_SI(end, :) = dQ_joint_SI(end-1, :); 

% 一阶差分计算加速度
ddQ_joint_SI(1:end-1, :) = diff(dQ_joint_SI) / dt;
ddQ_joint_SI(end, :) = ddQ_joint_SI(end-1, :);

% 【可选】对加速度进行简单平滑处理，减少数值微分噪声
for k = 1:7
    ddQ_joint_SI(:, k) = smoothdata(ddQ_joint_SI(:, k), 'movmean', 5);
end

% =========================================================================
% 6. 牛顿-欧拉动力学计算
% =========================================================================
F_list_all = zeros(step, 7);
f_external = zeros(6, 1); % 假设无外力

fprintf('正在进行动力学计算...\n');
for i = 1:step
    q_curr = Q_joint(i, :); % 使用原始单位(度/mm)用于计算变换矩阵
    dq_curr = dQ_joint_SI(i, :)';
    ddq_curr = ddQ_joint_SI(i, :)';
    
    % 计算各连杆相对变换矩阵 T_list 和 R_list
    T_list = zeros(4, 4, 7);
    R_list = zeros(3, 3, 7);
    
    for j = 1:7
        link_idx = j + 1; % Link 数组索引 (Link(1)是Base, Link(2)是J1...)
        
        % 获取上一连杆的 DH 参数 (MDH 定义)
        prev_idx = link_idx - 1;
        alpha_val = Link(prev_idx).alpha;
        a_val = Link(prev_idx).a;
        
        % 获取当前连杆的 DH 参数
        d_val = Link(link_idx).d;
        theta_val = Link(link_idx).theta;
        
        % 更新关节变量
        if joint_types(j) == 1 % 移动副
            d_val = d_val + q_curr(j); 
        else % 旋转副
            theta_val = theta_val + q_curr(j);
        end
        
        ct = cosd(theta_val); st = sind(theta_val);
        ca = cosd(alpha_val); sa = sind(alpha_val);
        
        % MDH 变换矩阵公式
        T_val = [    ct          -st         0          a_val;
                 st*ca       ct*ca      -sa      -d_val*sa;
                 st*sa       ct*sa       ca       d_val*ca;
                    0           0          0          1      ];
                    
        % 【关键】单位转换：位置部分 mm -> m
        T_val(1:3, 4) = T_val(1:3, 4) / 1000;
        
        T_list(:,:,j) = T_val;
        R_list(:,:,j) = T_val(1:3, 1:3);
    end
    
    F_list = NewtonEulerCal(T_list, R_list, mass_list, mass_center_list, inertia_tensor_list, f_external, dq_curr, ddq_curr, joint_types);
    F_list_all(i, :) = F_list';
end

% =========================================================================
% 7. 绘图
% =========================================================================
time = (0:step-1) * dt;

figure('Name', 'Joint Torques/Forces');
sgtitle('Joint Torques/Forces');
for k = 1:7
    subplot(4, 2, k);
    plot(time, F_list_all(:, k), 'LineWidth', 1.5);
    title(['Joint ', num2str(k)]);
    if joint_types(k) == 1
        ylabel('Force (N)');
    else
        ylabel('Torque (N·m)');
    end
    grid on;
end
