close all;
clear;
clc;

Link = MDH_Table_Build();
Link = Transimation_Matrix_Build(Link);
%% 1. 基础参数设置
dof = 7;                 % 6自由度
dt_control = 0.01;      % 控制周期(100Hz)
total_time = 2.0;        % 总仿真时间
dt_sim = 0.01;          % 仿真时间步长(均匀采样)

%% 2. 初始状态
th1=45.0000;
th2=41.7585;
th3=117.2310;
d4=0;
th5=90.0000;
th6=248.9896;
th7=135;
q0 = [th1, th2, th3, d4, th5, th6, th7]';      % 初始位置
q_prev = q0;
dq0 = zeros(7, 1);     % 初始速度
x0 = [q0;dq0];          % 初始状态向量

%% 3. 轨迹规划（使用你的Tplanning函数）
p0 = [150,150,150, 0, 0, 0];
p1 = [450, 450, 450, 360, 360, 180];
v_max = [1000, 1000, 1000, 1000, 1000, 1000];
A = [1000, 1000, 1000, 1000, 1000, 1000];
[Q_cart,~,~] =Tplanning(p0, p1, v_max, A, dt_sim);
%% 生成期望轨迹（每个关节）
step = size(Q_cart, 1);
Q_d = zeros(step, 7);
for i = 1:step
    r11 = cosd(Q_cart(i,4))*cosd(Q_cart(i,5))*cosd(Q_cart(i,6)) - sind(Q_cart(i,4))*sind(Q_cart(i,6));
    r21 = sind(Q_cart(i,4))*cosd(Q_cart(i,5))*cosd(Q_cart(i,6)) + cosd(Q_cart(i,4))*sind(Q_cart(i,6));
    r31 = -sind(Q_cart(i,5))*cosd(Q_cart(i,6));
    r12 = -cosd(Q_cart(i,4))*cosd(Q_cart(i,5))*sind(Q_cart(i,6)) - sind(Q_cart(i,4))*cosd(Q_cart(i,6));
    r22 = -sind(Q_cart(i,4))*cosd(Q_cart(i,5))*sind(Q_cart(i,6)) + cosd(Q_cart(i,4))*cosd(Q_cart(i,6));
    r32 = sind(Q_cart(i,5))*sind(Q_cart(i,6));
    r13 = cosd(Q_cart(i,4))*sind(Q_cart(i,5));
    r23 = sind(Q_cart(i,4))*sind(Q_cart(i,5));
    r33 = cosd(Q_cart(i,5));

    T= [r11    r12    r13    Q_cart(i,1);
        r21    r22    r23    Q_cart(i,2);
        r31    r32    r33    Q_cart(i,3);
        0      0      0        1  ];
    Q = Geometric_Inverse_Kinematics(Link,T);
    [q_best,~,~] = Select_Optimal_Solution(Q,q_prev);
    q_prev = q_best;
    Q_d(i, :) = q_best;
end
Q_d(:, [1 2 3 5 6 7]) = unwrap(deg2rad(Q_d(:, [1 2 3 5 6 7])));
Q_d(:,4) = Q_d(:,4) /1000;
V_d = zeros(size(Q_d));
ACC_d = zeros(size(Q_d));

% 一阶差分计算速度
V_d(1:end-1, :) = diff(Q_d) / dt_sim;
V_d(end, :) = V_d(end-1, :); 

% 一阶差分计算加速度
ACC_d(1:end-1, :) = diff(V_d) / dt_sim;
ACC_d(end, :) = ACC_d(end-1, :);

Q_d = Q_d';
V_d = V_d';
ACC_d = ACC_d';
%% 4. 控制器增益
Kp = diag([80, 80, 60, 40, 30, 20,20]);  % 位置增益
Kd = diag([15, 15, 12, 8, 6, 4,4]);     % 速度增益


%% 6. 创建ODE函数句柄
odefun = @(t, x) robot_dynamics_simple(t, x, dt_control, ...
                                        Kp, Kd, Q_d, V_d, ACC_d);

%% 7. 运行仿真（均匀采样时间点）
t_span = 0:dt_sim:total_time;
options = odeset('RelTol', 1e-5, 'AbsTol', 1e-7);

fprintf('开始仿真...\n');
[t_sim, x_sim] = ode45(odefun, t_span, x0, options);

%% 8. 提取结果
q_sim = x_sim(:, 1:dof)';
n = size(q_sim, 2);
for i = 1:n
    if i~=n
         Forward_kinematics(Link,q_sim(:,i)',1);
    else
         Forward_kinematics(Link,q_sim(:,i)',0);
    end
end
dq_sim = x_sim(:, dof+1:end)';

fprintf('仿真完成！\n');


function dxdt = robot_dynamics_simple(t, x,dt_control, ...
                                        Kp, Kd, Q_d, V_d, ACC_d)

[mass_list, mass_center_list, inertia_tensor_list] = KineticPara_Load();
f_external = zeros(6, 1);
T_list = zeros(4, 4, 7);
R_list = zeros(3, 3, 7);
T01 = @(theta) [cosd(theta)  -sind(theta)   0      0;
                sind(theta)   cosd(theta)   0      0;
                   0             0          1     0.1537;
                   0             0          0      1];

T12 = @(theta) [cosd(theta)  -sind(theta)   0      0;
                   0             0         -1      0;
               sind(theta)    cosd(theta)   0      0;
                   0             0          0      1];

T23 = @(theta) [cosd(theta)  -sind(theta)   0     0.25035;
               sind(theta)    cosd(theta)   0      0;
                   0             0          1      0;
                   0             0          0      1];

T34 = @(d) [ 0 -1  0  0;
             0  0  1  d*0.001;
            -1  0  0  0;
             0  0  0  1];

T45 = @(theta) [cosd(theta)  -sind(theta)   0      0;
                  0               0         1     0.0848;
                -sind(theta) -cosd(theta)   0      0;
                   0             0          0      1];

T56 = @(theta) [cosd(theta)  -sind(theta)   0      0;
                  0               0        -1      0;
                sind(theta)   cosd(theta)   0      0;
                   0             0          0      1];

T67 = @(theta) [cosd(theta)  -sind(theta)   0      0;
                  0               0         1      0;
                -sind(theta) -cosd(theta)   0      0;
                   0             0          0      1];        

    persistent last_tau last_control_time last_M
    if isempty(last_control_time)
        last_control_time = -inf;
        last_tau = zeros(7, 1);
        last_M = zeros(7, 7);
    end
    
    %% 1. 分解状态
    q = x(1:7);
    dq = x(8:end);

    %% 2. 获取期望轨迹（通过最近邻插值）
    traj_idx = min(floor(t/dt_control) + 1, size(Q_d, 2));
    
    qd = Q_d(:, traj_idx);
    dqd = V_d(:, traj_idx);
    ddqd = ACC_d(:, traj_idx);
    
    % 获取变换矩阵
    T_list(:,:,1) = T01(q(1));
    T_list(:,:,2) = T12(q(2));
    T_list(:,:,3) = T23(q(3));
    T_list(:,:,4) = T34(q(4)+150);
    T_list(:,:,5) = T45(q(5));
    T_list(:,:,6) = T56(q(6));
    T_list(:,:,7) = T67(q(7));

    R_list(:,:,1) = T_list(1:3, 1:3,1);
    R_list(:,:,2) = T_list(1:3, 1:3,2);
    R_list(:,:,3) = T_list(1:3, 1:3,3);
    R_list(:,:,4) = T_list(1:3, 1:3,4);
    R_list(:,:,5) = T_list(1:3, 1:3,5);
    R_list(:,:,6) = T_list(1:3, 1:3,6);
    R_list(:,:,7) = T_list(1:3, 1:3,7);
    %% 3. 控制计算（计算力矩控制）
    if t - last_control_time >= dt_control
        % 计算误差
        e = qd - q;
        de = dqd - dq;
        
        % 计算期望加速度（控制律输出）
        u = ddqd + Kp*e + Kd*de;
        
        % 关键简化：直接使用NewtonEulerCal计算控制力矩
        % 输入：当前dq，期望加速度u作为ddq_curr
        last_tau = NewtonEulerCal(T_list, R_list, mass_list, ...
                                 mass_center_list, inertia_tensor_list, ...
                                 f_external, dq, u);
        last_M = extract_mass_matrix_fast(q, dq,T_list, R_list, mass_list, ...
                                 mass_center_list, inertia_tensor_list, ...
                                 f_external);

        last_control_time = t;
    end
    
    %% 4. 计算实际加速度
    tau = last_tau;
    M = last_M;
    % 获取当前变换矩阵
    T_list_curr = T_list;
    R_list_curr = R_list;
    
    % 关键：计算实际加速度
    % 方法1：使用逆动力学公式（需要计算质量矩阵）
    % 方法2：直接求解 ddq = ?
    
    % 这里采用迭代法求解实际加速度（更稳定）
    % 初始化加速度估计
    % ddq_guess = zeros(7, 1);
    
    % % 简单迭代（牛顿法一步）
    % % 计算当前力矩与实际所需力矩的差异
    % tau_current = NewtonEulerCal(T_list_curr, R_list_curr, mass_list, ...
    %                             mass_center_list, inertia_tensor_list, ...
    %                             f_external, dq, ddq_guess);
    
    % 计算雅可比（近似）
    
    % M_approx = diag([0.5, 0.4, 0.3, 0.2, 0.15, 0.1 0.1]);
    % epsilon = 1e-6;
    % M_approx = zeros(7);
    % for i = 1:7
    %     ddq_test = ddq_guess;
    %     ddq_test(i) = ddq_test(i) + epsilon;
    %     tau_test = NewtonEulerCal(T_list_curr, R_list_curr, mass_list, ...
    %                              mass_center_list, inertia_tensor_list, ...
    %                              f_external, dq, ddq_test);
    %     M_approx(:, i) = (tau_test - tau_current) / epsilon;
    % end
    
    % % 计算加速度修正
    % tau_error = tau - tau_current;
    % ddq_correction = M_approx \ tau_error;
    % ddq = ddq_guess + ddq_correction;
    tau_cg = NewtonEulerCal(T_list_curr, R_list_curr, mass_list, ...
                           mass_center_list, inertia_tensor_list, ...
                           f_external, dq, zeros(7, 1));
    
    % 计算加速度：τ = M*ddq + τ_cg → ddq = M^(-1)*(τ - τ_cg)
    ddq = M \ (tau - tau_cg);
                 
    %% 5. 返回状态导数
    dxdt = [dq; ddq];
end

function M = extract_mass_matrix_fast(q, dq,T_list, R_list, mass_list, ...
                                 mass_center_list, inertia_tensor_list, ...
                                 f_external )
    % 高效提取质量矩阵（数值微分）
    
    n = length(q);
    M = zeros(n);
    
    % 计算零加速度的力矩（科氏力+重力）
    tau0 = NewtonEulerCal(T_list, R_list, ...
                         mass_list, ...
                         mass_center_list, ...
                         inertia_tensor_list, ...
                         f_external, ...
                         dq, zeros(n,1));
    
    % 数值微分步长
    epsilon = 1e-6;
    
    for j = 1:n
        % 小扰动
        ddq_eps = zeros(n,1);
        ddq_eps(j) = epsilon;
        
        tau_eps = NewtonEulerCal(T_list, R_list, ...
                                mass_list, ...
                                mass_center_list, ...
                                inertia_tensor_list, ...
                                f_external, ...
                                dq, ddq_eps);
        
        % 数值微分：M(:,j) ≈ (τ(ε) - τ(0)) / ε
        M(:, j) = (tau_eps - tau0) / epsilon;
    end
end