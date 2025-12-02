function F_list = NewtonEulerCal(T_list,R_list ,mass_list, mass_center_list, inertia_tensor_list, f_external)
% 使用牛顿欧拉法计算机械臂各关节的力矩
% 输入参数：
%   T_list:                 机械臂各连杆的齐次变换矩阵列表  (4x4xN)
%   R_list:                 机械臂各连杆的旋转矩阵列表      (3x3xN)
%   mass_list:              机械臂各连杆的质量列表          (Nx1)
%   mass_center_list:       机械臂各连杆质心位置列表        (Nx3)
%   inertia_tensor_list:    机械臂各连杆惯性张量列表        (3x3xN)
%   f_external:             末端连杆外力和外力矩            (6x3)
% 输出参数：
%   F_list:                 各关节的力矩列表                (Nx1)

number_of_links = size(mass_list,1);  % 连杆数量
% 初始化变量
w = sym(zeros(3,1,number_of_links+1));      % 角速度
dw = sym(zeros(3,1,number_of_links+1));     % 角加速度
dv = sym(zeros(3,1,number_of_links+1));     % 线加速度
dvc = sym(zeros(3,1,number_of_links+1));    % 质心加速度
F = sym(zeros(3,1,number_of_links+1));      % 惯性力
N = sym(zeros(3,1,number_of_links+1));      % 惯性力矩
P = sym(zeros(3,1,number_of_links+1));      % 连杆位置向量
% =========================================================================
% 第一步：前推
% =========================================================================

% 引入重力加速度 g
syms g
% 初始化基座的线加速度以模拟重力，简化计算，不需要之后每个关节考虑重力
% 假设重力方向沿 Z 轴负方向，则基座需产生沿 Z 轴正方向的加速度 g
dv(:,:,1) = [0; 0; g]; 

Z_axis = [0; 0; 1]; % Z轴向量

for i = 1:number_of_links  % 从基座向末端递推
    
    % 提取当前连杆的旋转矩阵R_i和位置向量P_i
    % R_list(:,:,i) 是从第i个坐标系到第i-1个坐标系的旋转矩阵
    R_curr = R_list(:,:,i);
    P_curr = T_list(1:3,4,i);
    
    % 获取当前关节的符号变量
    dqi = sym(['dq', num2str(i)]);
    ddqi = sym(['ddq', num2str(i)]);
    
    % 提取上一连杆的运动学参数
    w_prev = w(:,:,i);
    dw_prev = dw(:,:,i);
    dv_prev = dv(:,:,i);
    
    if i == 4 % === 第4关节为伸缩关节 (Prismatic) ===
        % 1. 计算角速度 (无相对旋转)
        w(:,:,i+1) = R_curr' * w_prev;
        
        % 2. 计算角加速度 (无相对角加速度)
        dw(:,:,i+1) = R_curr' * dw_prev;
        
        % 3. 计算线加速度 (包含科氏力和相对加速度)
        % 先计算牵连加速度
        dv_transport = R_curr' * (dv_prev + cross(dw_prev, P_curr) + cross(w_prev, cross(w_prev, P_curr)));
        % 加上科氏加速度和相对加速度
        dv(:,:,i+1) = dv_transport + 2 * cross(w(:,:,i+1), Z_axis * dqi) + Z_axis * ddqi;
        
    else % === 其他关节为旋转关节 (Revolute) ===
        % 1. 计算角速度
        % ω_i = R_i^T * ω_{i-1} + z_i * dq_i
        w(:,:,i+1) = R_curr' * w_prev + Z_axis * dqi;
        
        % 2. 计算角加速度
        % α_i = R_i^T * α_{i-1} + z_i * ddq_i + ω_i × (z_i * dq_i)
        dw(:,:,i+1) = R_curr' * dw_prev + Z_axis * ddqi + cross(w(:,:,i+1), Z_axis * dqi);
        
        % 3. 计算线加速度
        % a_i = R_i^T * (a_{i-1} + α_{i-1} × P_i + ω_{i-1} × (ω_{i-1} × P_i))
        dv(:,:,i+1) = R_curr' * (dv_prev + cross(dw_prev, P_curr) + cross(w_prev, cross(w_prev, P_curr))); 
    end

    % 4. 计算质心加速度
    % a_c_i = α_i × r_c_i + ω_i × (ω_i × r_c_i) + a_i
    r_ci = mass_center_list(i,:)'; % 质心位置
    dvc(:,:,i+1) = cross(dw(:,:,i+1), r_ci) + cross(w(:,:,i+1), cross(w(:,:,i+1), r_ci)) + dv(:,:,i+1);
    
    % 5. 计算惯性力
    % F_i = m_i * a_c_i
    F(:,:,i+1) = mass_list(i) * dvc(:,:,i+1);

    % 6. 计算惯性力矩（欧拉方程）
    % N_i = I_i·α_i + ω_i × (I_i·ω_i)
    % inertia_tensor_list(:,:,i) 对应第i个连杆
    I_curr = inertia_tensor_list(:,:,i);
    N(:,:,i+1) = I_curr * dw(:,:,i+1) + cross(w(:,:,i+1), I_curr * w(:,:,i+1));
end

% =========================================================================
% 第二步：后推
% =========================================================================


f = sym(zeros(3,1,number_of_links+1));  % 各连杆上施加的力（从关节i+1指向关节i）
n = sym(zeros(3,1,number_of_links+1));  % 各连杆上施加的力矩
% 初始化末端连杆的外力和外力矩
f(:,:,number_of_links+1) = f_external(1:3,:)';  % 末端外力
n(:,:,number_of_links+1) = f_external(4:6,:)';  % 末端外力矩
for i = number_of_links:-1:1  % 从末端连杆向基座递推

    R_next = R_list(:,:,i);  % 第i+1个连杆到第i个连杆的旋转矩阵
    P_next = T_list(1:3,4,i);  % 第i+1个连杆相对于第i个连杆的位置向量
    
    % 1. 递推计算力
    % f_i = R_{i+1} * f_{i+1} + F_i
    f(:,:,i) = R_next * f(:,:,i+1) + F(:,:,i+1);
    
    % 2. 递推计算力矩
    % n_i = N_i + R_{i+1} * n_{i+1} + r_c_i × F_i + P_{i+1} × (R_{i+1} * f_{i+1})
    r_ci = mass_center_list(i,:)'; % 质心位置
    n(:,:,i) = N(:,:,i+1) + R_next * n(:,:,i+1) + cross(r_ci, F(:,:,i+1)) + cross(P_next, R_next * f(:,:,i+1));
end

% 提取各关节的力矩（力矩在关节轴z方向的投影）
F_list = sym(zeros(number_of_links,1));  % 各关节力矩列表
for i = 1:number_of_links
    if i == 4 % 伸缩关节提取力
        F_list(i) = dot(f(:,:,i), Z_axis);
    else % 旋转关节提取力矩
        F_list(i) = dot(n(:,:,i), Z_axis);
    end
end

end
