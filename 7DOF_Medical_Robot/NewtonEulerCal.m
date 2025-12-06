function F_list = NewtonEulerCal(T_list,R_list ,mass_list, mass_center_list, inertia_tensor_list, f_external, dq_list, ddq_list, joint_type_list)
% 使用牛顿欧拉法计算机械臂各关节的力矩 (数值计算版)
% 输入参数：
%   ...
%   joint_type_list:        (可选) 各关节类型列表           (Nx1)   0:旋转副, 1:移动副
% 输出参数：
%   F_list:                 各关节的力矩列表                (Nx1)

number_of_links = size(mass_list,1);  % 连杆数量

% 处理可选参数 joint_type_list
if nargin < 9
    joint_type_list = zeros(number_of_links, 1);
    if number_of_links >= 4
        joint_type_list(4) = 1; % 默认第4关节为移动副
    end
end

% 初始化变量
w = zeros(3,1,number_of_links+1);      % 角速度
dw = zeros(3,1,number_of_links+1);     % 角加速度
dv = zeros(3,1,number_of_links+1);     % 线加速度
dvc = zeros(3,1,number_of_links+1);    % 质心加速度
F = zeros(3,1,number_of_links+1);      % 惯性力
N = zeros(3,1,number_of_links+1);      % 惯性力矩

% =========================================================================
% 第一步：前推 (Forward Recursion)
% =========================================================================

% 初始化基座的线加速度以模拟重力
% 假设重力方向沿 Z 轴负方向，则基座需产生沿 Z 轴正方向的加速度 g
dv(:,:,1) = [0; 0; 9.8]; 

Z_axis = [0; 0; 1]; % 关节轴向量 (假设DH参数建立的坐标系，关节轴都在Z轴)

for i = 1:number_of_links  % 从基座向末端递推
    
    % 提取当前连杆的旋转矩阵R_i和位置向量P_i
    % R_list(:,:,i) 是从第i个坐标系到第i-1个坐标系的旋转矩阵 ({}^{i-1}R_i)
    R_curr = R_list(:,:,i);
    P_curr = T_list(1:3,4,i); % {}^{i-1}P_i
    
    % 获取当前关节的速度和加速度
    dqi = dq_list(i);
    ddqi = ddq_list(i);
    
    % 提取上一连杆的运动学参数 (在上一连杆坐标系中)
    w_prev = w(:,:,i);
    dw_prev = dw(:,:,i);
    dv_prev = dv(:,:,i);
    
    % 注意：根据 joint_type_list 判断关节类型
    if joint_type_list(i) == 1 
        % --- 移动副 (Prismatic Joint) ---
        % 1. 角速度 (无相对旋转)
        w(:,:,i+1) = R_curr' * w_prev;
        
        % 2. 角加速度 (无相对角加速度)
        dw(:,:,i+1) = R_curr' * dw_prev;
        
        % 3. 线加速度 (包含科氏力和相对加速度)
        % dv_transport: 牵连加速度
        dv_transport = R_curr' * (dv_prev + cross(dw_prev, P_curr) + cross(w_prev, cross(w_prev, P_curr)));
        % 加上科氏加速度 (2 * w x v_rel) 和相对加速度 (a_rel)
        % v_rel = Z * dqi, a_rel = Z * ddqi
        dv(:,:,i+1) = dv_transport + 2 * cross(w(:,:,i+1), Z_axis * dqi) + Z_axis * ddqi;
        
    else 
        % --- 转动副 (Revolute Joint) ---
        % 1. 角速度
        % ω_i = R_i^T * ω_{i-1} + z_i * dq_i
        w(:,:,i+1) = R_curr' * w_prev + Z_axis * dqi;
        
        % 2. 角加速度
        % α_i = R_i^T * α_{i-1} + z_i * ddq_i + ω_i × (z_i * dq_i)
        dw(:,:,i+1) = R_curr' * dw_prev + Z_axis * ddqi + cross(w(:,:,i+1), Z_axis * dqi);
        
        % 3. 线加速度
        % a_i = R_i^T * (a_{i-1} + α_{i-1} × P_i + ω_{i-1} × (ω_{i-1} × P_i))
        dv(:,:,i+1) = R_curr' * (dv_prev + cross(dw_prev, P_curr) + cross(w_prev, cross(w_prev, P_curr))); 
    end

    % 4. 计算质心加速度
    % a_c_i = α_i × r_c_i + ω_i × (ω_i × r_c_i) + a_i
    r_ci = mass_center_list(i,:)'; % 质心位置 (在当前连杆坐标系下)
    dvc(:,:,i+1) = cross(dw(:,:,i+1), r_ci) + cross(w(:,:,i+1), cross(w(:,:,i+1), r_ci)) + dv(:,:,i+1);
    
    % 5. 计算惯性力 (牛顿方程)
    % F_i = m_i * a_c_i
    F(:,:,i+1) = mass_list(i) * dvc(:,:,i+1);

    % 6. 计算惯性力矩 (欧拉方程)
    % N_i = I_i·α_i + ω_i × (I_i·ω_i)
    % inertia_tensor_list(:,:,i) 必须是关于质心的惯性张量
    I_curr = inertia_tensor_list(:,:,i);
    N(:,:,i+1) = I_curr * dw(:,:,i+1) + cross(w(:,:,i+1), I_curr * w(:,:,i+1));
end

% =========================================================================
% 第二步：后推 (Backward Recursion)
% =========================================================================

f = zeros(3,1,number_of_links+1);  % 各连杆上施加的力
n = zeros(3,1,number_of_links+1);  % 各连杆上施加的力矩

% 初始化末端连杆的外力和外力矩
f(:,:,number_of_links+1) = f_external(1:3);
n(:,:,number_of_links+1) = f_external(4:6);

for i = number_of_links:-1:1  % 从末端连杆向基座递推

    if i == number_of_links
        R_next = eye(3);
        P_next = [0; 0; 0];
    else
        R_next = R_list(:,:,i+1);    % {}^{i}R_{i+1}
        P_next = T_list(1:3,4,i+1);  % {}^{i}P_{i+1}
    end
    
    % 1. 递推计算力 (力平衡)
    % f_i = R_{i+1} * f_{i+1} + F_i
    f(:,:,i) = R_next * f(:,:,i+1) + F(:,:,i+1);
    
    % 2. 递推计算力矩 (力矩平衡)
    % n_i = N_i + R_{i+1} * n_{i+1} + r_c_i × F_i + P_{i+1} × (R_{i+1} * f_{i+1})
    r_ci = mass_center_list(i,:)'; 
    n(:,:,i) = N(:,:,i+1) + R_next * n(:,:,i+1) + cross(r_ci, F(:,:,i+1)) + cross(P_next, R_next * f(:,:,i+1));
end

% 提取各关节的力矩（力矩在关节轴z方向的投影）
F_list = zeros(number_of_links,1);  % 各关节力矩列表

for i = 1:number_of_links
    if joint_type_list(i) == 1 % 伸缩关节提取力
        F_list(i) = dot(f(:,:,i), Z_axis);
    else % 旋转关节提取力矩
        F_list(i) = dot(n(:,:,i), Z_axis);
    end
end

end
