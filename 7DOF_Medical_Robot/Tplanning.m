function [Q, V, ACC] = Tplanning(q0, q1, v_max, A, dt)
% TPLANNING 6自由度同步梯形速度规划函数
% 功能:
%   为空间刚体的六个自由度规划同步的梯形速度轨迹。所有自由度同时开始、同时结束。
%   采用混合策略：
%   1. 前三个自由度(XYZ)不仅在时间上同步，其运动形态(加/匀速时间比)也尽可能保持一致，
%      形态由XYZ中最慢的那个轴决定。这通过同时调整v_max和A实现。
%   2. 后三个自由度(ABC)仅在时间上同步，通过降低其加速度实现。
%
% 输入: 
%  【警告:输入向量必须为行向量A=（1，2，3）这种用逗号分割的形势】
%   q0    : 起始位置向量 (1x6)
%   q1    : 目标位置向量 (1x6)
%   v_max : 各轴最大允许速度向量 (1x6)
%   A     : 各轴最大允许加速度向量 (1x6)
%   dt    : 采样时间间隔 (例如 0.001)
%
% 输出:
%   Q     : 位置矩阵 (Nx6)
%   V     : 速度矩阵 (Nx6)
%   ACC   : 加速度矩阵 (Nx6)

    % --- 数值容差常量 ---
    EPS = 1e-10;

    % --- 输入参数校验 ---
    if any(size(q0) ~= [1, 6]) || any(size(q1) ~= [1, 6]) || ...
       any(size(v_max) ~= [1, 6]) || any(size(A) ~= [1, 6])
        error('错误: q0, q1, v_max, A 都必须是 1x6 的行向量。');
    end
    if any(v_max <= 0) || any(A <= 0) || dt <= 0
        error('错误: v_max, A 和 dt 的所有元素都必须为正数。');
    end

    num_dof = 6;

    dist = q1 - q0;
    dir = sign(dist);
    L = abs(dist);

    % --- 步骤1: 独立规划，计算各轴最短时间 ---
    T_totals = zeros(1, num_dof);
    ta_optimal = zeros(1, num_dof);
    tc_optimal = zeros(1, num_dof);

    for i = 1:num_dof
        if L(i) < EPS
            T_totals(i) = 0; continue;
        end
        if (v_max(i)^2 / A(i)) <= L(i) % 梯形
            ta_optimal(i) = v_max(i) / A(i);
            La = 0.5 * A(i) * ta_optimal(i)^2;
            L_const = L(i) - 2 * La;
            tc_optimal(i) = L_const / v_max(i);
            T_totals(i) = 2 * ta_optimal(i) + tc_optimal(i);
        else % 三角形
            ta_optimal(i) = sqrt(L(i) / A(i));
            tc_optimal(i) = 0;
            T_totals(i) = 2 * ta_optimal(i);
        end
    end

    % --- 步骤2: 确定全局同步时间 和 XYZ的参考形态 ---
    T_sync = max(T_totals);
    if T_sync < EPS
        % 所有轴都不动，返回单点轨迹（保持与正常返回格式一致）
        Q = q0;           % 1x6
        V = zeros(1, num_dof);   % 1x6
        ACC = zeros(1, num_dof); % 1x6
        return;
    end
    
    % 找到前三个轴(XYZ)中最慢的那个，作为形态参考
    % 注意：如果前三个轴都不动，需要特殊处理
    xyz_moving = L(1:3) > EPS;
    if any(xyz_moving)
        % 只在运动的轴中找最慢的
        T_xyz = T_totals(1:3);
        T_xyz(~xyz_moving) = -1; % 排除不动的
        [~, idx_local] = max(T_xyz);
        
        ta_ref = ta_optimal(idx_local);
        tc_ref = tc_optimal(idx_local);
        
        is_ref_triangle = (tc_ref < EPS);
        k_ratio = 0; 
        if ~is_ref_triangle && ta_ref > EPS
            k_ratio = tc_ref / ta_ref;
        end
    else
        % XYZ都不动，参数无所谓
        is_ref_triangle = true;
        k_ratio = 0;
    end

    % --- 步骤3 & 4: 分策略重新计算所有轴的运动参数 ---
    A_new = zeros(1, num_dof);
    v_lim = zeros(1, num_dof);
    ta_new = zeros(1, num_dof);
    tc_new = zeros(1, num_dof);

    for i = 1:num_dof
        if L(i) < EPS; continue; end

        if i <= 3 % --- XYZ轴: 保持形态，同时调整v和A ---
            if is_ref_triangle
                % 如果参考形态是三角形，所有XYZ轴都规划为三角形
                ta_new(i) = T_sync / 2;
                tc_new(i) = 0;
            else
                % 参考形态是梯形，按比例分配时间
                % T_sync = 2*ta + tc = 2*ta + k_ratio*ta = ta*(2+k_ratio)
                ta_new(i) = T_sync / (2 + k_ratio);
                tc_new(i) = T_sync - 2 * ta_new(i);
            end
            
            % 根据新的时间分配，反解v_lim和A_new
            % L = v_lim * (ta + tc) = v_lim * (T_sync - ta)
            % v_lim = L / (ta + tc)
            denom_v = ta_new(i) + tc_new(i);
            if denom_v < EPS
                v_lim(i) = 0;
                A_new(i) = 0;
            else
                v_lim(i) = L(i) / denom_v;
                % A = v / ta
                if ta_new(i) < EPS
                    A_new(i) = 0;
                else
                    A_new(i) = v_lim(i) / ta_new(i);
                end
            end

        else % --- ABC轴: 只调整A来满足时间 (常规同步) ---
            % 检查在新T_sync下，规划是梯形还是三角
            if L(i) < v_max(i) * T_sync / 2
                % 新规划为三角形
                v_lim(i) = 2 * L(i) / T_sync;
                A_new(i) = 4 * L(i) / T_sync^2;
                ta_new(i) = T_sync / 2;
                tc_new(i) = 0;
            else
                % 新规划为梯形
                v_lim(i) = v_max(i);
                denom_a = v_max(i) * T_sync - L(i);
                if denom_a < EPS
                    % 退化为三角形情况
                    v_lim(i) = 2 * L(i) / T_sync;
                    A_new(i) = 4 * L(i) / T_sync^2;
                    ta_new(i) = T_sync / 2;
                    tc_new(i) = 0;
                else
                    A_new(i) = v_max(i)^2 / denom_a;
                    ta_new(i) = v_max(i) / A_new(i);
                    tc_new(i) = T_sync - 2 * ta_new(i);
                end
            end
        end
    end

    % --- 步骤5: 同步生成轨迹 (向量化) ---
    steps = ceil(T_sync / dt);
    t = (0:steps)' * dt;
    N = length(t);
    
    Q = zeros(N, num_dof);
    V = zeros(N, num_dof);
    ACC = zeros(N, num_dof);
    
    % 准备矩阵进行计算
    t1 = ta_new;            % 1x6
    t2 = ta_new + tc_new;   % 1x6
    
    % 扩展维度
    T_mat = repmat(t, 1, num_dof);      % Nx6
    T1_mat = repmat(t1, N, 1);          % Nx6
    T2_mat = repmat(t2, N, 1);          % Nx6
    A_mat = repmat(A_new, N, 1);        % Nx6
    V_lim_mat = repmat(v_lim, N, 1);    % Nx6
    Dir_mat = repmat(dir, N, 1);        % Nx6
    Q0_mat = repmat(q0, N, 1);          % Nx6
    Moving_mask = repmat(L > EPS, N, 1);
    
    % 掩码
    mask_acc = (T_mat < T1_mat) & Moving_mask;
    mask_const = (T_mat >= T1_mat) & (T_mat < T2_mat) & Moving_mask;
    mask_dec = (T_mat >= T2_mat) & (T_mat <= T_sync) & Moving_mask;
    mask_end = (T_mat > T_sync) & Moving_mask;
    
    % --- 加速段 ---
    if any(mask_acc(:))
        ti = T_mat(mask_acc);
        aa = A_mat(mask_acc);
        dd = Dir_mat(mask_acc);
        qq0 = Q0_mat(mask_acc);
        
        ACC(mask_acc) = dd .* aa;
        V(mask_acc)   = dd .* aa .* ti;
        Q(mask_acc)   = qq0 + dd .* 0.5 .* aa .* ti.^2;
    end
    
    % --- 匀速段 ---
    if any(mask_const(:))
        ti = T_mat(mask_const);
        t1_val = T1_mat(mask_const);
        vv = V_lim_mat(mask_const);
        aa = A_mat(mask_const);
        dd = Dir_mat(mask_const);
        qq0 = Q0_mat(mask_const);
        
        ACC(mask_const) = 0;
        V(mask_const)   = dd .* vv;
        
        dist_acc = 0.5 .* aa .* t1_val.^2;
        dist_const = vv .* (ti - t1_val);
        Q(mask_const) = qq0 + dd .* (dist_acc + dist_const);
    end
    
    % --- 减速段 ---
    if any(mask_dec(:))
        ti = T_mat(mask_dec);
        t1_val = T1_mat(mask_dec);
        t2_val = T2_mat(mask_dec);
        tc_val = t2_val - t1_val;
        vv = V_lim_mat(mask_dec);
        aa = A_mat(mask_dec);
        dd = Dir_mat(mask_dec);
        qq0 = Q0_mat(mask_dec);
        
        ACC(mask_dec) = -dd .* aa;
        
        t_dec = ti - t2_val;
        V(mask_dec) = dd .* (vv - aa .* t_dec);
        
        dist_acc = 0.5 .* aa .* t1_val.^2;
        dist_const = vv .* tc_val;
        dist_dec = vv .* t_dec - 0.5 .* aa .* t_dec.^2;
        
        Q(mask_dec) = qq0 + dd .* (dist_acc + dist_const + dist_dec);
    end
    
    % --- 结束段 ---
    if any(mask_end(:))
        Q1_mat = repmat(q1, N, 1);
        Q(mask_end) = Q1_mat(mask_end);
        V(mask_end) = 0;
        ACC(mask_end) = 0;
    end
    
    % 不动轴保持原位，速度和加速度为0
    Q(~Moving_mask) = Q0_mat(~Moving_mask);
    V(~Moving_mask) = 0;
    ACC(~Moving_mask) = 0;
    
    % 强制修正最后一点
    Q(end, :) = q1;
    V(end, :) = 0;
    ACC(end, :) = 0;
end