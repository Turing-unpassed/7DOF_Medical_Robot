function [Q,V,ACC]=Tplanning(q0, q1, v_max, A, dt)
% TPLANNING 梯形速度规划函数
% 输入:
%   q0    : 起始位置 
%   q1    : 目标位置 
%   v_max : 最大允许速度 
%   A     : 最大允许加速度 (改为A提高可读性)
%   dt    : 采样时间间隔 (例如 0.001)
%
% 输出:
%   Q     : 位置数组
%   V     : 速度数组
%   ACC   : 加速度数组


    if v_max <= 0 || A <= 0 || dt <= 0
        error('Error: v_max, A 和 dt 必须为正数');
    end

    % 计算位移总距离
    dist = q1 - q0;
    % 如果起点和终点相同，直接返回
    if abs(dist) < 1e-10
        Q = q0 * ones(1, 1);
        V = 0 * ones(1, 1);
        ACC = 0 * ones(1, 1);
        return;
    end

    % 确定运动方向并提取位移的绝对值 (1 为正向, -1 为负向)
    dir = sign(dist);
    L = abs(dist);
    
    
    if (v_max^2 / A) <= L
        % 梯形速度规划
        v_lim = v_max;              % 实际能达到的最大速度
        ta = v_lim / A;             % 加速时间
        La = 0.5 * A * ta^2;        % 加速段距离
        L_const = L - 2 * La;       % 匀速段距离
        tc = L_const / v_lim;       % 匀速时间
        
        T_total = 2 * ta + tc;      % 总时间
    else
        % 三角形速度规划
        ta = sqrt(L / A);
        v_lim = A * ta;             % 此时达到的峰值速度 (< v_max)
        tc = 0;                     % 匀速时间为 0
        
        T_total = 2 * ta;           % 总时间
    end
    
    % 生成离散时间序列
    % 向上取整确保覆盖整个时间段，最后再修正终点
    steps = ceil(T_total / dt);
    t = (0:steps)' * dt; 

    N = length(t);
    Q = zeros(N, 1);
    V = zeros(N, 1);
    ACC = zeros(N, 1);
    
    % 逐点生成轨迹
    t1 = ta;            % 加速结束时刻
    t2 = ta + tc;       % 匀速结束时刻 
    
    for i = 1:N
        ti = t(i);
        
        if ti < t1
            %  加速阶段 
            ACC(i) = dir * A;
            V(i)   = dir * A * ti;
            Q(i)   = q0 + dir * 0.5 * A * ti^2;
            
        elseif ti >= t1 && ti < t2
            %  匀速阶段 
            ACC(i) = 0;
            V(i)   = dir * v_lim;
            
            dist_acc = 0.5 * A * t1^2;
            dist_const = v_lim * (ti - t1);
            
            Q(i)   = q0 + dir * (dist_acc + dist_const);
            
        elseif ti >= t2 && ti <= T_total
            %  减速阶段 
            ACC(i) = -dir * A;
            
            t_dec = ti - t2;
            V(i) = dir * (v_lim - A * t_dec);
            
            dist_acc = 0.5 * A * t1^2;
            dist_const = v_lim * tc;
            dist_dec = v_lim * t_dec - 0.5 * A * t_dec^2;
            
            Q(i) = q0 + dir * (dist_acc + dist_const + dist_dec);
            
        else
            %  超出规划时间 (防止浮点误差导致多出的点) 
            ACC(i) = 0;
            V(i) = 0;
            Q(i) = q1; 
        end
    end
    
    % 强制将最后一个点设为目标状态，消除离散化带来的微小误差
    Q(end) = q1;
    V(end) = 0;
    ACC(end) = 0;
    
    % 说明：如果 ceil(T_total/dt) 导致时间超过 T_total，最后几个点
    % 仍会被正确处理，因为我们已经在 else 分支中处理了超时的情况
end