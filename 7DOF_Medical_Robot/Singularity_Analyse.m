function [singular_points, end_positions] = Singularity_Analyse(Link, N)
% SINGULARITY_ANALYSE 分析7轴机械臂奇异点
% 输入：
%   Link: 机械臂连杆参数结构体
%   N: 采样点总数
% 输出：
%   singular_points: M×7矩阵，奇异点关节配置
%   end_positions: M×3矩阵，奇异点对应的末端位置

T01 = @(theta) [cosd(theta)  -sind(theta)   0      0;
                sind(theta)   cosd(theta)   0      0;
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

T56 = @(theta) [cosd(theta)  -sind(theta)   0      0;
                  0               0        -1      0;
                sind(theta)   cosd(theta)   0      0;
                   0             0          0      1];

T67 = @(theta) [cosd(theta)  -sind(theta)   0      0;
                  0               0         1      0;
                -sind(theta) -cosd(theta)   0      0;
                   0             0          0      1];     

T07 = @(q) T01(q(1)) * T12(q(2)) * T23(q(3)) * T34(q(4)+150) * T45(q(5)) * T56(q(6)) * T67(q(7));

    % ========== 参数设置 ==========
SINGULARITY_THRESHOLD = 1e-4;  % 奇异点判断阈值
    
    % 关节限制（根据你的机械臂调整）
joint_limits = [0,320;
                0,180;
                270,360;
                0,300;
                0,360;
                0,360;
                0,360];
    
    % ========== 随机采样生成 ==========
    fprintf('生成 %d 个随机采样点...\n', N);
    Q_samples = zeros(N, 7);
    
    for i = 1:N
        for j = 1:7
            Q_samples(i, j) = joint_limits(j, 1) + ...
                rand() * (joint_limits(j, 2) - joint_limits(j, 1));
        end
    end
    
    % ========== 奇异点检测 ==========
    fprintf('开始奇异点检测...\n');
    
    singular_points = [];
    end_positions = [];
    
    for i = 1:N
        q = Q_samples(i, :)';
        
        % 计算雅可比矩阵
        J = Jocobian_Build(Link, q);
        
        % 计算最小奇异值
        sigma = svd(J);
        min_sigma = min(sigma);
        
         if min_sigma < SINGULARITY_THRESHOLD
            % 检查雅可比矩阵的秩
            rank_J = rank(J, 1e-4);
            
            % 7轴机械臂的雅可比应该是6×7，满秩应为6
            % 如果秩小于6，说明确实奇异
            if rank_J < 6
                % 计算条件数作为辅助验证
                cond_J = cond(J);
                
                % 只有条件数也很大时才认为是奇异点
                if cond_J > 1000  % 条件数阈值
                    % 记录奇异点
                    singular_points = [singular_points; q'];
                    
                    % 计算末端位置
                    T = T07(q);
                    end_pos = T(1:3, 4)';
                    end_positions = [end_positions; end_pos];
                end
            end
        end
        
        % 显示进度
        if mod(i, 500) == 0
            fprintf('已处理 %d/%d，发现奇异点: %d\n', ...
                i, N, size(singular_points, 1));
        end
    end
    
    fprintf('分析完成！发现奇异点: %d 个\n', size(singular_points, 1));
    
    % ========== 可视化 ==========
    if ~isempty(singular_points)
        % 计算所有采样点的末端位置
        all_end_positions = zeros(N, 3);
        for i = 1:N
            T = T07(Q_samples(i, :)');
            all_end_positions(i, :) = T(1:3, 4)';
        end
        
        % 调用可视化子函数
        visualize_singularities(all_end_positions, end_positions);
    else
        fprintf('未发现奇异点！\n');
    end
end

%% ========== 可视化子函数 ==========
function visualize_singularities(all_end_positions, singular_end_positions)
    % 创建新的图形窗口
    figure;
    
    % 子图1：工作空间中的奇异点分布（三维散点图）
    hold on;
    grid on;
    
    % 绘制所有采样点的末端位置（蓝色，半透明）
    scatter3(all_end_positions(:,1), all_end_positions(:,2), all_end_positions(:,3), ...
        10, [0.2, 0.4, 0.8], 'filled', 'MarkerFaceAlpha', 0.1);
    
    % 绘制奇异点的末端位置（红色）
    scatter3(singular_end_positions(:,1), singular_end_positions(:,2), ...
        singular_end_positions(:,3), 40, 'r', 'filled', 'MarkerEdgeColor', 'k');
    
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    legend('所有采样点', '奇异点', 'Location', 'best');
    view(45, 30);
    axis equal;
    
    % 调整布局
    sgtitle('7轴机械臂奇异点分析结果', 'FontSize', 14, 'FontWeight', 'bold');
    
end





