function workspace_points = WorkSpace_Create(num_samples)
% 计算7自由度机械臂的工作空间
% 输入：
%   DH_params: 7x4矩阵，[a, alpha, d, theta] 标准DH参数
%   joint_limits: 7x2矩阵，各关节角度范围 [min, max] (弧度)
%   num_samples: 随机采样点数
% 输出：
%   workspace_points: num_samplesx3矩阵，末端执行器在基坐标系中的位置
%   fig: 图形句柄

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

joint_limits = [0,320;
                0,180;
                270,360;
                0,300;
                0,360;
                0,360;
                0,360];
% 预分配内存
workspace_points = zeros(num_samples, 3);

% 蒙特卡洛法采样
for i = 1:num_samples
    % 生成随机关节角
    joint_angles = joint_limits(:,1) + rand(7,1).*diff(joint_limits, 1, 2);
    
    % 计算正向运动学
    T = T07(joint_angles);
    pos = T(1:3, 4)';
    workspace_points(i, :) = pos;
end

% 可视化
plot_workspace_surface_alphashape(workspace_points, 70);
plot_workspace_slices(workspace_points);
% 保存点云数据（可选）
% save('workspace_data.mat', 'workspace_points');
end

function plot_workspace_surface_alphashape(points, alpha_value)
    % points: N×3的矩阵，包含末端执行器位置
    % alpha_value: alpha半径，控制曲面紧密度
    
    if nargin < 2
        alpha_value = 50; % 默认alpha值，需要根据数据调整
    end
    
    % 创建alphaShape对象
    shp = alphaShape(points, alpha_value);
    
    % 绘制曲面
    figure('Name', '7DOF机械臂工作空间','Position', [100, 100, 1200, 500]);
    
    % 子图1：点云
    subplot(1,2,1);
    scatter3(points(:,1), points(:,2), points(:,3), 1, 'b', 'filled');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('原始采样点');
    axis equal; grid on; view(45,30);
    
    % 子图2：曲面
    subplot(1,2,2);
    plot(shp, 'FaceColor', [0.3, 0.7, 1.0], 'EdgeColor', 'none', ...
         'FaceAlpha', 0.6, 'AmbientStrength', 0.5);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(['工作空间曲面 (α=', num2str(alpha_value), ')']);
    axis equal; grid on; view(45,30);
    
    % 添加光照效果
    lighting gouraud;
    camlight('headlight');
    material('dull');
    
end

function plot_workspace_slices(points)
    % 在不同Z平面查看工作空间
    figure("Name","剖面图");
    
    % XY平面切片
    subplot(2,2,1);
    scatter(points(:,1), points(:,2), 1, 'b', 'filled');
    xlabel('X'); ylabel('Y'); title('XY平面投影');
    axis equal;
    
    % XZ平面切片
    subplot(2,2,2);
    scatter(points(:,1), points(:,3), 1, 'r', 'filled');
    xlabel('X'); ylabel('Z'); title('XZ平面投影');
    axis equal;
    
    % YZ平面切片
    subplot(2,2,3);
    scatter(points(:,2), points(:,3), 1, 'g', 'filled');
    xlabel('Y'); ylabel('Z'); title('YZ平面投影');
    axis equal;
end