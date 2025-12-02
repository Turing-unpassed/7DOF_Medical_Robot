close all;
clear;
clc;

% =====================================================
% Tplanning 函数测试脚本
% =====================================================

% 采样时间间隔 (50ms)
dt = 0.05;

% 初始位置 [度数/毫米]
q0 = [0,0,0,0,0,0];

% 目标位置 [度数/毫米]
qf = [15,30,40,20,30,40];

% 各关节最大速度 [度/秒 或 mm/秒]
v_max = [20,20,20,20,20,20];

% 各关节最大加速度 [度/秒² 或 mm/秒²]
a_max = [25,25,25,25,25,25];

% =====================================================
% 调用 Tplanning 函数
% =====================================================
[Q, V, ACC] = Tplanning(q0, qf, v_max, a_max, dt);

% 生成时间向量
N = size(Q, 1);
t = (0:N-1)' * dt;

% =====================================================
% 绘图
% =====================================================
dof_names = {'DOF1 (X)', 'DOF2 (Y)', 'DOF3 (Z)', 'DOF4 (n)', 'DOF5 (o)', 'DOF6 (a)'};
colors = lines(6);  % 使用MATLAB内置颜色

% --- 图1: 位置曲线 ---
figure('Name', '位置曲线', 'NumberTitle', 'off', 'Position', [100, 100, 900, 600]);
for i = 1:6
    subplot(2, 3, i);
    plot(t, Q(:, i), 'Color', colors(i,:), 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('位置');
    title(dof_names{i});
    grid on;
end
sgtitle('各自由度位置曲线', 'FontSize', 14, 'FontWeight', 'bold');

% --- 图2: 速度曲线 ---
figure('Name', '速度曲线', 'NumberTitle', 'off', 'Position', [150, 150, 900, 600]);
for i = 1:6
    subplot(2, 3, i);
    plot(t, V(:, i), 'Color', colors(i,:), 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('速度');
    title(dof_names{i});
    grid on;
    % 标注最大速度限制
    hold on;
    yline(v_max(i), 'r--', 'v_{max}', 'LineWidth', 1);
    yline(-v_max(i), 'r--', '-v_{max}', 'LineWidth', 1);
    hold off;
end
sgtitle('各自由度速度曲线', 'FontSize', 14, 'FontWeight', 'bold');

% --- 图3: 加速度曲线 ---
figure('Name', '加速度曲线', 'NumberTitle', 'off', 'Position', [200, 200, 900, 600]);
for i = 1:6
    subplot(2, 3, i);
    plot(t, ACC(:, i), 'Color', colors(i,:), 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('加速度');
    title(dof_names{i});
    grid on;
    % 标注最大加速度限制
    hold on;
    yline(a_max(i), 'r--', 'a_{max}', 'LineWidth', 1);
    yline(-a_max(i), 'r--', '-a_{max}', 'LineWidth', 1);
    hold off;
end
sgtitle('各自由度加速度曲线', 'FontSize', 14, 'FontWeight', 'bold');

% --- 图4: 所有自由度对比图 ---
figure('Name', '综合对比', 'NumberTitle', 'off', 'Position', [250, 250, 1000, 700]);

% 位置对比
subplot(3, 1, 1);
hold on;
for i = 1:6
    plot(t, Q(:, i), 'LineWidth', 1.5, 'DisplayName', dof_names{i});
end
hold off;
xlabel('时间 (s)');
ylabel('位置');
title('位置曲线对比');
legend('Location', 'best');
grid on;

% 速度对比
subplot(3, 1, 2);
hold on;
for i = 1:6
    plot(t, V(:, i), 'LineWidth', 1.5, 'DisplayName', dof_names{i});
end
hold off;
xlabel('时间 (s)');
ylabel('速度');
title('速度曲线对比');
legend('Location', 'best');
grid on;

% 加速度对比
subplot(3, 1, 3);
hold on;
for i = 1:6
    plot(t, ACC(:, i), 'LineWidth', 1.5, 'DisplayName', dof_names{i});
end
hold off;
xlabel('时间 (s)');
ylabel('加速度');
title('加速度曲线对比');
legend('Location', 'best');
grid on;

sgtitle('6自由度同步梯形速度规划结果', 'FontSize', 14, 'FontWeight', 'bold');

% =====================================================
% 打印规划信息
% =====================================================
fprintf('\n========== 轨迹规划结果 ==========\n');
fprintf('总时间: %.3f 秒\n', t(end));
fprintf('采样点数: %d\n', N);
fprintf('采样间隔: %.3f 秒\n', dt);
fprintf('\n--- 各轴运动参数 ---\n');
for i = 1:6
    fprintf('%s: 起点=%.2f, 终点=%.2f, 位移=%.2f\n', ...
        dof_names{i}, q0(i), qf(i), qf(i)-q0(i));
end
fprintf('\n--- 实际达到的最大速度 ---\n');
for i = 1:6
    fprintf('%s: %.2f (限制: %.2f)\n', dof_names{i}, max(abs(V(:,i))), v_max(i));
end
fprintf('\n--- 实际达到的最大加速度 ---\n');
for i = 1:6
    fprintf('%s: %.2f (限制: %.2f)\n', dof_names{i}, max(abs(ACC(:,i))), a_max(i));
end
fprintf('==================================\n');

