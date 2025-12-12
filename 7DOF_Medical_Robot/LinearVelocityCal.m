function v = LinearVelocityCal(p1, p2, dt)
    % 输入:
    %   p1, p2 - 3x1 位置向量（mm）
    %   dt     - 采样时间间隔（s）
    % 输出:
    %   v      - 3x1 线速度向量（m/s）
    v(1) = (p2(1) - p1(1)) * 0.001 / dt; % vx
    v(2) = (p2(2) - p1(2)) * 0.001 / dt; % vy
    v(3) = (p2(3) - p1(3)) * 0.001 / dt; % vz
    v = v(:);
end