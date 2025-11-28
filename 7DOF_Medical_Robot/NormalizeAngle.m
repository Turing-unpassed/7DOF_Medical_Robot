function normalized_angle = NormalizeAngle(angle)
% NORMALIZEANGLE360 将角度归一化到[0, 360)度范围
%   输入：angle - 任意角度值（标量或矩阵）
%   输出：normalized_angle - 归一化后的角度值

normalized_angle = mod(angle, 360);
normalized_angle(normalized_angle < 0) = normalized_angle(normalized_angle < 0) + 360;
end