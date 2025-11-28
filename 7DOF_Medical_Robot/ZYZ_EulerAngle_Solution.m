function [alpha, beta, gamma] = ZYZ_EulerAngle_Solution(T)
%T2ZYZ  Extract Z-Y-Z Euler angles from a homogeneous transform.
%   T : 4x4 或 3x3，齐次变换矩阵或旋转矩阵
%   R : 旋转部分满足  R = Rz(alpha)*Ry(beta)*Rz(gamma)
%
%   输出：alpha, beta, gamma 均为角度

    % 允许传 4x4 或 3x3
    if all(size(T) == [4 4])
        R = T(1:3, 1:3);
    elseif all(size(T) == [3 3])
        R = T;
    else
        error('Input must be 3x3 or 4x4 matrix.');
    end

    % 数值容差
    eps_val = 1e-9;

    % β = acos(R33)
    beta1 = acosd(max(min(R(3,3), 1), -1));  % 防止数值超出 [-1,1]
    beta2 = 360-beta1;
    % 判断是否奇异（sin(beta) ≈ 0）
    if abs(sind(beta1)) > eps_val
        % 一般情况：
        % R13 = cos(alpha)*sin(beta)
        % R23 = sin(alpha)*sin(beta)
        alpha1 = atan2d(R(2,3), R(1,3));

        % R31 = -sin(beta)*cos(gamma)
        % R32 = sin(beta)*sin(gamma)
        gamma1 = atan2d(R(3,2),-R(3,1));
    else
        % 奇异情况：beta ≈ 0 或 beta ≈ pi
        % 这里只给出一种常用约定：令 alpha = 0，把所有转动并到 gamma 里
        if abs(beta1) < eps_val
            beta1 = 0;
            alpha1 = 0;
            % 此时 R ≈ Rz(alpha + gamma) = Rz(gamma)
            gamma1 = atan2d(-R(1,2), R(1,1));
        else
            % beta ≈ pi
            beta1 = 180;
            alpha1 = 0;
            % 此时 R ≈ Rz(alpha - gamma)*Ry(pi),
            % 一个常见处理：把符号翻一下再用 atan2
            gamma1 = atan2d(R(1,2), -R(1,1));
        end
    end

    alpha2=alpha1+180;
    gamma2 =gamma1+180;
    
    alpha = [alpha1;alpha2];
    beta = [beta1;beta2];
    gamma = [gamma1;gamma2];
end