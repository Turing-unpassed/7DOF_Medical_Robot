function [phi, theta, psi] = XYZ_FixedAngle_Solution(T)
%XYZ_EulerAngle_Solution  Extract Euler angles for extrinsic X-Y-Z (roll-pitch-yaw)
%  [phi,theta,psi] = XYZ_EulerAngle_Solution(T)
%  T: 3x3 (R) or 4x4 homogeneous transform
%  Outputs: phi,theta,psi are 2x1 vectors (degrees), two possible solutions.
%
%  Rotation convention: R = Rz(psi) * Ry(theta) * Rx(phi)
%  so: phi = roll (about X), theta = pitch (about Y), psi = yaw (about Z)

    if all(size(T) == [4 4])
        R = T(1:3,1:3);
    elseif all(size(T) == [3 3])
        R = T;
    else
        error('Input must be 3x3 or 4x4 matrix.');
    end

    eps_val = 1e-9;

    % 标量提取（使用度）
    % r31 = -sin(theta)  => theta = asind(-r31)
    r31 = R(3,1);
    theta1 = asind( max(min(-r31,1),-1) );   % principal value in [-90,90]
    theta2 = 180 - theta1;

    % 判断是否为奇异（cos(theta) ≈ 0）
    if abs(cosd(theta1)) > eps_val
        % 一般情况
        % phi = atan2(r32, r33)
        % psi = atan2(r21, r11)
        phi1 = atan2d( R(3,2), R(3,3) );
        psi1 = atan2d( R(2,1), R(1,1) );
    else
        % 奇异情况：theta ≈ ±90°，cos(theta)≈0，phi 与 psi 不是唯一的
        % 采用常用约定：令 phi = 0，把多余旋转并入 psi
        if abs(theta1) < eps_val
            % theta ≈ 0 的情形通常不会落到这里（cos ≈1），但保留保护
            theta1 = 0;
            phi1 = 0;
            psi1 = atan2d( -R(1,2), R(1,1) );
        else
            % theta ≈ ±90
            phi1 = 0;
            % 从非零子块求 psi：用元素 (1,2) (2,2) 的组合
            psi1 = atan2d( -R(1,2), R(2,2) );
        end
    end

    % 第二组解（对 theta 的另一个三角解对应 phi,psi 加 180）
    phi2 = phi1 + 180;
    psi2 = psi1 + 180;

    % 规范化到 0..360 (或你需要的范围)
    phi = NormalizeAngle([phi1; phi2]);
    theta = NormalizeAngle([theta1; theta2]);
    psi = NormalizeAngle([psi1; psi2]);
end