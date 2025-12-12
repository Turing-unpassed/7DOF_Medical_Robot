function omega_base = RotationRateCal(R1, R2, dt)
% 输入:
%   R1, R2 - 3x3 旋转矩阵（表示末端在时刻1/2相对于基坐标系的旋转）
%   dt     - 采样时间间隔（s）
% 输出:
%   omega_base - 3x1 角速度向量（rad/s），在基坐标系表示

    % 相对旋转
    Rrel = R2 * R1';
    % 计算角度 (数值稳健)
    tr = (trace(Rrel) - 1) / 2;
    tr = max(-1, min(1, tr));          % 裁剪数值在[-1,1]
    angle = acos(tr);                  % angle in [0, pi]
    if angle < 1e-8
        % 小角近似： rotVec ≈ 0.5 * vee(Rrel - Rrel')
        S = 0.5 * (Rrel - Rrel');
        rotVec = [S(3,2); S(1,3); S(2,1)]; % 近似的 axis*angle (rad)
    else
        % 精确公式： axis = (1/(2*sin(angle))) * [R32-R23; R13-R31; R21-R12]
        axis = (1/(2*sin(angle))) * [ Rrel(3,2)-Rrel(2,3);
                                       Rrel(1,3)-Rrel(3,1);
                                       Rrel(2,1)-Rrel(1,2) ];
        rotVec = axis * angle;        % rotation vector (rad)
    end
    omega_base = rotVec / dt;         % rad/s, in base frame
end