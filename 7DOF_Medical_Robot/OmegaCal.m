function omega = OmegaCal(q_prev, q, dt)
% 输入:
%   q_prev - 7x1 关节角度向量（degree/mm）
%   q      - 7x1 关节角度向量（degree/mm）
%   dt     - 采样时间间隔（s）
% 输出:
%   omega  - 7x1 角速度向量（rad/s)&(m/s)

ToRad = pi/180;
ToM = 0.001;

q_prev = q_prev(:)';
q = q(:)';
if numel(q_prev) ~= 7 || numel(q) ~= 7
    error('q_prev and q must be length-7 vectors');
end
if dt <= 0
    error('dt must be positive');
end

% joint types: revolute = [1,2,3,5,6,7], prismatic = 4
rev_idx = [1,2,3,5,6,7];
pr_idx = 4;

q_prev(rev_idx) = q_prev(rev_idx)*ToRad;
q(rev_idx) = q(rev_idx)*ToRad;
q_prev(pr_idx) = q_prev(pr_idx)*ToM;
q(pr_idx) = q(pr_idx)*ToM;


omega = zeros(7,1);

 % shortest difference in radians
angdiff_rad = @(a,b) mod((a - b) + pi, 2*pi) - pi;
dtheta_rad = angdiff_rad(q(rev_idx), q_prev(rev_idx));
omega(rev_idx) = dtheta_rad / dt;  %rad/s
omega(pr_idx) = (q(pr_idx) - q_prev(pr_idx)) / dt;  %m/s

end