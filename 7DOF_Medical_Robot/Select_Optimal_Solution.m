function [q_best, d_idx, cand_idx] = Select_Optimal_Solution(Q, q_prev)
% 从 Q(8x7xN) 中挑出一组解，使相对于 q_prev 的关节移动之和最小（只返回一组解）
% 输入:
%   Q      - 8 x 7 x N 矩阵，NaN 表示该候选不可达
%   q_prev - 1x7 或 7x1 上一采样点关节向量（角度按度，滑移关节为列4）
% 输出:
%   q_best   - 1x7 所选解，若无可行解则为 NaN(1,7)
%   d_idx    - 第三维索引（采样点索引，1..N），无解时为 0
%   cand_idx - 候选行索引（1..8），无解时为 0

if nargin < 2
    error('需要 Q 和 q_prev 两个输入');
end
q_prev = reshape(q_prev,1,7);

[~,~,N] = size(Q);
angIdx = [1 2 3 5 6 7]; % 角度关节
linIdx = 4;             % 线性/滑移关节

% 最短角差函数（-180..180）
angdiff = @(a,b) mod((a - b) + 180, 360) - 180;

bestCost = inf;
q_best = NaN(1,7);
d_idx = 0;
cand_idx = 0;

for k = 1:N
    C = squeeze(Q(:,:,k)); % 8 x 7
    if all(isnan(C(:))), continue; end
    for r = 1:8
        row = C(r,:);
        if ~all(isfinite(row)), continue; end
        % 角度差（取模最短差）
        a_diff = abs( angdiff(row(angIdx), q_prev(angIdx)) );
        l_diff = abs( row(linIdx) - q_prev(linIdx) );
        cost = sum(a_diff) + l_diff;  % 简单加权（可调整权重）
        if cost < bestCost
            bestCost = cost;
            q_best = row;
            d_idx = k;
            cand_idx = r;
        end
    end
end

% 若未找到合法解，保持 NaN 与 0 索引
end