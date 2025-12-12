function EndPoint_HeartTrajectory(Link)
% EndPoint_HeartTrajectory  Draw a red heart with the robot end-effector
% Usage: EndPoint_HeartTrajectory(Link)
% If Link is not provided, the function will build the MDH table.

if nargin < 1 || isempty(Link)
    if exist('MDH_Table_Build','file')
        Link = MDH_Table_Build();
    else
        error('Link not provided and MDH_Table_Build not found');
    end
end

% Number of sampled points along the heart curve
nPts = 200;
% param t
t = linspace(0,2*pi,nPts);

% Classic heart shape (2D) scale and offset to robot workspace (mm)
scale = 10;                % tune if required for your robot workspace
cx = 300; cy = 0; cz = 200; % center position in workspace (mm)

% Parametric heart (x,y). Use common cardioid-like polynomial
X = scale * 16 * (sin(t)).^3;
Y = scale * (13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t));
Z = cz + zeros(size(t));   % constant height; change if you want 3D variation

% Preallocate storage
traj_q = NaN(nPts,7);
traj_p = [X; Y; Z]';

% initial previous joint (home) -- try to get a feasible initial via IK of first point
q_prev = zeros(1,7);

% Create figure with robot view and heart plot
fig = figure('Name','End-effector Heart Trajectory','NumberTitle','off');
clf(fig);
ax1 = subplot(1,2,1);
ax2 = subplot(1,2,2);

% Prepare heart plot (right): full curve in light red and dynamic marker
axes(ax2);
hold on; grid on; axis equal;
plot(X+cx, Y+cy, 'Color',[1 0.6 0.6], 'LineWidth',1.5);
hCurr = plot(X(1)+cx, Y(1)+cy, 'ro', 'MarkerFaceColor','r');
title('Heart path (XY)'); xlabel('X (mm)'); ylabel('Y (mm)');

% Main loop: for each target point compute IK and animate
for k = 1:nPts
    % target transform (fixed orientation: identity rotation)
    P = [X(k)+cx; Y(k)+cy; Z(k)];
    R = eye(3);
    T07 = [R, P; 0 0 0 1];

    % call IK (may return multiple d4 slices -> 3rd dim)
    try
        Qcand = Geometric_Inverse_Kinematics(Link, T07);
    catch ME
        warning('Geometric_Inverse_Kinematics failed at point %d: %s', k, ME.message);
        continue
    end

    % select single best solution minimizing joint movement from q_prev
    [q_best, d_idx, cand_idx] = Select_Optimal_Solution(Qcand, q_prev);
    if isempty(q_best) || all(~isfinite(q_best))
        % no valid solution for this point; skip plotting but keep marker
        set(hCurr, 'XData', X(k)+cx, 'YData', Y(k)+cy);
        drawnow limitrate;
        continue
    end

    traj_q(k,:) = q_best;
    q_prev = q_best; % update previous

    % draw robot in left axes using Forward_kinematics (assumed to plot when flag=1)
    axes(ax1); cla(ax1); hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
    try
        Forward_kinematics(Link, q_best, 1); % if your FK supports plotting with flag
    catch
        % fallback: just plot current end position
        plot3(P(1),P(2),P(3),'ro');
    end
    title(ax1,'Robot configuration');

    % update heart marker on right plot
    axes(ax2);
    set(hCurr, 'XData', X(k)+cx, 'YData', Y(k)+cy);

    drawnow limitrate;
    pause(0.02);
end

% after loop, draw full tracked points in red
axes(ax2);
hold on;
valid = all(isfinite(traj_q),2);
plot(traj_p(valid,1)+cx, traj_p(valid,2)+cy, 'r-', 'LineWidth',1.8);
title('End-effector heart (tracked points)');

fprintf('Heart trajectory finished. %d/%d points tracked.\n', sum(valid), nPts);
end
