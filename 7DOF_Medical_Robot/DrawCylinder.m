function h = DrawCylinder(T_global, radius, len, col)
% T_global: 4x4 全局齐次矩阵（关节的世界位姿）
% radius, len, col

    % 1) 在“局部坐标系”生成一个标准圆柱（沿局部 z 轴）
    nSide = 20;
    theta = linspace(0, 2*pi, nSide+1);

    x = [radius*cos(theta); radius*cos(theta)];
    y = [radius*sin(theta); radius*sin(theta)];
    z = [ len/2*ones(1,nSide+1); 
         -len/2*ones(1,nSide+1)];

    % 2) 把局部点堆成 3xN，然后用T变换到世界坐标
    Xw = zeros(size(x));
    Yw = zeros(size(y));
    Zw = zeros(size(z));

    for k = 1:size(x,1)
        pts_local = [x(k,:); y(k,:); z(k,:);ones(1,nSide+1)];   % 4xN
        pts_world = T_global*pts_local;          % 4xN  (这一步就是 T_global 做的事)

        Xw(k,:) = pts_world(1,:);
        Yw(k,:) = pts_world(2,:);
        Zw(k,:) = pts_world(3,:);
    end
    c = col*ones(size(x));
    % 3) surf 画侧面
    h = surf(Xw, Yw, Zw,c);
   
    % 4) patch 封上下端面
    hold on;
    patch(Xw(1,:), Yw(1,:), Zw(1,:),c(1,:));
    patch(Xw(2,:), Yw(2,:), Zw(2,:), c(2,:));
end
