function trajref = generate_trajref(trajref_params, roadmap_name)
% 生成参考路径 trajref = [X, X, Theta, Radius, Curvature]

% 输出:
% X         : 横坐标, m
% Y         : 纵坐标, m
% Theta     : 航向角，与横坐标轴的角度，逆时针0~2*pi, rad
% Radius    : 点的半径, m
% Curvature : 点的曲率

% 输入:
% trajref_params : 期望路径的相关参数

dist_interval = trajref_params.dist_interval;   %trajref两点距离间隔, m
traj1_dist = trajref_params.traj1_dist;         %traj1的长度, m
if (~contains(roadmap_name,'wave_test'))
    r2 = trajref_params.r2;                         %traj2的半径, m
    traj3_dist = trajref_params.traj3_dist;         %traj3的长度, m
    r4 = trajref_params.r4;                         %traj4的半径, m
    r5 = trajref_params.r5;                         %traj5的半径, m
    traj6_dist = trajref_params.traj6_dist;         %traj6的长度, m
end

default_r = 10000;    %直道上点的默认半径

% traj1的点的位置、航向、半径值
X1 = (0 : dist_interval : traj1_dist)';  %traj1的点的x坐标
n1 = length(X1);                %traj1的点的个数?
Y1 = zeros(n1, 1);              %traj1的点的y坐标
T1 = zeros(n1, 1);              %traj1的点的航向角
R1 = ones(n1, 1) * default_r;   %traj1的点的曲率半径
C1 = 1 ./ R1;                   %traj1的点的曲率
traj1 = [X1, Y1, T1, R1, C1];   %traj1的点

if (~contains(roadmap_name,'wave_test'))
    % traj2的点的位置、航向、半径值
    theta_interval_2 = dist_interval / r2;  %角度间隔, rad
    Theta_2 = (-pi/2 : theta_interval_2 : pi/2)';
    X2 = r2 * cos(Theta_2) + traj1(end, 1);
    Y2 = r2 * sin(Theta_2) + traj1(end, 2) + r2;
    n2 = length(X2);
    T2 = zeros(n2, 1);
    R2 = ones(n2, 1) * r2;
    C2 = 1 ./ R2; 
    traj2 = [X2, Y2, T2, R2, C2];

    % traj3的点的位置、航向、半径值
    X3 = (traj2(end, 1) : -dist_interval : (traj2(end, 1) - traj3_dist))';
    n3 = length(X3);
    Y3 = ones(n3, 1) * traj2(end, 2);
    T3 = zeros(n3, 1);
    R3 = ones(n3, 1) * default_r;
    C3 = 1 ./ R3; 
    traj3 = [X3, Y3, T3, R3, C3];

    % traj4的点的位置、航向、半径值
    theta_interval_4 = dist_interval / r4;  %角度间隔, rad
    Theta_4 = (3/2*pi : -theta_interval_4 : pi/2)';
    X4 = r4 * cos(Theta_4) + traj3(end, 1);
    Y4 = r4 * sin(Theta_4) + traj3(end, 2) + r4;
    n4 = length(X4);
    T4 = zeros(n4, 1);
    R4 = ones(n4, 1) * -r4;
    C4 = 1 ./ R4; 
    traj4 = [X4, Y4, T4, R4, C4];

    % traj5的点的位置、航向、半径值
    theta_interval_5 = dist_interval / r5;  %角度间隔, rad
    Theta_5 = (-pi/2 : theta_interval_5 : 0)';
    X5 = r5 * cos(Theta_5) + traj4(end, 1);
    Y5 = r5 * sin(Theta_5) + traj4(end, 2) + r5;
    n5 = length(X5);
    T5 = zeros(n5, 1);
    R5 = ones(n5, 1) * r5;
    C5 = 1 ./ R5;
    traj5 = [X5, Y5, T5, R5, C5];

    % traj6的点的位置、航向、半径值
    Y6 = (traj5(end, 2) : dist_interval : (traj5(end, 2) + traj6_dist))';
    n6 = length(Y6);
    X6 = ones(n6, 1) * traj5(end, 1);
    T6 = zeros(n6, 1);
    R6 = ones(n6, 1) * default_r;
    C6 = 1 ./ R6; 
    traj6 = [X6, Y6, T6, R6, C6];

    % 将上述traj连起来，组成trajref
    trajref = [traj1(1:n1-1, :); traj2(1:n2-1, :); traj3(1:n3-1, :);...
        traj4(1:n4-1, :); traj5(1:n5-1, :); traj6];
else
    trajref = [traj1(1:n1-1, :)];
end

% 计算trajref每点的航向角
n = length(trajref);
for i = 1 : 1 : (n-1)
    point1 = trajref(i, 1:2);
    point2 = trajref(i+1, 1:2);
    heading_angle = calc_heading_angle(point1, point2);
    trajref(i, 3) = heading_angle;
end
trajref(n, 3) = trajref(n-1, 3);

%% 平滑trajref每点的曲率及半径
figure('name', 'Path Curvature');
title('Path Curvature', 'fontsize', 15);  %设置title名称
hold on;
plot(trajref(:, 5), 'r');

% 1. 取前面10个点的曲率平均值?
% k = 10;
% for i = 1 : 1 : (n - k)
%     sum_curvature = 0;
%     for j = i : 1 :(i + k - 1)
%         sum_curvature = sum_curvature + trajref(j, 5);
%     end
%     c = sum_curvature / k;
%     trajref(i, 5) = c;
%     trajref(i, 4) = 1 / c;
% end

% 2. 从前向后曲率以一定大小逐渐生成
% inc_curvature = 0.01;
% for i = 2 : 1 : n
%     c_prev = trajref(i-1, 5);
%     c_curr = trajref(i, 5);
%     
%     c_max = c_prev + inc_curvature;
%     c_min = c_prev - inc_curvature;
%     
%     c_curr = max(c_curr, c_min);
%     c_curr = min(c_curr, c_max);
%     
%     trajref(i, 5) = c_curr;
%     trajref(i, 4) = 1 / c_curr;
% end

% 3. 从后向前曲率以一定大小逐渐生成
inc_curvature = 0.02;
for i = (n-1) : -1 : 1
    c_next = trajref(i+1, 5);
    c_curr = trajref(i, 5);
    
    c_max = c_next + inc_curvature;
    c_min = c_next - inc_curvature;
    
    c_curr = max(c_curr, c_min);
    c_curr = min(c_curr, c_max);
    
    trajref(i, 5) = c_curr;
    trajref(i, 4) = 1 / c_curr;
end

plot(trajref(:, 5), 'b'); hold on
grid minor

%% 约束每个点的曲率半径不超过其最大值
for i = 1 : 1 : n
    trajref(i, 4) = limit_radius(trajref(i, 4), default_r);
    trajref(i, 5) = 1 / trajref(i, 4);
end

