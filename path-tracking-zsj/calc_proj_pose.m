          function proj_pose = calc_proj_pose(p0, p1, p2)
% 求点p0在点p1和点p2组成直线上的投影点坐标及航向

% 输出:
% proj_point : 投影点位姿[x, y, theta]

% 输入:
% p0    : point0点位姿[x0, y0, theta0]
% p1    : point0点位姿[x1, y1, theta1]
% p2    : point0点位姿[x2, y2, theta2]

tol = 0.0001;
proj_pose = [0, 0, 0];

if abs(p2(1) - p1(1)) < tol
    % p1和p2直线的斜率无穷大
    x = p1(1);     %投影点x坐标为p1的x坐标
    y = p0(2);     %投影点y坐标为p0的y坐标

elseif abs(p2(2) - p1(2)) < tol
    % p1和p2直线的斜率无穷大
    x = p0(1);     %投影点x坐标为p1的x坐标
    y = p1(2);     %投影点y坐标为p0的y坐标
    
else
    k1 = (p2(2) - p1(2)) / (p2(1) - p1(1)); %p1和p2的直线斜率
    k2 = -1 / k1;   %p0和投影点的直线斜率? two perpendicular line mutiply is -1
    
    x = (p0(2) - p1(2) + k1 * p1(1) - k2 * p0(1)) / (k1 - k2);
    y = p0(2) + k2 * (x - p0(1));
end

proj_pose(1) = x;
proj_pose(2) = y;

dist = norm(p2(1:2) - p1(1:2));         %点p1到点p2的距离
dist2 = norm(p2(1:2) - proj_pose(1:2)); %投影点到点p2的距离

ratio = dist2 / dist;
theta = ratio * p1(3) + (1 - ratio) * p2(3);

proj_pose(3) = theta;


