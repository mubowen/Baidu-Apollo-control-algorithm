function [nearest_point, index] = calc_nearest_point(veh_pose, trajref)
% 求车辆当前位置在期望路径上的最近点
% 输出:
% nearest_point: 最近点坐标[x, y]
% index        : 最近点的index

% 输入:
% veh_pose     : 车辆当前位姿[x, y, theta]
% trajref      : 期望路径[X, Y, Theta, Radius]

n = length(trajref);
dist = zeros(n, 1);

for i = 1:n
    dist(i, :) = norm(trajref(i, 1:2) - veh_pose(1:2));
end

[~, index] = min(dist); 

index = index(1);
nearest_point = trajref(index, 1:2);

