function heading_angle = calc_heading_angle(point1, point2)
% 根据两点位置计算航向角

% 输出:
% heading_angle : 航向角，与横坐标轴的角度，逆时针0~2*pi, rad

% 输入:
% point1        : 位置坐标 [x, y]
% point2        : 位置坐标 [x, y]

delta_y = point2(2) - point1(2);
delta_x = point2(1) - point1(1);

heading_angle = mod(atan2(delta_y, delta_x), 2*pi);

