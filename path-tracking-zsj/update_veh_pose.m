function veh_pose = update_veh_pose(origin_pose, steer_angle,...
    veh_params, time_step)
% 更新车辆的位姿状态 vehicle_pose = [x, y, theta]
% x     : 横坐标, m
% y     : 纵坐标, m
% theta : 航向角，与横坐标的角度，逆时针[0, 2*pi], rad

% 输入:
% origin_pose   : 初始车辆位姿状态
% steer_angle   : 前轮偏角, rad
% veh_params    : 车辆参数
% time_step     : 仿真步长, s

x0 = origin_pose(1);
y0 = origin_pose(2);
theta0 = origin_pose(3);

delta_dist = veh_params.velocity * time_step;  %车辆在单位仿真步长走过的距离

tol = 0.0001;   %判断直行还是转弯的前轮偏角阈值

if abs(steer_angle) > tol
    % 转弯
    radius = veh_params.wheel_base / tan(steer_angle);  %转弯半径
    center_x = x0 - radius * sin(theta0);
    center_y = y0 + radius * cos(theta0);
    
    delta_theta = delta_dist / radius;
    theta = theta0 + delta_theta;
    x = center_x + radius * sin(theta);
    y = center_y - radius * cos(theta);
else
    % 直行
    x = x0 + delta_dist * cos(theta0);
    y = y0 + delta_dist * sin(theta0);
    theta = theta0;
end

theta = mod(theta, 2*pi); %theta的范围为0~2*pi
veh_pose = [x, y, theta];


