function draw_veh_pose(veh_pose, veh_params)
% 画出车辆的位姿状态? veh_pose = [x, y, theta]

% 输入:
% veh_pose   : 车辆的位姿[x, y, theta]
% veh_params : 车辆参数

x0 = veh_pose(1);
y0 = veh_pose(2);
theta = veh_pose(3);

% 车辆位置
hold on
plot(x0, y0, 'b.', 'markersize', veh_params.vehicle_size);
grid minor

% 车辆航向
x1 = x0 + veh_params.vehicle_length * cos(theta);
y1 = y0 + veh_params.vehicle_length * sin(theta);
draw_arrow([x0, y0], [x1, y1]);

