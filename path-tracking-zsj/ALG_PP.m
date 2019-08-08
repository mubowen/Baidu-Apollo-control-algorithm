function steer_cmd = ALG_PP(veh_pose, trajref, pp_params,...
    veh_params, steer_state, time_step)
% 用PP算法计算期望前轮偏角

% 输出:
% steer_cmd         : 期望前轮偏角, rad

% 输入:
% veh_pose          : 车辆当前位姿[x, y, theta]
% trajref           : 期望路径[X, Y, Theta, Radius]
% pp_params         : pp参数
% veh_params        : 车辆参数
% steer_state       : 当前前轮偏角, rad
% time_step         : 仿真时间步长, s

% 1. 计算车辆在当前位置再期望路径上的最近点
[~, index] = calc_nearest_point(veh_pose, trajref);

% 2. 计算预瞄距离
preview_dist = calc_preview_dist(pp_params, veh_params.velocity);

% 3. 计算预瞄点位置? 
% 3.1. 计算在期望路径上的预瞄点
preview_point_global = preview_point_on_trajref(veh_pose,...
    preview_dist, trajref, index);

% 3.2. 将预瞄点从全局坐标系转到车辆局部坐标系
base_local = veh_pose;  %车辆位姿作为局部坐标系原点
base_local(3) = veh_pose(3) - pi/2;  %车的航向为Y轴
preview_point_local = cvt_global_to_local(preview_point_global,...
    base_local);

% 4. 计算期望前轮偏角
steer_cmd = pure_pursuit(preview_point_local, veh_params.wheel_base);

% 5. 限制期望前轮偏角?
steer_cmd = limit_steer_by_angular_vel(steer_cmd, steer_state,...
    veh_params.max_angular_vel, time_step);

steer_cmd = limit_steer_angle(steer_cmd, veh_params.max_steer_angle);

