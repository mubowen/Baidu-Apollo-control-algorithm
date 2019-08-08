function steer_cmd = ALG_LQR_model3(veh_pose, trajref, lqr_params,...
    veh_params, steer_state, time_step)
% 用LQR算法计算期望前轮偏角
% 采用的车辆模型model3: [delta_x, delta_y, delta_theta]

% 输出:
% steer_cmd     : 期望前轮偏角, rad

% 输入:
% veh_pose      : 车辆当前位姿[x, y, theta]
% trajref       : 期望路径[X, Y, Theta, Radius]
% lqr_params    : LQR参数
% veh_params    : 车辆参数
% steer_state   : 当前前轮偏角, rad
% time_step     : 仿真时间步长, s

% 1. 计算车辆当前位置在期望路径上的投影点位姿
[~, index] = calc_nearest_point(veh_pose, trajref);
ref_pose = calc_proj_pose(veh_pose(1:2), trajref(index, 1:3),...
    trajref(index + 1, 1:3));

% 2. 计算参考点的前轮偏角前馈控制量
ref_index = index + lqr_params.ref_index;
ref_radius = trajref(ref_index, 4);
steer_feedforward = calc_steer_feedforward(ref_radius,...
    veh_params.wheel_base);

% 3. 用LQR计算前轮偏角反馈控制量
delta_x = (veh_pose - ref_pose)';
steer_feedbackward = calc_lqr_feedbackward(trajref, delta_x, ...
    lqr_params, index, veh_params, steer_feedforward);

% 4. 计算期望前轮偏角
steer_cmd = steer_feedforward + steer_feedbackward;

% 5. 限制期望前轮偏角
steer_cmd = limit_steer_by_angular_vel(steer_cmd, steer_state,...
    veh_params.max_angular_vel, time_step);

steer_cmd = limit_steer_angle(steer_cmd, veh_params.max_steer_angle);


