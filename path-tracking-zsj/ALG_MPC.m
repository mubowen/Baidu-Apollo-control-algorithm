function [steer_cmd,acc]= ALG_MPC(veh_pose, trajref,...
    mpc_params, veh_params, steer_state, time_step)
% 用MPC算法计算期望前轮偏角

% 输出:
% steer_cmd     : 期望前轮偏角, rad
% acc           ：加速度补足，m/s^2

% 输入:
% veh_pose          : 车辆当前位姿[x, y, theta]
% trajref           : 期望路径[X, Y, Theta, Radius]
% mpc_params        : mpc参数
% veh_params        : 车辆参数
% steer_state       : 当前前轮偏角, rad
% time_step         : 仿真时间步长, s

% 1. 计算车辆当前位置在期望路径上的投影点位姿
[~, index] = calc_nearest_point(veh_pose, trajref);
ref_pose = calc_proj_pose(veh_pose(1:2), trajref(index, 1:3),...
    trajref(index + 1, 1:3));

delta_x = (veh_pose - ref_pose)';

% 2. 计算命令偏角
[steer_command,acc,steer_feedforward] = calc_mpc(trajref, delta_x, veh_pose, ref_pose, ...
    mpc_params, index, veh_params);


% 3. 计算期望前轮偏角
steer_cmd = steer_feedforward + steer_command;

% 4. 限制期望前轮偏角
steer_cmd = limit_steer_by_angular_vel(steer_cmd, steer_state,...
    veh_params.max_angular_vel, time_step);

steer_cmd = limit_steer_angle(steer_cmd, veh_params.max_steer_angle);


