function log = log_init(time_step, simulation_stop_time)
% log初始化

% 输出:
% log       : 需要记录的log信息

% 输入:
% time_step : 仿真时间步长
% simulation_stop_time : 仿真停止时间

log_size = simulation_stop_time / time_step;    %log大小

log.time = zeros(log_size, 1);      %仿真时间log
log.steer_cmd = zeros(log_size, 1); %期望前轮偏角log
log.veh_pose = zeros(log_size, 3);  %车辆位姿log
log.dist = zeros(log_size, 1);      %车辆里程log

