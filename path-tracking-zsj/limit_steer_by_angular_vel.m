function steer_angle = limit_steer_by_angular_vel(expect_steer_angle,...
    current_steer_angle, max_angular_vel, time_step)
% 限制前轮偏角不超过其最大角速度的限制?

% 杈撳嚭:
% steer_angle  : 满足角速度约束带期望前轮偏角?

% 杈撳叆:
% expect_steer_angle  : 当前周期期望的前轮偏角
% current_steer_angle : 当前时刻前轮偏角的实际值?
% max_angular_vel     : 最大前轮偏角角速度
% time_step           : 时间步长

steer_angle = expect_steer_angle;

max_steer_angle = current_steer_angle + max_angular_vel * time_step;
min_steer_angle = current_steer_angle - max_angular_vel * time_step;

steer_angle = min(steer_angle, max_steer_angle);
steer_angle = max(steer_angle, min_steer_angle);
