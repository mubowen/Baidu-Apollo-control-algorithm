function steer_angle = limit_steer_angle(steer_angle, max_steer_angle)
% 限制前轮偏角不超过其最大值

% 输出:
% steer_angle     : 前轮偏角

% 输入:
% steer_angle     : 前轮偏角
% max_steer_angle : 最大前轮偏角

steer_angle = min(steer_angle, max_steer_angle);
steer_angle = max(steer_angle, -max_steer_angle);
