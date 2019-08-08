function steer_angle = pure_pursuit(preview_point, wheel_base)
% pure pursuit计算期望前轮偏角

% 杈撳嚭:
% steer_angle       : 前轮转角, rad

% 杈撳叆: 
% preview_point     : 预瞄点坐标[x, y]
% wheel_base        : 轴距, m

preview_dist = norm(preview_point);     %预瞄距离, m
alpha = asin(-preview_point(1) / preview_dist);     %预瞄点与当前位置的角度
steer_angle = atan(2 * wheel_base * sin(alpha) / preview_dist); %前轮偏角

