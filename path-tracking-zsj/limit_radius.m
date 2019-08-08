function radius = limit_radius(radius, max_radius)
% 限制半径值不超过其最大值

% 杈撳嚭:
% radius     : 半径大小, m

% 杈撳叆:
% radius     : 半径大小, m
% max_radius : 最大半径, m

radius = min(radius, max_radius);
radius = max(radius, -max_radius);