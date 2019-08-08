function preview_dist = calc_preview_dist(params, velocity)
% ¼ÆËãÔ¤Ãé¾àÀë
% Êä³ö:
% preview_dist  : Ô¤Ãé¾àÀë, m

% ÊäÈë:
% params        : Ô¤Ãé¾àÀë²ÎÊı
% velcity       : ³µËÙ, m/s

preview_dist = params.k * velocity;

preview_dist = max(preview_dist, params.min_preview_dist);
preview_dist = min(preview_dist, params.max_preview_dist);


