function angle_norm = angle_normalization(angle_origin)
% 将角度的取值范围标准化到[-pi, pi]

% 输出:
% angle_norm    : 标准化后的角度值,[-pi, pi]

% 输入:
% angle_origin  : 原始角度值，[-2*pi, 2*pi]

if angle_origin > pi
    angle_norm = angle_origin - 2 * pi;
    
elseif angle_origin < -pi
    angle_norm = angle_origin + 2 * pi;
    
else
    angle_norm = angle_origin;
end

