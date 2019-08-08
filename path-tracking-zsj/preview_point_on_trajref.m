function preview_point = preview_point_on_trajref(veh_pose,...
    preview_dist, trajref, index)
% 计算在期望路径上的预瞄点

% 输出:
% preview_point : 预瞄点坐标[x, y]

% 输入:
% veh_pose      : 车辆当前位姿[x, y, theta]
% preview_dist  : 预瞄距离, m
% trajref       : 期望路径
% index         : 在期望路径上开始搜索的index

n = length(trajref);

for i = index : 1 : n
    % 依次计算trajref上每点到车辆当前位置的距离?
    tmp_dist = norm(trajref(i, 1:2) - veh_pose(1:2));
    
    if tmp_dist > preview_dist
        preview_index = i;
        break;
    end
end

preview_point = trajref(preview_index, 1:2);


