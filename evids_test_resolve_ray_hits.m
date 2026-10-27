function [absoluteZ, mu, detected, roadZ] = evids_test_resolve_ray_hits(hits, props, roadMu, initialZ)
% hits: N×4 [world_x,world_y,world_z,semantic_label]，竖直向下射线。
% props: 已登记actor的包围盒与响应参数快照；不按固定语义或中心半径识别。
% 包围盒采用XY有向矩形和Z范围；调用方只允许水平、绕Z轴旋转的道具。
mu = roadMu;
detected = 0;
roadZ = NaN;
propIndex = 0;
propZ = NaN;
if ~isempty(hits)
    hits = hits(all(isfinite(hits), 2), :);
    % 不依赖服务器返回顺序：竖直向下时先处理较高的命中。
    [~, order] = sort(hits(:, 3), 'descend');
    hits = hits(order, :);
end
for h = 1:size(hits, 1)
    hit = hits(h, :);
    if hit(4) == 10 % 保留现有逻辑：跳过车辆类命中。
        continue;
    end
    if any(hit(4) == [6, 7, 8])
        % 道路语义优先，避免道具底面与道路重合时污染初始道路基准。
        roadZ = hit(3);
        break;
    end
    matched = 0;
    for p = 1:numel(props)
        delta = hit(1:3) - props(p).Center;
        c = cosd(props(p).Yaw);
        s = sind(props(p).Yaw);
        local = [c * delta(1) + s * delta(2), ...
            -s * delta(1) + c * delta(2), delta(3)];
        outsideXY = max(abs(local(1:2)) - props(p).Extent(1:2), 0);
        % 增加Z范围检查，避免把同一XY下方的道路误认为道具表面。
        insideZ = abs(local(3)) <= props(p).Extent(3) + props(p).Tolerance + 1e-6;
        if norm(outsideXY) <= props(p).Tolerance + 1e-6 && insideZ
            matched = p;
            break;
        end
    end
    if matched > 0
        if propIndex == 0
            propIndex = matched;
            propZ = hit(3);
        end
        % 非道路道具无需固定Label=3，继续寻找其下方的道路基准。
        continue;
    end
end
if isfinite(roadZ)
    absoluteZ = roadZ;
elseif isfinite(initialZ)
    absoluteZ = initialZ;
else
    absoluteZ = 0;
end
if propIndex > 0
    p = props(propIndex);
    mu = p.Mu;
    detected = 1;
    if p.UseSurfaceHeight
        if isfinite(initialZ)
            referenceZ = initialZ;
        elseif isfinite(roadZ)
            referenceZ = roadZ;
        else
            referenceZ = propZ;
        end
        absoluteZ = referenceZ + (propZ - referenceZ) * p.ZScale;
    end
end
end
