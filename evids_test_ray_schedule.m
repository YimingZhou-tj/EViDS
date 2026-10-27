function [isNear, interval, distance, speed] = evids_test_ray_schedule( ...
    wheels, props, currentTime, previousTime, previousWheels, wasNear, ...
    nearDt, farDt, margin, hysteresis, approachSpeed)
% Only schedules ray queries. Mu/Z always come from actual ray hits.
% Static geometry is cached by setup; no Python/actor calls occur here.
speed = approachSpeed;
elapsed = currentTime - previousTime;
if isfinite(elapsed) && elapsed > 1e-9
    measured = max(sqrt(sum((wheels - previousWheels).^2, 2))) / elapsed;
    speed = max(speed, measured);
end
distance = inf;
if ~isempty(props)
    centers = vertcat(props.Center);
    extents = vertcat(props.Extent);
    yaw = reshape([props.Yaw], [], 1);
    tolerance = reshape([props.Tolerance], [], 1);
    c = cosd(yaw); s = sind(yaw);
    for wheel = 1:size(wheels, 1)
        dx = wheels(wheel, 1) - centers(:, 1);
        dy = wheels(wheel, 2) - centers(:, 2);
        localX = c .* dx + s .* dy;
        localY = -s .* dx + c .* dy;
        gapX = max(abs(localX) - extents(:, 1) - tolerance, 0);
        gapY = max(abs(localY) - extents(:, 2) - tolerance, 0);
        distance = min(distance, min(hypot(gapX, gapY)));
    end
end
% Cover travel during one far interval and one near interval, plus margin.
enterDistance = margin + speed * (farDt + nearDt);
exitDistance = enterDistance + hysteresis;
isNear = distance <= enterDistance || (wasNear && distance <= exitDistance);
interval = farDt;
if isNear
    interval = nearDt;
end
end
