function definitions = evids_test_props(world)
% 仅用于Town04回归：恢复原程序测试过的蓝图和位置，不作为发布默认场景。
% 在MATLAB中addpath('examples')，将System块的PropConfigurationFunction
% 设为"evids_test_props"。同一场景同时验证两种高度响应。
if ~contains(string(world.get_map().name), "Town04")
    error('V22:TestMap', 'This regression configuration requires Town04.');
end
definitions = struct('Blueprint', {}, 'Transforms', {}, 'Mu', {}, ...
    'UseSurfaceHeight', {}, 'ZScale', {}, 'Tolerance', {});
definitions(1) = struct('Blueprint', "static.prop.dirtdebris01", ...
    'Transforms', [9.5, 40, 0.05, 0], 'Mu', 1, ...
    'UseSurfaceHeight', true, 'ZScale', 1, 'Tolerance', 0);
% 原低附着铺设网格：中心X=5.4、宽3.5、Y=-15至-100、间距1 m。
% 按原floor计数：85行×3列，原点X=3.65/4.65/5.65，Y=-15...-99。
poses = zeros(85 * 3, 4);
row = 0;
map = world.get_map();
for iy = 0:84
    for ix = 0:2
        row = row + 1;
        x = (5.4 - 3.5 / 2) + ix;
        y = -15 - iy;
        wp = map.get_waypoint(py.carla.Location(pyargs('x', x, 'y', y, 'z', 1)));
        if isequal(wp, py.None)
            error('V22:TestGround', 'No waypoint for the known test position.');
        end
        poses(row, :) = [x, y, double(wp.transform.location.z) + 0.02, ...
            double(wp.transform.rotation.yaw)];
    end
end
definitions(2) = struct('Blueprint', "static.prop.dirtdebris01", ...
    'Transforms', poses, 'Mu', 0.3, 'UseSurfaceHeight', false, ...
    'ZScale', 1, 'Tolerance', 0);
end
