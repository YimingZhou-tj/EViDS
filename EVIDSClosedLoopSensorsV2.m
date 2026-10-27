classdef EVIDSClosedLoopSensorsV2 < matlab.System
    % CARLA/CarSim多速率接口：可配置传感器路由与统一道具射线检测。
    % 输出端口始终固定：8个核心端口 + 6个传感器端口。
    % SensorDataRoute="simulink"时才把已启用的传感器数据复制到MATLAB；
    % SensorDataRoute="python"时真实数据仅保留在Python回调管理器，传感器端口输出预分配零值。
    
    properties
        
        MapName = "Town04";
        
        % Town03初始0.1 s的坐标偏移；正常出生位姿来自CarSim映射输入。
        SpawnX =  0;
        SpawnY = 0;
        SpawnZ = 0.0;
               
        VehicleBlueprintName = "vehicle.tesla.model3";
        % SimulinkDt应与模型真实基础步长一致，不用它控制射线频率。
        SimulinkDt = 0.001;
        RaycastDt = 0.01;   % 四轮射线更新周期10 ms，对应100 Hz仿真时间频率
        FarRaycastDt = 0.1; % 远离登记道具：10 Hz；RaycastDt仍为接近时100 Hz
        RayNearMargin = 3; % 包围盒外额外提前距离，单位m
        RayExitHysteresis = 2; % 离开时额外距离，避免边界反复切换
        RayApproachSpeed = 80; % 预判用速度下限m/s；实测更快时采用实测轮位速度
        CarlaDt = 0.05;    % CARLA world.tick、车辆视觉位姿和相机/渲染更新周期
        
        % 普通道路默认摩擦系数；道具响应在configure_v22_props.m中填写。
        RoadFriction = 1.0;

        FilterAlpha = 1;
        
        % --- 数据保存（仅 MATLAB/Simulink 路由使用） ---
        EnableSave = false;
        DatasetRoot = ""; % 空值时自动使用本运行包内outputs目录
        SavePeriod = 0.05;
        CameraHeight = 1.2;    
        CameraX = 1.5;         
        CameraYOffset = 0.5;        
        CameraFOV = 90;     

        % 车道线采样参数
        LaneSamplePoints = 50;
        LaneSampleDist = 2.0;
        ViewMode = 1;

        % 道具蓝图、位置、摩擦和高度响应统一在configure_v22_props.m填写。
        % 默认列表为空，不生成或自动清理场景中的其他道具。
    end

    properties(Nontunable)
        % 连接地址在仿真开始前配置，运行中不能热切换。
        CarlaHost = 'localhost';
        CarlaPort = 2000;
        % 可指定独立的测试配置函数；该函数接收CARLA world并返回道具配置。
        PropConfigurationFunction = 'evids_test_props';
        % 传感器数据路由：
        % "simulink"：Python回调后，将启用的传感器数据传到固定Simulink端口。
        % "python"  ：数据只保留在Python管理器中，不进行Python->MATLAB大数组复制；
        %             固定传感器端口仍存在并输出零值。请在仿真开始前切换路由。
        %SensorDataRoute = "simulink";
        SensorDataRoute = "python";

        EnableSemantic = true;
        EnableDepth = true;
        EnableStereoRGB = true;
        EnableLidar = false;

        % Simulink LiDAR端口采用固定上限，另输出有效点数。
        LidarMaxPoints = 30000;

    end

    properties(Access = private)
        client, world, spectator, ghost_vehicle
        registered_props = {}
        cached_prop_geometry = struct('Center', {}, 'Extent', {}, 'Yaw', {}, 'Mu', {}, ...
            'UseSurfaceHeight', {}, 'ZScale', {}, 'Tolerance', {});
        ray_near_region = false;
        ray_previous_time = nan;
        ray_previous_points = zeros(4, 2);
        original_world_settings = []
        ss_rgb, depth, module_ss_rgb, module_depth, map
        
        lidar, module_lidar
        lidar_save_subdir
        
        width = 752; height = 480;
        cache_SS, cache_DepthRaw
        cache_LeftImg, cache_RightImg
        cache_LidarPoints, cache_LidarCount
        
        cache_TireZ = zeros(4, 1);
        cache_TireMu = ones(4, 1);
        cache_TargetDetect = zeros(4, 1);
        
        last_visual_update_time = -inf;
        last_ray_update_time = -inf;
        is_first_step = true;
        is_first_ray_step = true;
        
        InitialGroundZ = nan(4, 1);
        is_ground_initialized = false;
        prev_Filtered_Z = zeros(4, 1);
        is_filter_initialized = false;
        
        % 相机与存储
        cam_left, cam_right
        module_left, module_right
        current_save_dir = ""
        frame_counter = int64(0)
        timestamp_fid = []
        last_saved_index = int64(-1);
        
        cache_Centerline, cache_LeftBound, cache_RightBound
        cache_PoseCarla
        
        lane_update_count = 0;
    end

    methods
        function obj = EVIDSClosedLoopSensorsV2(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods(Access = protected)
        function setupImpl(obj, vehiclePose, ~, ~, clock_time)
            % Prepare CARLA, spawn ghost vehicle, obstacles, sensors, and dataset dirs.
            coder.extrinsic('py', 'fprintf', 'fopen', 'fclose');
            
            evids_test_initialize_python();

            % DatasetRoot是公开配置属性，System运行期间只能读取，不能修改。
            dataset_root = string(obj.DatasetRoot);

            if strlength(dataset_root) == 0
                dataset_root = string(fullfile( ...
                    fileparts(mfilename('fullpath')), 'outputs'));
            end

            obj.validateConfiguration();

            % Connect to CARLA
            obj.client = py.carla.Client(char(obj.CarlaHost), int32(obj.CarlaPort));
            obj.client.set_timeout(60.0);
            
            curr_map = string(obj.client.get_world().get_map().name);
            if ~contains(curr_map, obj.MapName); obj.client.load_world(obj.MapName); end
            
            obj.world = obj.client.get_world();
            obj.map = obj.world.get_map();
            obj.spectator = obj.world.get_spectator();
            obj.registered_props = {};

            definitions = feval(char(obj.PropConfigurationFunction), obj.world);
            evids_test_validate_prop_definitions(definitions);
            obj.original_world_settings = obj.world.get_settings();
            try
            settings = obj.world.get_settings();
            settings.synchronous_mode = true;
            settings.fixed_delta_seconds = obj.CarlaDt;
            obj.world.apply_settings(settings);
            
            % 使用CarSim映射后的初始位姿生成，不随机选择出生点、不删除其他车辆。
            bp_lib = obj.world.get_blueprint_library();
            car_bp = bp_lib.find(obj.VehicleBlueprintName);
            pose = double(vehiclePose(:));
            if obj.MapName == "Town03" && clock_time <= 0.1
                pose(1:3) = pose(1:3) + [obj.SpawnX; obj.SpawnY; obj.SpawnZ];
            end
            initialTransform = py.carla.Transform( ...
                py.carla.Location(pyargs('x', pose(1), 'y', pose(2), 'z', pose(3))), ...
                py.carla.Rotation(pyargs('pitch', pose(4), 'roll', pose(5), 'yaw', pose(6))));
            diagnosticsFile = fullfile(fileparts(mfilename('fullpath')), 'last_spawn_diagnostics.json');
            if evalin('base', 'exist(''EVIDS_TEST_RUN_FOLDER'',''var'')')
                diagnosticsFile = fullfile(evalin('base', 'EVIDS_TEST_RUN_FOLDER'), 'spawn_diagnostics.json');
            end
            obj.ghost_vehicle = evids_test_spawn_vehicle_v2( ...
                obj.world, car_bp, initialTransform, diagnosticsFile);
            
            % 所有配置道具统一生成并登记；不再区分土堆/低附着模式。
            obj.spawnRegisteredProps(definitions);

            % Sensors: semantic & depth attached at recommended center transform 
            att = pyargs('attach_to', obj.ghost_vehicle);
            rel_tf_driver = py.carla.Transform(py.carla.Location(pyargs('x',0.0, 'y',-0.4, 'z',1.2)), py.carla.Rotation(pyargs('pitch',0.0)));
            
            obj.ss_rgb = []; obj.module_ss_rgb = [];
            if obj.EnableSemantic
                bp_ss = bp_lib.find('sensor.camera.semantic_segmentation');
                bp_ss.set_attribute('image_size_x', num2str(obj.width));
                bp_ss.set_attribute('image_size_y', num2str(obj.height));
                obj.ss_rgb = obj.world.spawn_actor(bp_ss, rel_tf_driver, att);
                obj.module_ss_rgb = py.evids_test_sensor_manager.SensorManager(int32(obj.width), int32(obj.height));
                obj.ss_rgb.listen(obj.module_ss_rgb);
            end

            obj.depth = []; obj.module_depth = [];
            if obj.EnableDepth
                bp_depth = bp_lib.find('sensor.camera.depth');
                bp_depth.set_attribute('image_size_x', num2str(obj.width));
                bp_depth.set_attribute('image_size_y', num2str(obj.height));
                obj.depth = obj.world.spawn_actor(bp_depth, rel_tf_driver, att);
                obj.module_depth = py.evids_test_sensor_manager.DepthSensorManager(int32(obj.width), int32(obj.height));
                obj.depth.listen(obj.module_depth);
            end
            
            % --- 左右 RGB 相机---
            obj.cam_left = []; obj.cam_right = []; obj.module_left = []; obj.module_right = [];
            if obj.EnableStereoRGB
                try
                    loc_L = py.carla.Location(pyargs('x',obj.CameraX, 'y', -obj.CameraYOffset, 'z',obj.CameraHeight));
                    rot = py.carla.Rotation();
                    tf_L = py.carla.Transform(loc_L, rot);

                % Right Cam
                loc_R = py.carla.Location(pyargs('x',obj.CameraX, 'y', obj.CameraYOffset, 'z',obj.CameraHeight));
                tf_R = py.carla.Transform(loc_R, rot);

                att = pyargs('attach_to', obj.ghost_vehicle);

                bp_rgb = bp_lib.find('sensor.camera.rgb');
                bp_rgb.set_attribute('image_size_x', num2str(obj.width));
                bp_rgb.set_attribute('image_size_y', num2str(obj.height));
                bp_rgb.set_attribute('fov', num2str(obj.CameraFOV));

                obj.cam_left = obj.world.spawn_actor(bp_rgb, tf_L, att);
                obj.module_left = py.evids_test_sensor_manager.SensorManager(int32(obj.width), int32(obj.height));
                obj.cam_left.listen(obj.module_left);

                    obj.cam_right = obj.world.spawn_actor(bp_rgb, tf_R, att);
                    obj.module_right = py.evids_test_sensor_manager.SensorManager(int32(obj.width), int32(obj.height));
                    obj.cam_right.listen(obj.module_right);
                catch ME
                    warning('Failed to create left/right rgb cameras: %s', char(ME.message));
                    obj.cam_left = []; obj.cam_right = []; obj.module_left = []; obj.module_right = [];
                end
            end
            
            % --- LiDAR（ray_cast）---
            obj.lidar = []; obj.module_lidar = [];
            if obj.EnableLidar
                try
                    bp_lidar = bp_lib.find('sensor.lidar.ray_cast');
                % Lidar属性
                bp_lidar.set_attribute('range', '100.0'); % 最大测距 m
                bp_lidar.set_attribute('channels', '64'); % 激光束数量
                bp_lidar.set_attribute('points_per_second', '500000'); % 每秒点数
                bp_lidar.set_attribute('upper_fov', '15.0');
                bp_lidar.set_attribute('lower_fov', '-25.0');
                % 每秒旋转轮数1/0.05
                bp_lidar.set_attribute('rotation_frequency', num2str(1.0 / double(obj.CarlaDt)));

                % 放置 LiDAR（相对于车辆）
                loc_LIDAR = py.carla.Location(pyargs('x', -0.5, 'y', 0.0, 'z', 1.8));
                rot_LIDAR = py.carla.Rotation();
                tf_LIDAR  = py.carla.Transform(loc_LIDAR, rot_LIDAR);

                att = pyargs('attach_to', obj.ghost_vehicle);
                obj.lidar = obj.world.spawn_actor(bp_lidar, tf_LIDAR, att);

                % Python 端管理器：确保 sensor_manager.py 中实现了 LidarSensorManager
                obj.module_lidar = py.evids_test_sensor_manager.LidarSensorManager();

                % 启动监听（回调会把点云存到 Python 管理器）
                    obj.lidar.listen(obj.module_lidar);
                catch ME
                    warning('Failed to create lidar: %s', char(ME.message));
                    obj.lidar = [];
                    obj.module_lidar = [];
                end
            end

            
            % MATLAB侧保存只在Simulink路由启用；Python路由由回调管理器接收数据。
            if obj.EnableSave && obj.isSimulinkSensorRoute()
                % 基于时间创建数据集子目录（与 v2 保持类似）
                try
                    folder_name = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
                    obj.current_save_dir = fullfile(char(dataset_root), folder_name);
                    path_l = fullfile(obj.current_save_dir, 'image_0');
                    path_r = fullfile(obj.current_save_dir, 'image_1');
                    if ~exist(obj.current_save_dir, 'dir'); mkdir(obj.current_save_dir); end
                    if ~exist(path_l, 'dir'); mkdir(path_l); end
                    if ~exist(path_r, 'dir'); mkdir(path_r); end
                    
                    % ---  LiDAR 子目录 ---
                    path_lidar = fullfile(obj.current_save_dir, 'lidar');
                    if ~exist(path_lidar, 'dir'); mkdir(path_lidar); end
                    obj.lidar_save_subdir = path_lidar;

                    
                    % 打开 timestamps.txt
                    txt_path = fullfile(obj.current_save_dir, 'timestamps.txt');
                    obj.timestamp_fid = fopen(txt_path, 'w');
                    if obj.timestamp_fid == -1
                        warning('Could not open timestamps file for writing: %s', txt_path);
                        obj.timestamp_fid = [];
                    end
                catch ME
                    warning('Failed to prepare save directories: %s', char(ME.message));
                    obj.current_save_dir = "";
                    obj.timestamp_fid = [];
                end
            end
            
            % 初始化缓存
            obj.cache_SS = zeros(obj.height, obj.width, 'uint8');
            obj.cache_DepthRaw = zeros(obj.height, obj.width, 'double');
            obj.cache_LeftImg = zeros(obj.height, obj.width, 3, 'uint8');
            obj.cache_RightImg = zeros(obj.height, obj.width, 3, 'uint8');
            obj.cache_LidarPoints = zeros(double(obj.LidarMaxPoints), 4, 'single');
            obj.cache_LidarCount = 0;
            obj.InitialGroundZ = nan(4, 1);
            obj.cache_TireZ = zeros(4, 1);
            obj.cache_TireMu = ones(4, 1) * obj.RoadFriction;
            obj.cache_TargetDetect = zeros(4, 1);
            obj.is_ground_initialized = false;
            obj.is_filter_initialized = false;
            obj.prev_Filtered_Z = zeros(4, 1);
            
            obj.cache_Centerline = zeros(obj.LaneSamplePoints, 3);
            obj.cache_LeftBound  = zeros(obj.LaneSamplePoints, 3);
            obj.cache_RightBound = zeros(obj.LaneSamplePoints, 3);
            
            %位姿缓存
            obj.cache_PoseCarla = zeros(1, 3);
            
            obj.world.tick();
            % CARLA客户端在新actor收到首帧快照前get_transform会返回零位姿。
            % 必须先tick再缓存，否则道具都被误认为位于原点。
            obj.cached_prop_geometry = obj.getRegisteredPropGeometry();
            if evalin('base', 'exist(''EVIDS_TEST_RUN_FOLDER'',''var'')')
                snapshotFile = fullfile(evalin('base', 'EVIDS_TEST_RUN_FOLDER'), 'prop_geometry_snapshot.mat');
                propGeometry = obj.cached_prop_geometry; %#ok<NASGU>
                save(snapshotFile, 'propGeometry');
            end
            obj.last_visual_update_time = -inf;
            obj.last_ray_update_time = -inf;
            obj.ray_near_region = false;
            obj.ray_previous_time = nan;
            obj.ray_previous_points = zeros(4, 2);
            obj.frame_counter = int64(0);
            obj.last_saved_index = int64(-1);
            obj.is_first_step = true;
            obj.is_first_ray_step = true;
            obj.lane_update_count = 0;
            catch ME
                obj.releaseImpl();
                rethrow(ME);
            end
        end

        function varargout = stepImpl(obj, vehiclePose, tirePosFlatX, tirePosFlatY, clock_time)
            % Main step: same core logic as original V10 + left/right capture & optional save.
            coder.extrinsic('py', 'sprintf', 'fprintf', 'imwrite');

            tirePosX_CS = reshape(tirePosFlatX, [1, 4])';
            tirePosY_CS = reshape(tirePosFlatY, [1,4])';

            % 1. 多速率 CARLA 逻辑
            %
            % 路面反馈与视觉渲染必须解耦：
            %   - carla_veh_* / visual_Z 每个 SimulinkDt 都按最新 CarSim 位姿计算；
            %   - 四轮cast_ray按距离在RaycastDt/FarRaycastDt间切换，其余步保持缓存；
            %   - ghost vehicle、spectator、world.tick 和相机读取仅按 CarlaDt 执行。
            %
            % cast_ray 查询的是当前 CARLA 场景中的静态道路/道具几何，射线起点直接采用
            % 本次 Simulink 输入的四轮位置，因此不需要为了 1 kHz 路面反馈而让渲染也以 1 kHz tick。
            cond3 = obj.MapName == "Town03";
            if clock_time <= 0.1 && cond3
                carla_veh_x = vehiclePose(1) + obj.SpawnX;
                carla_veh_y = vehiclePose(2) + obj.SpawnY;
                visual_Z = vehiclePose(3) + obj.SpawnZ;
                current_veh_yaw = vehiclePose(6);
            else
                carla_veh_x = vehiclePose(1);
                carla_veh_y = vehiclePose(2);
                visual_Z = vehiclePose(3);
                current_veh_yaw = vehiclePose(6);
            end

            current_time = double(clock_time);
            do_visual_update = obj.is_first_step || ...
                (current_time - obj.last_visual_update_time >= obj.CarlaDt - 1e-8);
            if do_visual_update
                obj.is_first_step = false;
                obj.last_visual_update_time = current_time;
                
                
                
                loc = py.carla.Location(carla_veh_x, carla_veh_y, visual_Z);
                rot = py.carla.Rotation(pyargs('pitch', vehiclePose(4), 'yaw', current_veh_yaw, 'roll', vehiclePose(5)));
                obj.ghost_vehicle.set_transform(py.carla.Transform(loc, rot));
                
                % 视角: 根据 ViewMode 选择
                if obj.ViewMode == 1
                    % 驾驶员视角（和原来类似）
                    yaw_rad = deg2rad(current_veh_yaw);
                    off_x = 0.0; off_y = -0.4; off_z = 1.2;
                    cam_x = carla_veh_x + off_x * cos(yaw_rad) - off_y * sin(yaw_rad);
                    cam_y = carla_veh_y + off_x * sin(yaw_rad) + off_y * cos(yaw_rad);
                    cam_z = visual_Z + off_z;
                    obj.spectator.set_transform(py.carla.Transform( ...
                        py.carla.Location(cam_x, cam_y, cam_z), ...
                        py.carla.Rotation(pyargs('pitch', 0, 'yaw', double(current_veh_yaw), 'roll', 0))));
                else
                    % ViewMode == 2 : 全局俯视（以车辆为中心，固定高度俯视）
                    try
                        % --- 参数微调区 ---
                        % side_offset 控制观察距离：负数代表在车辆左侧观察，正数代表右侧。
                        % 为了看清全车，距离需要拉开（比如 6 到 8 米）。
                        side_offset = 7.0;

                        % back_offset 控制前后对齐：0 代表正对车身中央。
                        % 如果想稍微偏后一点观察，可以设为负数（如 -1.0）。
                        back_offset = -4;

                        % height_offset 控制相机高度：0.5m 大致与车轮轴心平齐。
                        % 这样能最清晰地看到轮胎与地面的切线。
                        height_offset = 6;
                        % ------------------

                        yaw_rad = deg2rad(current_veh_yaw);

                        % 1. 计算相机在世界坐标系中的绝对位置
                        cam_x = carla_veh_x + back_offset * cos(yaw_rad) - side_offset * sin(yaw_rad);
                        cam_y = carla_veh_y + back_offset * sin(yaw_rad) + side_offset * cos(yaw_rad);
                        cam_z = visual_Z + height_offset;

                        % 2. 计算相机的偏航角 (Yaw)
                        % 如果相机在左侧 (side_offset 为负)，需要向右看 (yaw + 90) 才能看到车。
                        % 如果相机在右侧 (side_offset 为正)，需要向左看 (yaw - 90)。
                        if side_offset < 0
                            cam_yaw = current_veh_yaw + 90.0;
                        else
                            cam_yaw = current_veh_yaw - 90.0;
                        end

                        % 3. 设置视角
                        % pitch = 0.0 代表水平观察。如果觉得太低，可以稍微设个负角 (如 -5.0) 俯视一点点。
                        obj.spectator.set_transform(py.carla.Transform( ...
                            py.carla.Location(cam_x, cam_y, cam_z), ...
                            py.carla.Rotation(pyargs('pitch', -35.0, 'yaw', double(cam_yaw), 'roll', 0.0))));

                    catch
                        % ignore spectator failures
                    end
                end
            end

            % 2. 静态道具包围盒只在setup读取；逐步距离检查仅使用MATLAB缓存。
            wheelPoints = [tirePosX_CS, tirePosY_CS];
            wasNear = obj.ray_near_region;
            [obj.ray_near_region, rayInterval] = evids_test_ray_schedule( ...
                wheelPoints, obj.cached_prop_geometry, current_time, ...
                obj.ray_previous_time, obj.ray_previous_points, wasNear, ...
                obj.RaycastDt, obj.FarRaycastDt, obj.RayNearMargin, ...
                obj.RayExitHysteresis, obj.RayApproachSpeed);
            obj.ray_previous_time = current_time;
            obj.ray_previous_points = wheelPoints;
            if ~obj.is_ground_initialized
                rayInterval = obj.RaycastDt; % 初始道路基准尚未建立时保持高频
            end
            do_ray_update = obj.is_first_ray_step || ...
                (obj.ray_near_region && ~wasNear) || ...
                (current_time - obj.last_ray_update_time >= rayInterval - 1e-8);
            if do_ray_update
                obj.is_first_ray_step = false;
                obj.last_ray_update_time = current_time;
                if ~cond3 || clock_time > 0.1
                    props = obj.cached_prop_geometry;
                    for i = 1:4
                        start_point = py.carla.Location(tirePosX_CS(i), tirePosY_CS(i), visual_Z + 3.0);
                        end_point = py.carla.Location(tirePosX_CS(i), tirePosY_CS(i), visual_Z - 30.0);
                        ray_list = cell(py.list(obj.world.cast_ray(start_point, end_point)));
                        hits = zeros(numel(ray_list), 4);
                        for k = 1:numel(ray_list)
                            hit = ray_list{k};
                            try label = double(py.int(hit.label)); catch; label = -1; end
                            hits(k, :) = [double(hit.location.x), double(hit.location.y), ...
                                double(hit.location.z), label];
                        end
                        [absoluteZ, mu, detected, roadZ] = evids_test_resolve_ray_hits( ...
                            hits, props, obj.RoadFriction, obj.InitialGroundZ(i));
                        % 基准仅来自道路命中，不使用道具表面；每轮建立一次。
                        if isnan(obj.InitialGroundZ(i)) && isfinite(roadZ)
                            obj.InitialGroundZ(i) = roadZ;
                        end
                        obj.cache_TireZ(i) = absoluteZ;
                        obj.cache_TireMu(i) = mu;
                        obj.cache_TargetDetect(i) = detected;
                    end
                    obj.is_ground_initialized = all(isfinite(obj.InitialGroundZ));
                end
            end

            % 3. 低频视觉/世界更新（仅按 CarlaDt 执行）
            if do_visual_update
                if obj.lane_update_count < 6
                    try
                        veh_loc = py.carla.Location(carla_veh_x, carla_veh_y, visual_Z);
                        wp = obj.map.get_waypoint(veh_loc);
                        obj.lane_update_count = obj.lane_update_count + 1;
                        for k = 1:obj.LaneSamplePoints
                            if wp ~= py.None
                                loc = wp.transform.location;
                                obj.cache_Centerline(k, :) = [double(loc.x), double(loc.y), double(loc.z)];
                                lw = double(wp.lane_width);
                                rv = wp.transform.get_right_vector();
                                obj.cache_LeftBound(k, :)  = [double(loc.x - rv.x*lw/2), double(loc.y - rv.y*lw/2), double(loc.z - rv.z*lw/2)];
                                obj.cache_RightBound(k, :) = [double(loc.x + rv.x*lw/2), double(loc.y + rv.y*lw/2), double(loc.z + rv.z*lw/2)];
                                nxts = wp.next(obj.LaneSampleDist);
                                if py.len(nxts) > 0
                                    wp = nxts{1};
                                else
                                    wp = py.None;
                                end
                            end
                        end
                    catch 
                    end
                end
                
                obj.world.tick();
                % 提取并缓存CARLA位姿
                try
                    actual_tf = obj.ghost_vehicle.get_transform();
                    obj.cache_PoseCarla = [double(actual_tf.location.x), double(actual_tf.location.y), double(actual_tf.rotation.yaw)];
                catch
                    % 忽略偶尔的通信断层
                end
                
                % Python路由下，传感器回调仍接收数据，但不跨语言复制。
                if obj.isSimulinkSensorRoute()
                    obj.updateSensorCaches();
                    if obj.EnableSave
                        obj.saveSensorFrame(clock_time);
                    end
                end
                
            end
            
            CENTERLINE = obj.cache_Centerline;
            L_BOUND = obj.cache_LeftBound;
            R_BOUND = obj.cache_RightBound;

            % 输出相对高度
            % 注意：InitialGroundZ 在四轮都完成道路初始化前可能仍包含 NaN，
            % 因此不能直接 obj.cache_TireZ - obj.InitialGroundZ。
            raw_relative_Z = zeros(4, 1);

            for zi = 1:4
                if ~isnan(obj.InitialGroundZ(zi))
                    raw_relative_Z(zi) = obj.cache_TireZ(zi) - obj.InitialGroundZ(zi);
                else
                    % 该轮胎初始地面高度尚未建立时，先输出 0，避免 NaN 进入 Simulink。
                    raw_relative_Z(zi) = 0.0;
                end

                if isnan(raw_relative_Z(zi)) || isinf(raw_relative_Z(zi))
                    raw_relative_Z(zi) = 0.0;
                end
            end

            if ~obj.is_filter_initialized
                obj.prev_Filtered_Z = raw_relative_Z;
                obj.is_filter_initialized = true;
            end

            alpha = obj.FilterAlpha;
            obj.prev_Filtered_Z = obj.prev_Filtered_Z * (1 - alpha) + raw_relative_Z * alpha;
            Z_GND = obj.prev_Filtered_Z;
            FRICTION_MU = obj.cache_TireMu;
            TARGET_DETECT = obj.cache_TargetDetect;
            % 最终输出位姿 (X, Y, Yaw)
            POSE_CARSIM = [vehiclePose(1), vehiclePose(2), vehiclePose(6)];
            POSE_CARLA = obj.cache_PoseCarla;

            % 端口数量和顺序始终固定。Python路由或传感器关闭时，
            % setupImpl初始化的零值缓存会直接输出，不触发跨语言大数组复制。
            outputs = {Z_GND, FRICTION_MU, TARGET_DETECT, ...
                CENTERLINE, L_BOUND, R_BOUND, POSE_CARSIM, POSE_CARLA, ...
                obj.cache_SS, obj.cache_DepthRaw, ...
                obj.cache_LeftImg, obj.cache_RightImg, ...
                obj.cache_LidarPoints, obj.cache_LidarCount};
            varargout = outputs;
        end

        function updateSensorCaches(obj)
            % 仅Simulink路由调用；Python路由避免所有大数组跨语言复制。
            if obj.EnableSemantic && ~isempty(obj.module_ss_rgb)
                try
                    ss_mat = uint8(py.getattr(obj.module_ss_rgb, 'array').copy());
                    obj.cache_SS = ss_mat(:, :, 3);
                catch ME
                    warning('Failed to read semantic sensor: %s', char(ME.message));
                end
            end

            if obj.EnableDepth && ~isempty(obj.module_depth)
                try
                    obj.cache_DepthRaw = ...
                        double(py.getattr(obj.module_depth, 'array').copy()) * 1000.0;
                catch ME
                    warning('Failed to read depth sensor: %s', char(ME.message));
                end
            end

            if obj.EnableStereoRGB
                try
                    if ~isempty(obj.module_left)
                        rgb_left = uint8(py.getattr(obj.module_left, 'array').copy());
                        obj.cache_LeftImg = rgb_left(:, :, 1:3);
                    end
                catch ME
                    warning('Failed to read left RGB sensor: %s', char(ME.message));
                end
                try
                    if ~isempty(obj.module_right)
                        rgb_right = uint8(py.getattr(obj.module_right, 'array').copy());
                        obj.cache_RightImg = rgb_right(:, :, 1:3);
                    end
                catch ME
                    warning('Failed to read right RGB sensor: %s', char(ME.message));
                end
            end

            if obj.EnableLidar && ~isempty(obj.module_lidar)
                try
                    points = single(obj.module_lidar.get_point_cloud());
                    point_count = min(size(points, 1), double(obj.LidarMaxPoints));
                    previous_count = double(obj.cache_LidarCount);
                    if point_count > 0
                        obj.cache_LidarPoints(1:point_count, :) = points(1:point_count, :);
                    end
                    if previous_count > point_count
                        obj.cache_LidarPoints((point_count + 1):previous_count, :) = 0;
                    end
                    obj.cache_LidarCount = double(point_count);
                catch ME
                    warning('Failed to read LiDAR sensor: %s', char(ME.message));
                end
            end
        end

        function saveSensorFrame(obj, clock_time)
            % 保存仅在低频CARLA视觉更新中执行；跳帧时不复制同一帧填补历史网格。
            if isempty(obj.current_save_dir) || isempty(obj.timestamp_fid)
                return;
            end
            current_index = int64(floor((double(clock_time) + 1e-9) / double(obj.SavePeriod)));
            if current_index <= obj.last_saved_index
                return;
            end

            frame_index = double(obj.frame_counter);
            if obj.EnableStereoRGB
                imwrite(obj.cache_LeftImg, fullfile(obj.current_save_dir, 'image_0', ...
                    sprintf('%06d.png', frame_index)));
                imwrite(obj.cache_RightImg, fullfile(obj.current_save_dir, 'image_1', ...
                    sprintf('%06d.png', frame_index)));
            end
            if obj.EnableLidar && ~isempty(obj.module_lidar) && ~isempty(obj.lidar_save_subdir)
                lidar_path = fullfile(obj.lidar_save_subdir, sprintf('%06d.npy', frame_index));
                obj.module_lidar.save_to_npy(py.str(char(lidar_path)));
            end
            fprintf(obj.timestamp_fid, '%d, %.6f\n', frame_index, ...
                double(current_index) * double(obj.SavePeriod));
            obj.frame_counter = obj.frame_counter + 1;
            obj.last_saved_index = current_index;
        end

        function releaseImpl(obj)
            % Cleanup: close timestamps file, destroy actors, restore world settings
            coder.extrinsic('py', 'fclose');
            
            if ~isempty(obj.timestamp_fid)
                try
                    fclose(obj.timestamp_fid);
                catch
                end
                obj.timestamp_fid = [];
            end
            
            try if ~isempty(obj.ghost_vehicle) && obj.ghost_vehicle.is_alive; obj.ghost_vehicle.destroy(); end; catch; end
            try if ~isempty(obj.ss_rgb) && obj.ss_rgb.is_alive; obj.ss_rgb.destroy(); end; catch; end
            try if ~isempty(obj.depth) && obj.depth.is_alive; obj.depth.destroy(); end; catch; end
            try if ~isempty(obj.cam_left) && obj.cam_left.is_alive; obj.cam_left.destroy(); end; catch; end
            try if ~isempty(obj.cam_right) && obj.cam_right.is_alive; obj.cam_right.destroy(); end; catch; end
            try if ~isempty(obj.lidar) && obj.lidar.is_alive; obj.lidar.destroy(); end; catch; end
            obj.module_lidar = [];

            obj.destroyRegisteredProps();
            % 使用独立快照恢复启动前设置，不强制改成异步模式。
            if ~isempty(obj.world) && ~isempty(obj.original_world_settings)
                try
                    obj.world.apply_settings(obj.original_world_settings);
                    obj.original_world_settings = [];
                catch ME
                    warning('V22:RestoreWorldSettings', 'Could not restore CARLA world settings: %s', char(ME.message));
                end
            end
        end


        function flag = isSimulinkSensorRoute(obj)
            flag = strcmpi(char(obj.SensorDataRoute), 'simulink');
        end

        function validateConfiguration(obj)
            validatestring(char(obj.SensorDataRoute), {'simulink', 'python'});
            validateattributes(obj.CarlaPort, {'numeric'}, {'scalar','integer','>=',1,'<=',65535});
            if ~isvarname(char(obj.PropConfigurationFunction)) || exist(char(obj.PropConfigurationFunction), 'file') ~= 2
                error('V22:PropConfigurationFunction', 'Provide a valid prop configuration function on the MATLAB path.');
            end
            validateattributes(obj.SimulinkDt, {'numeric'}, ...
                {'scalar', 'real', 'finite', 'positive'});
            validateattributes(obj.RaycastDt, {'numeric'}, ...
                {'scalar', 'real', 'finite', 'positive'});
            validateattributes(obj.FarRaycastDt, {'numeric'}, ...
                {'scalar', 'real', 'finite', '>=', obj.RaycastDt});
            validateattributes(obj.RayNearMargin, {'numeric'}, {'scalar','real','finite','nonnegative'});
            validateattributes(obj.RayExitHysteresis, {'numeric'}, {'scalar','real','finite','nonnegative'});
            validateattributes(obj.RayApproachSpeed, {'numeric'}, {'scalar','real','finite','positive'});
            validateattributes(obj.CarlaDt, {'numeric'}, ...
                {'scalar', 'real', 'finite', 'positive'});
            validateattributes(obj.LidarMaxPoints, {'numeric'}, ...
                {'scalar', 'integer', 'positive'});
            validateattributes(obj.SavePeriod, {'numeric'}, {'scalar', 'positive'});
            validateattributes(obj.RoadFriction, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
            validateattributes(obj.FilterAlpha, {'numeric'}, {'scalar', 'real', 'finite', '>=', 0, '<=', 1});
            if obj.EnableSave && ~obj.isSimulinkSensorRoute()
                warning(['EnableSave is ignored for SensorDataRoute="python". ', ...
                    'Python callbacks still receive and retain the latest sensor data.']);
            end
        end

        function names = getConfiguredOutputNames(~)
            % 固定端口使路由和传感器开关变化时无需重新连线。
            names = {'Z_GND', 'FRICTION_MU', 'TARGET_DETECT', ...
                'CENTERLINE', 'L_BOUND', 'R_BOUND', ...
                'POSE_CARSIM', 'POSE_CARLA', ...
                'SEMANTIC', 'DEPTH', 'LEFTIMG', 'RIGHTIMG', ...
                'LIDAR_POINTS', 'LIDAR_COUNT'};
        end

        function spawnRegisteredProps(obj, definitions)
            % 每个成功生成的actor立即登记，摩擦/高度响应由其配置决定。
            bp_lib = obj.world.get_blueprint_library();
            try
                for d = 1:numel(definitions)
                    spec = definitions(d);
                    blueprint = bp_lib.find(char(spec.Blueprint));
                    for row = 1:size(spec.Transforms, 1)
                        pose = double(spec.Transforms(row, :));
                        tf = py.carla.Transform( ...
                            py.carla.Location(pyargs('x', pose(1), 'y', pose(2), 'z', pose(3))), ...
                            py.carla.Rotation(pyargs('yaw', pose(4))));
                        actor = obj.world.try_spawn_actor(blueprint, tf);
                        if isequal(actor, py.None)
                            error('V22:PropSpawnFailed', ...
                                'Could not spawn prop definition %d, transform row %d.', d, row);
                        end
                        record = struct('Actor', actor, 'SpawnPose', pose, 'Mu', double(spec.Mu), ...
                            'UseSurfaceHeight', spec.UseSurfaceHeight, ...
                            'ZScale', double(spec.ZScale), 'Tolerance', double(spec.Tolerance));
                        obj.registered_props{end + 1} = record;
                        if py.hasattr(actor, 'set_simulate_physics')
                            actor.set_simulate_physics(false);
                        end
                    end
                end
                % 几何快照在setup推进world.tick后读取，不能在actor刚生成时缓存。
            catch ME
                obj.destroyRegisteredProps();
                rethrow(ME);
            end
        end

        function props = getRegisteredPropGeometry(obj)
            props = struct('Center', {}, 'Extent', {}, 'Yaw', {}, 'Mu', {}, ...
                'UseSurfaceHeight', {}, 'ZScale', {}, 'Tolerance', {});
            for k = 1:numel(obj.registered_props)
                record = obj.registered_props{k};
                actor = record.Actor;
                if ~actor.is_alive; continue; end
                tf = actor.get_transform();
                actualPosition = [double(tf.location.x), double(tf.location.y), double(tf.location.z)];
                if norm(actualPosition - record.SpawnPose(1:3)) > 1e-3
                    error('EVIDS:PropSnapshotNotReady', ...
                        'Prop %d snapshot does not match its spawn pose. Expected %s, got %s.', ...
                        k, mat2str(record.SpawnPose(1:3)), mat2str(actualPosition));
                end
                bbox = actor.bounding_box;
                bboxYaw = 0; bboxPitch = 0; bboxRoll = 0;
                if py.hasattr(bbox, 'rotation')
                    bboxYaw = double(bbox.rotation.yaw);
                    bboxPitch = double(bbox.rotation.pitch);
                    bboxRoll = double(bbox.rotation.roll);
                end
                if any(abs([double(tf.rotation.pitch), double(tf.rotation.roll), ...
                        bboxPitch, bboxRoll]) > 1e-5)
                    error('V22:TiltedProp', ...
                        'Registered props currently require horizontal bounding boxes (yaw rotation is supported).');
                end
                yaw = double(tf.rotation.yaw);
                c = cosd(yaw); s = sind(yaw);
                offsetX = double(bbox.location.x); offsetY = double(bbox.location.y);
                center = [double(tf.location.x) + c * offsetX - s * offsetY, ...
                    double(tf.location.y) + s * offsetX + c * offsetY, ...
                    double(tf.location.z) + double(bbox.location.z)];
                extent = [double(bbox.extent.x), double(bbox.extent.y), double(bbox.extent.z)];
                if any(~isfinite([center, extent, yaw, bboxYaw])) || any(extent < 0)
                    error('V22:PropBounds', 'Registered prop has invalid bounding-box geometry.');
                end
                props(end + 1) = struct('Center', center, 'Extent', extent, ...
                    'Yaw', yaw + bboxYaw, 'Mu', record.Mu, ...
                    'UseSurfaceHeight', record.UseSurfaceHeight, ...
                    'ZScale', record.ZScale, 'Tolerance', record.Tolerance);
            end
        end

        function destroyRegisteredProps(obj)
            % 仅销毁本次登记并由本程序生成的道具，不搜索/删除其他场景道具。
            for k = 1:numel(obj.registered_props)
                try
                    actor = obj.registered_props{k}.Actor;
                    if actor.is_alive; actor.destroy(); end
                catch
                end
            end
            obj.registered_props = {};
            obj.cached_prop_geometry = struct('Center', {}, 'Extent', {}, 'Yaw', {}, 'Mu', {}, ...
                'UseSurfaceHeight', {}, 'ZScale', {}, 'Tolerance', {});
        end

        function validateInputsImpl(~, vehiclePose, tirePosFlatX, tirePosFlatY, clock_time)
            validateattributes(vehiclePose, {'double'}, {'vector', 'numel', 6});
            validateattributes(tirePosFlatX, {'double'}, {'vector', 'numel', 4});
            validateattributes(tirePosFlatY, {'double'}, {'vector', 'numel', 4});
            validateattributes(clock_time, {'double'}, {'scalar', 'real', 'finite'});
        end

        function count = getNumOutputsImpl(obj)
            count = numel(obj.getConfiguredOutputNames());
        end

        function varargout = getOutputNamesImpl(obj)
            varargout = obj.getConfiguredOutputNames();
        end

        function varargout = getOutputSizeImpl(obj)
            sizes = {[4 1], [4 1], [4 1], ...
                [obj.LaneSamplePoints 3], [obj.LaneSamplePoints 3], ...
                [obj.LaneSamplePoints 3], [1 3], [1 3], ...
                [obj.height obj.width], [obj.height obj.width], ...
                [obj.height obj.width 3], [obj.height obj.width 3], ...
                [double(obj.LidarMaxPoints) 4], 1};
            varargout = sizes;
        end

        function varargout = getOutputDataTypeImpl(~)
            types = {'double', 'double', 'double', 'double', ...
                'double', 'double', 'double', 'double', ...
                'uint8', 'double', 'uint8', 'uint8', ...
                'single', 'double'};
            varargout = types;
        end

        function varargout = isOutputFixedSizeImpl(obj)
            varargout = repmat({true}, 1, obj.getNumOutputsImpl());
        end

        function varargout = isOutputComplexImpl(obj)
            varargout = repmat({false}, 1, obj.getNumOutputsImpl());
        end
    end
end
