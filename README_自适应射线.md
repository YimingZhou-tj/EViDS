# Ray Feedback / 射线反馈

[English](#english) | [中文](#中文)

## English

### Outputs

The four-wheel ray module returns three vectors, ordered L1, R1, L2, R2:

| Output | Meaning |
| --- | --- |
| `Z_GND` | Ground height relative to each wheel's initial road baseline, in metres. |
| `FRICTION_MU` | Friction coefficient; ordinary roads use `RoadFriction`, which defaults to 1. |
| `TARGET_DETECT` | 1 when a ray hit matches a registered prop; otherwise 0. |

Only registered props receive their configured friction and height responses. Proximity to a prop controls the query interval; it does not directly assign Mu or Z.

### Prop configuration and geometry

`evids_test_props.m` defines each prop's blueprint, spawn transforms, Mu, `UseSurfaceHeight`, `ZScale`, and matching tolerance. The included configuration uses Town04 with one raised prop and a tiled low-friction area.

After spawning the props, the module advances the CARLA world and reads their actual transforms and bounding boxes. It checks the positions against the requested spawn positions and caches the geometry for subsequent queries. Registered props must be static and have horizontal bounding boxes; rotation about the vertical axis is supported. Restart the simulation after moving or deleting these props externally.

### Query scheduling

At every Simulink step, the scheduler calculates the minimum XY distance from any wheel to any registered prop's oriented bounding box using the cached geometry.

| Property | Default | Meaning |
| --- | --- | --- |
| `RaycastDt` | 0.01 s | Query interval near props: 100 Hz. |
| `FarRaycastDt` | 0.1 s | Query interval away from props: 10 Hz. |
| `RayNearMargin` | 3 m | Additional approach margin outside the prop bounds. |
| `RayExitHysteresis` | 2 m | Extra exit distance to reduce repeated switching. |
| `RayApproachSpeed` | 80 m/s | Minimum speed used to calculate look-ahead distance. |

The look-ahead speed is the larger of `RayApproachSpeed` and the maximum wheel XY speed measured from consecutive simulation inputs:

```text
entry_distance = RayNearMargin + speed * (FarRaycastDt + RaycastDt)
exit_distance  = entry_distance + RayExitHysteresis
```

With the default speed and intervals, the entry distance is 11.8 m and the exit distance is 13.8 m. Higher measured speeds increase both distances. The 80 m/s parameter is a conservative look-ahead value, not a vehicle speed limit.

Entering the near region triggers a query immediately. The shorter interval is also used until all four initial road baselines have been established. Between queries, the latest feedback values are held.

Road heights away from props are sampled at the far interval as well. Reduce `FarRaycastDt` for fine road irregularities; setting it equal to `RaycastDt` uses the same interval throughout the scene.

### Hit processing

At each scheduled query, one vertical ray is cast for each wheel using its current mapped XY position. Hits are sorted from high to low. Vehicle-labelled hits are skipped. Registered prop matches are determined using their XY oriented bounding boxes and Z bounds, while road hits establish the road height and initial baseline.

For a matched prop, the configured Mu is returned and `TARGET_DETECT` becomes 1. If `UseSurfaceHeight` is false, Z follows the underlying road. If it is true, the prop's hit height contributes according to `ZScale`:

```text
relative_Z = ZScale * (prop_hit_Z - initial_road_Z)
```

An optional first-order filter is applied using `FilterAlpha`. The default `FilterAlpha=1` passes the computed height directly. A wheel whose initial road baseline is unavailable returns Z=0 until that baseline is established.

### Implementation and saved data

- `EVIDSClosedLoopSensorsV2.m`: prop lifecycle, cached geometry, four-wheel queries, feedback, and optional sensor interfaces.
- `evids_test_ray_schedule.m`: proximity, look-ahead, and interval selection.
- `evids_test_resolve_ray_hits.m`: road/prop matching and height/friction responses.

Each run saves `prop_geometry_snapshot.mat` with the cached prop geometry and response settings. Wheel Z, Mu, and detection flags are included in the run's MAT, CSV, and PNG outputs.

## 中文

### 输出定义

四轮射线模块输出以下三个向量，轮序均为 L1、R1、L2、R2：

| 输出 | 含义 |
| --- | --- |
| `Z_GND` | 相对各轮初始道路基准的地面高度，单位为米。 |
| `FRICTION_MU` | 附着系数；普通道路采用 `RoadFriction`，默认为 1。 |
| `TARGET_DETECT` | 射线命中与已登记道具匹配时为 1，否则为 0。 |

只有已登记道具采用其配置的附着和高度响应。与道具的距离只控制查询间隔，不直接赋予 Mu 或 Z。

### 道具配置与几何缓存

`evids_test_props.m` 定义各道具的蓝图、生成位姿、Mu、`UseSurfaceHeight`、`ZScale` 和匹配容差。随附配置使用 Town04，包含一个凸起道具和铺设的低附着区域。

道具生成后，模块推进 CARLA 世界，读取实际位姿与包围盒，检查实际位置与要求的生成位置一致，再缓存几何信息用于后续查询。已登记道具须保持静止且包围盒水平，支持绕竖直轴旋转。若从外部移动或删除这些道具，应重新开始仿真。

### 查询频率调度

每个 Simulink 步利用缓存几何，计算任一车轮到任一道具有向包围盒的最小 XY 距离。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `RaycastDt` | 0.01 s | 接近道具时的查询周期，对应 100 Hz。 |
| `FarRaycastDt` | 0.1 s | 远离道具时的查询周期，对应 10 Hz。 |
| `RayNearMargin` | 3 m | 道具包围盒外的额外提前距离。 |
| `RayExitHysteresis` | 2 m | 退出时增加的距离，减少频繁切换。 |
| `RayApproachSpeed` | 80 m/s | 计算提前距离时采用的速度下限。 |

提前量速度取 `RayApproachSpeed` 与相邻仿真输入估算出的最大轮位 XY 移动速度中的较大值：

```text
进入距离 = RayNearMargin + speed * (FarRaycastDt + RaycastDt)
退出距离 = 进入距离 + RayExitHysteresis
```

按默认速度和周期计算，进入距离为 11.8 m，退出距离为 13.8 m。估算速度更大时，两者相应增加。80 m/s 是保守的提前量计算值，不限制车辆速度。

进入接近区域时立即查询一次。四轮初始道路基准尚未全部建立时，也采用较短周期。两次查询之间保持最近的反馈值。

远离道具时，道路高度同样按远处周期采样。研究细小路面起伏时应缩短 `FarRaycastDt`；将其设为与 `RaycastDt` 相同，可在全场景采用统一周期。

### 命中解析

每次调度查询使用各轮当前映射后的 XY 位置，各发射一条竖直射线。命中点按高度从高到低排序，跳过车辆语义命中。模块利用道具的 XY 有向包围盒和 Z 范围匹配已登记道具，同时利用道路命中获取道路高度和初始基准。

匹配到道具后，返回其配置的 Mu，并将 `TARGET_DETECT` 设为 1。`UseSurfaceHeight=false` 时，Z 采用下方道路高度；为 true 时，按 `ZScale` 使用道具命中高度：

```text
相对 Z = ZScale * (道具命中高度 - 初始道路高度)
```

高度随后可通过 `FilterAlpha` 进行一阶滤波，默认 `FilterAlpha=1`，直接传递计算结果。某轮尚未建立初始道路基准时，该轮 Z 暂时输出 0。

### 实现文件及保存数据

- `EVIDSClosedLoopSensorsV2.m`：道具生命周期、几何缓存、四轮查询、反馈及可选传感器接口。
- `evids_test_ray_schedule.m`：距离判断、速度提前量及周期选择。
- `evids_test_resolve_ray_hits.m`：道路/道具匹配及高度、附着响应。

每次运行保存 `prop_geometry_snapshot.mat`，记录缓存的道具几何和响应配置。四轮 Z、Mu 和命中标志包含在对应运行的 MAT、CSV 和 PNG 输出中。
