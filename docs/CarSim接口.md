# CarSim Interface / CarSim 接口

[English](#english) | [中文](#中文)

## English

### Signal directions and wheel order

Model: `EVIDS_core.slx`.

- **Imports:** 12 values sent from Simulink to CarSim for wheel-ground feedback.
- **Exports:** 14 values sent from CarSim to Simulink for vehicle pose and wheel positions.
- **Wheel order:** L1, R1, L2, R2 — front left, front right, rear left, rear right.

The CarSim S-Function has one input vector and one output vector. Configure the channel order exactly as specified below.

### Imports: Simulink to CarSim

| Index | CarSim variable | Feedback | Unit |
| --- | --- | --- | --- |
| 1 | IMP_ZGND_L1 | Front-left relative ground height | m |
| 2 | IMP_ZGND_R1 | Front-right relative ground height | m |
| 3 | IMP_ZGND_L2 | Rear-left relative ground height | m |
| 4 | IMP_ZGND_R2 | Rear-right relative ground height | m |
| 5 | IMP_MUX_L1 | Front-left longitudinal friction | Dimensionless |
| 6 | IMP_MUX_R1 | Front-right longitudinal friction | Dimensionless |
| 7 | IMP_MUX_L2 | Rear-left longitudinal friction | Dimensionless |
| 8 | IMP_MUX_R2 | Rear-right longitudinal friction | Dimensionless |
| 9 | IMP_MUY_L1 | Front-left lateral friction | Dimensionless |
| 10 | IMP_MUY_R1 | Front-right lateral friction | Dimensionless |
| 11 | IMP_MUY_L2 | Rear-left lateral friction | Dimensionless |
| 12 | IMP_MUY_R2 | Rear-right lateral friction | Dimensionless |

Select **Replace** mode for all imports. Simulink constructs the vector as:

```matlab
[Z(1:4); Mu(1:4); Mu(1:4)]
```

Each wheel's initial road hit establishes its height baseline. The returned Z is relative to that baseline. The same Mu is supplied to longitudinal and lateral friction; ordinary roads use Mu=1.

### Exports: CarSim to Simulink

| Index | CarSim variable | Purpose | Unit |
| --- | --- | --- | --- |
| 1 | Xo | Vehicle X coordinate | m |
| 2 | Yo | Vehicle Y coordinate | m |
| 3 | Zcg_TM | Vehicle height input | m |
| 4 | Pitch | Pitch angle | deg |
| 5 | Roll_E | Roll angle input | deg |
| 6 | Yaw | Yaw angle | deg |
| 7 | Xctc_L1 | Front-left contact-point X | m |
| 8 | Yctc_L1 | Front-left contact-point Y | m |
| 9 | Xctc_R1 | Front-right contact-point X | m |
| 10 | Yctc_R1 | Front-right contact-point Y | m |
| 11 | Xctc_L2 | Rear-left contact-point X | m |
| 12 | Yctc_L2 | Rear-left contact-point Y | m |
| 13 | Xctc_R2 | Rear-right contact-point X | m |
| 14 | Yctc_R2 | Rear-right contact-point Y | m |

The model groups these channels into:

```text
Vehicle pose: [1 2 3 4 5 6]
Wheel X:      [7 9 11 13]
Wheel Y:      [8 10 12 14]
```

Use the listed variables, including Zcg_TM and Roll_E, and configure the stated units in CarSim.

### Coordinate mapping

The `Coordinate conversion` block maps the exported state into CARLA coordinates. It stores the first frame's X, Y, and yaw as a reference and mirrors the lateral component of positions about the initial heading axis. The same XY mapping is applied to the vehicle and all four contact points.

Let the initial position be p0 and the initial yaw be yaw0. Define the forward unit vector f=[cosd(yaw0),sind(yaw0)] and lateral unit vector l=[sind(yaw0),-cosd(yaw0)]. For any XY point p:

```text
p_CARLA = p0 + dot(p-p0, f)*f - dot(p-p0, l)*l
```

Height is passed through. Attitude is mapped as follows:

```text
Pitch_CARLA = -Pitch
Roll_CARLA  = Roll_E
Yaw_CARLA   = wrapTo180(2*yaw0 - Yaw)
```

Angles in the block are expressed in degrees; wrapTo180 maps to [-180,180).

### Configure and run a CarSim case

1. In MATLAB, set the current folder to the repository directory, or add the directory to the MATLAB path.
2. Fill in the installation paths at the top of `evids_test_initialize.m`.
3. In CarSim, select a vehicle and road/procedure appropriate for the intended scenario.
4. Configure the 12 imports in Replace mode and the 14 exports using the exact orders and units above. Save the case.
5. Set the case's Simulink model to `EVIDS_core.slx` in this repository and select **Send to Simulink**.
6. Click **Run** in Simulink. Duration is controlled by `env.StopTime` in the initialization file.

CarSim generates `simfile.sim` and its associated parameter files for the selected case. Keep those generated files available to the solver. Generate the case through CarSim rather than manually editing channel-count declarations in simfile.sim. The EViDS initializer preserves the SIMFILE path selected by CarSim.

The vehicle trajectory, CARLA map, and configured prop locations must describe the same scenario. The included prop configuration uses Town04; modify the prop configuration and the System block's map selection together when using another map.

## 中文

### 信号方向与轮序

主体模型：`EVIDS_core.slx`。

- **Import：** Simulink 向 CarSim 发送的 12 项轮地反馈。
- **Export：** CarSim 向 Simulink 发送的 14 项车身位姿及四轮位置。
- **轮序：** L1、R1、L2、R2，即左前、右前、左后、右后。

CarSim S-Function 使用一个输入向量和一个输出向量。通道顺序须与下表完全一致。

### Import：Simulink → CarSim

| 序号 | CarSim 变量 | 反馈含义 | 单位 |
| --- | --- | --- | --- |
| 1 | IMP_ZGND_L1 | 左前轮相对地面高度 | m |
| 2 | IMP_ZGND_R1 | 右前轮相对地面高度 | m |
| 3 | IMP_ZGND_L2 | 左后轮相对地面高度 | m |
| 4 | IMP_ZGND_R2 | 右后轮相对地面高度 | m |
| 5 | IMP_MUX_L1 | 左前轮纵向附着系数 | 无量纲 |
| 6 | IMP_MUX_R1 | 右前轮纵向附着系数 | 无量纲 |
| 7 | IMP_MUX_L2 | 左后轮纵向附着系数 | 无量纲 |
| 8 | IMP_MUX_R2 | 右后轮纵向附着系数 | 无量纲 |
| 9 | IMP_MUY_L1 | 左前轮横向附着系数 | 无量纲 |
| 10 | IMP_MUY_R1 | 右前轮横向附着系数 | 无量纲 |
| 11 | IMP_MUY_L2 | 左后轮横向附着系数 | 无量纲 |
| 12 | IMP_MUY_R2 | 右后轮横向附着系数 | 无量纲 |

各项均选择 **Replace** 模式。Simulink 按下面的方式构造输入向量：

```matlab
[Z(1:4); Mu(1:4); Mu(1:4)]
```

每个车轮首次命中的道路高度用于建立该轮的高度基准，输出 Z 为相对此基准的高度。纵向与横向采用同一 Mu，普通道路默认 Mu=1。

### Export：CarSim → Simulink

| 序号 | CarSim 变量 | 用途 | 单位 |
| --- | --- | --- | --- |
| 1 | Xo | 车身 X 坐标 | m |
| 2 | Yo | 车身 Y 坐标 | m |
| 3 | Zcg_TM | 车身高度输入 | m |
| 4 | Pitch | 俯仰角 | deg |
| 5 | Roll_E | 侧倾角输入 | deg |
| 6 | Yaw | 航向角 | deg |
| 7 | Xctc_L1 | 左前轮接地点 X | m |
| 8 | Yctc_L1 | 左前轮接地点 Y | m |
| 9 | Xctc_R1 | 右前轮接地点 X | m |
| 10 | Yctc_R1 | 右前轮接地点 Y | m |
| 11 | Xctc_L2 | 左后轮接地点 X | m |
| 12 | Yctc_L2 | 左后轮接地点 Y | m |
| 13 | Xctc_R2 | 右后轮接地点 X | m |
| 14 | Yctc_R2 | 右后轮接地点 Y | m |

模型将通道分为三组：

```text
车身位姿：[1 2 3 4 5 6]
四轮 X： [7 9 11 13]
四轮 Y： [8 10 12 14]
```

请使用表中指定的变量，包括 Zcg_TM 和 Roll_E，并在 CarSim 中设置对应单位。

### 坐标映射

`Coordinate conversion` 模块将导出的状态转换为 CARLA 坐标。模块保存首帧 X、Y 和 Yaw 作为参考，以初始航向轴为基准，对位置的横向分量进行镜像。车身位置与四轮接地点使用相同的 XY 映射。

设初始位置为 p0、初始航向为 yaw0，前向单位向量为 f=[cosd(yaw0),sind(yaw0)]，横向单位向量为 l=[sind(yaw0),-cosd(yaw0)]，则任意 XY 点 p 的映射为：

```text
p_CARLA = p0 + dot(p-p0, f)*f - dot(p-p0, l)*l
```

高度直接传递，姿态角按下式转换：

```text
Pitch_CARLA = -Pitch
Roll_CARLA  = Roll_E
Yaw_CARLA   = wrapTo180(2*yaw0 - Yaw)
```

模块中的角度单位为度，wrapTo180 将角度限制在 [-180,180) 区间。

### 配置并运行 CarSim 工况

1. 将 MATLAB 当前文件夹切换到本仓库目录，或将该目录加入 MATLAB 搜索路径。
2. 在 `evids_test_initialize.m` 开头填写自己的安装路径。
3. 在 CarSim 中选择与目标场景对应的车辆、道路和行驶工况。
4. 按上表配置 12 个 Import 和 14 个 Export，确认 Import 为 Replace 模式、顺序和单位正确，然后保存工况。
5. 将工况关联的 Simulink 模型设为本仓库中的 `EVIDS_core.slx`，点击 **Send to Simulink**。
6. 在 Simulink 中点击 **Run**。运行时长由初始化文件中的 `env.StopTime` 控制。

CarSim 根据所选工况生成 `simfile.sim` 和相关参数文件，这些文件须能被求解器访问。请通过 CarSim 生成工况配置，不直接修改 simfile.sim 中的通道数量声明。EViDS 初始化函数会保留 CarSim 指定的 SIMFILE 路径。

车辆轨迹、CARLA 地图和道具位置须对应同一场景。随附道具配置使用 Town04；使用其他地图时，应同时调整道具配置与 System 模块的地图选择。
