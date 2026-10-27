# Quick Start / 快速开始

[English](#english) | [中文](#中文)

## English

1. Install the software listed in [README](README.md) and run `python -m pip install -r requirements.txt` in your Python 3.7 environment.
2. Set the MATLAB current folder to the repository directory, or add the directory to the MATLAB path.
3. Fill in the installation paths at the top of `evids_test_initialize.m`.
4. In CarSim, select the vehicle, road, and driving procedure. Configure the 12 imports and 14 exports in the order specified in the [interface guide](docs/CarSim接口.md).
5. Set the associated model to this repository's `EVIDS_core.slx` and select **Send to Simulink**.
6. Click **Run** in Simulink. The model initializes its dependencies automatically. Run duration is set by `env.StopTime`.
7. Open `results/<timestamp>/` for the MAT data, wheel-feedback CSV/PNG, numerical summary, spawn diagnostics, and prop geometry.

Keep the CarSim-generated simfile.sim and parameter files accessible to the solver. Match the CARLA map, prop locations, and vehicle trajectory to the selected scenario. The included prop configuration uses Town04.

## 中文

1. 安装 [README](README.md) 中列出的软件，并在 Python 3.7 环境执行 `python -m pip install -r requirements.txt`。
2. 将 MATLAB 当前文件夹切换到仓库目录，或将该目录加入 MATLAB 搜索路径。
3. 在 `evids_test_initialize.m` 开头填写自己的安装路径。
4. 在 CarSim 中选择车辆、道路和行驶工况，按 [接口说明](docs/CarSim接口.md) 的顺序配置 12 个 Import、14 个 Export。
5. 将关联模型设为本仓库的 `EVIDS_core.slx`，点击 **Send to Simulink**。
6. 在 Simulink 中点击 **Run**，模型会自动完成依赖初始化。仿真时长由 `env.StopTime` 控制。
7. 在 `results/<时间戳>/` 中查看 MAT 数据、四轮反馈 CSV/PNG、数值汇总、车辆生成诊断及道具几何。

CarSim 生成的 simfile.sim 和参数文件须能被求解器访问。CARLA 地图、道具位置和车辆轨迹应对应所选场景，随附道具配置使用 Town04。
