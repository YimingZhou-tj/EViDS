# CarSim核心输入输出顺序

适用模型：`EVIDS_core.slx`。CarSim Import是Simulink送入CarSim的反馈；CarSim Export是CarSim送入Simulink的状态。两个方向必须分别按下面的顺序设置。

轮序统一为L1、R1、L2、R2，即左前、右前、左后、右后。

## Import：12项，保持原顺序

| 序号 | CarSim变量 | 来自Simulink |
| --- | --- | --- |
| 1 | IMP_ZGND_L1 | 左前轮Z |
| 2 | IMP_ZGND_R1 | 右前轮Z |
| 3 | IMP_ZGND_L2 | 左后轮Z |
| 4 | IMP_ZGND_R2 | 右后轮Z |
| 5 | IMP_MUX_L1 | 左前轮Mu |
| 6 | IMP_MUX_R1 | 右前轮Mu |
| 7 | IMP_MUX_L2 | 左后轮Mu |
| 8 | IMP_MUX_R2 | 右后轮Mu |
| 9 | IMP_MUY_L1 | 左前轮Mu |
| 10 | IMP_MUY_R1 | 右前轮Mu |
| 11 | IMP_MUY_L2 | 左后轮Mu |
| 12 | IMP_MUY_R2 | 右后轮Mu |

拼接为`[Z(1:4); Mu(1:4); Mu(1:4)]`，纵向和横向使用相同Mu。Import模式保持原Replace。Z单位为米，是各轮相对初始道路基准的高度；Mu为无量纲，普通道路默认1。此整理没有改变原有反馈定义。

## Export：从原51项精简为14项

| 新序号 | CarSim变量 | 原51项序号 | 用途/单位 |
| --- | --- | --- | --- |
| 1 | Xo | 1 | 车身X，m |
| 2 | Yo | 2 | 车身Y，m |
| 3 | Zcg_TM | 50 | 现有模型采用的车身高度，m |
| 4 | Pitch | 4 | 俯仰角，deg |
| 5 | Roll_E | 51 | 现有模型采用的侧倾角，deg |
| 6 | Yaw | 6 | 航向角，deg |
| 7 | Xctc_L1 | 35 | 左前轮接地点X，m |
| 8 | Yctc_L1 | 36 | 左前轮接地点Y，m |
| 9 | Xctc_R1 | 37 | 右前轮接地点X，m |
| 10 | Yctc_R1 | 38 | 右前轮接地点Y，m |
| 11 | Xctc_L2 | 39 | 左后轮接地点X，m |
| 12 | Yctc_L2 | 40 | 左后轮接地点Y，m |
| 13 | Xctc_R2 | 41 | 右后轮接地点X，m |
| 14 | Yctc_R2 | 42 | 右后轮接地点Y，m |

前三项及姿态选择沿用已验证版本；不要用旧列表中的`Zo`替换`Zcg_TM`，也不要用`Roll`替换`Roll_E`。模型直接接收角度制姿态，配置时应核对CarSim实际导出单位。

模型只保留三个状态选择器：位姿1:6、轮X=[7 9 11 13]、轮Y=[8 10 12 14]。原侧偏角、速度、轮速、轮胎力、驱动力矩、道路站距等算法专用输出及其选择/显示/记录支路已从新模型删除。

## 坐标映射保持原实现

`Coordinate conversion`沿用旧模型的MATLAB Function：以首帧X/Y/Yaw建立参考，镜像横向位置；高度不变；Pitch取负，Roll保持原符号，Yaw按`2*yawRef0-yaw`包裹到[-180,180)。四轮位置执行相同XY映射。没有为精简通道修改场景位置或坐标算法。

## CarSim中需要手动完成

1. 复制已验证工况作为14输出工况，保留车辆、道路及12个Import设置。
2. 在Export列表中仅保留上述14项，严格按新序号排序；保存工况。
3. 先在evids_test_initialize.m开头填写本机安装路径，将MATLAB当前文件夹切换到本目录，再将CarSim关联模型设为本目录的`EVIDS_core.slx`。
4. 再次Send to Simulink，确认生成的配置为12输入/14输出，然后Run。

原`simfile.sim`及其外部Run_all.par仍属于原51输出工况，不能通过修改一个PORTS_EXP数字冒充完整14输出工况；应由CarSim重新生成。

