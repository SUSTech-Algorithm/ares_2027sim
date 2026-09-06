# ares_2027sim

`ares_2027sim` 是面向 ABU Robocon 2027 比赛的独立 ROS 2 Jazzy 仿真包，
使用 Gazebo Harmonic 构建比赛场地、物理环境和后续机器人仿真。项目不依赖现有的
`manipulator` 项目。

## 当前实现

目前的第一版场地包含：

- Ground、L1 和 L2 三级实体平台及对应颜色区域；
- 红蓝双方镜像布置的 3.5 m 实心坡道和 Transfer Area；
- Ground 通往 L1 的四级台阶；
- 每队一个 L1 通往 L2 的实体踏步，尺寸为
  `300 mm × 1000 mm × 150 mm`，L2 灰色平台表面构成第二级；
- L1、L2、坡道、台阶和 Transfer Area 的可见、可碰撞立面；
- 每队一个 `700 mm × 700 mm` 的 L1 BR Retry Zone；
- Ground 与 L1 的红蓝隔离墙、外围挡板及对应碰撞体；
- L1 的 6 个 Building Spot 和 L2 的 4 个 Building Spot；
- Ground Mustika Pillar 与 L2 Central Pillar，包括可碰撞的顶部圆柱形凹槽；
- 红蓝双方各 20 个 `350 mm` Earth Block，按每层 10 个、最多两层放在各自 Storage Area；
- 12 个 `200 mm` Sky Block，按中文版规则图 3 放在 Ground Shared Area 的 5 × 5
  交错点位，中心点留空；
- 每个 Sky Block 由红、蓝两个半立方体组成：四个外表面红蓝各半，两个相对端面分别为
  纯红和纯蓝；初始红蓝分界面朝上，红色半面的方向按图 3 在南北、东西之间交替 90°；
- 放置在 Ground Mustika Pillar 上的动态 Mustika 球，以及运行中的一键复位工具。

Earth Block、Sky Block 和 Mustika 都是带质量、惯量、摩擦与碰撞体的独立动态模型。当前采用
规则给出的质量范围中值：Earth Block 为 `0.275 kg`，Sky Block 为 `0.115 kg`，Mustika 为
`0.420 kg`。这些参数可在取得实物测量结果后继续标定。

## 坐标约定

```text
原点：场地中心
+X：向左，蓝方
-X：向右，红方
+Y：向前
+Z：向上
单位：米
```

## 构建和启动

项目的 Python、生成脚本、测试和 `colcon` 命令统一使用工作区根目录的 `.venv`。
应先激活虚拟环境，再加载 ROS 2 环境。

首次安装 Gazebo 与 ROS 2 桥接组件：

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz
```

安装 Python 开发依赖并构建：

```bash
cd /home/gracekite/Documents/MyFiles/RC_Projects/AlgoWorkspace-27
source .venv/bin/activate
python -m pip install -r ares_2027sim/requirements-dev.txt

cd ares_2027sim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python"
source install/setup.bash
ros2 launch ares_2027sim ares_2027sim.launch.py
```

## 复位 Mustika

仿真运行时，在另一个已加载工作区环境的终端执行：

```bash
cd /home/gracekite/Documents/MyFiles/RC_Projects/AlgoWorkspace-27/ares_2027sim
source ../.venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ares_2027sim reset_mustika.py
```

该命令通过 Gazebo 的世界服务把 Mustika 恢复到 Ground Mustika Pillar 的初始位置
`(0.000, 4.225, 0.545) m`，可在同一场仿真中重复执行。

## 重新生成场地

提交到项目中的 SDF 世界文件由参数化 Python 脚本生成。修改场地尺寸或颜色后执行：

```bash
cd /home/gracekite/Documents/MyFiles/RC_Projects/AlgoWorkspace-27
source .venv/bin/activate
python ares_2027sim/scripts/generate_robocon_2027_world.py
```

生成结果位于 `ares_2027sim/worlds/robocon_2027.sdf`。

## 测试

```bash
cd /home/gracekite/Documents/MyFiles/RC_Projects/AlgoWorkspace-27
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
python -m pytest ares_2027sim/test
```

测试覆盖场地尺寸、区域坐标、红蓝镜像关系、台阶尺寸、Retry Zone、Building Spot、
柱体凹槽、主要碰撞结构，以及 Earth/Sky Block 的数量、尺寸、质量与初始摆放。

## Gazebo 可以仿真的比赛内容

### 场地与通行

Gazebo 可以验证机器人在三级场地中的实际通行能力，包括爬坡、上下台阶、越过平台边缘、
通过 Transfer Area 开口以及与挡板和隔离墙发生碰撞。通过配置机器人质量、重心、轮胎摩擦、
电机扭矩和悬挂参数，可以判断底盘是否打滑、托底、倾覆或无法跨越台阶。

### 机器人机构

TR 和 BR 可以分别使用 URDF 或 SDF 建模。仿真可覆盖底盘、轮子、关节、机械臂、升降机构、
夹爪和末端执行器，并向现有 ROS 2 控制、导航和任务程序提供接近真机的关节状态与控制接口。
这适合验证取块姿态、机械臂工作空间、结构干涉以及在 L1、L2 上的操作高度。

### 比赛道具与接触

Earth Block、Sky Block 和 Mustika 可以作为带质量、惯量、摩擦和碰撞体的动态物体。
Gazebo 能模拟抓取、抬升、释放、掉落、堆叠、塔体倒塌、球体滚动以及道具与场地或机器人的碰撞。
夹爪接触仍需要合理的摩擦参数、接触求解设置，必要时还需要抓取约束插件，才能稳定复现真机抓取。

### 传感器与定位

可以加入 RGB 相机、深度相机、激光雷达、IMU、轮速计和关节编码器，并模拟视野、分辨率、
帧率、噪声和延迟。这样可以测试方块识别、Mustika 定位、建塔对准、场地定位、避障和状态估计，
而算法只接收模拟传感器数据，不直接读取 Gazebo 真值。

### 多机器人与对抗

Gazebo 可以同时运行双方 TR、BR，测试会车、路径冲突、共享区域抢占、机器人之间的物理碰撞，
以及通信延迟或消息丢失时的行为。为了重复对比策略，应给每次试验固定初始位姿、道具摆放和随机种子。

### 比赛流程和裁判逻辑

比赛计时、得分、Retry、越区、Transfer Area 交付判定、Building Spot 完整落位、机器人是否仍
接触道具、共享区阻挡计时和 Sanctuary Mandate 等规则不会由 Gazebo 自动理解，需要单独编写
ROS 2 裁判节点。裁判节点可以读取 Gazebo 真值和接触事件，形成可重复运行的整场比赛测试。

推荐的数据边界如下：

```text
Gazebo 真值与接触事件 ──> 裁判节点 ──> 得分、犯规、Retry、比赛状态
模拟传感器数据       ──> 机器人算法 ──> 控制指令 ──> Gazebo
Gazebo 与算法状态     ──> 监控界面
```

### 不应直接当作真机结论的内容

仿真结果会受到摩擦系数、轮胎模型、接触求解器、结构柔性和传感器噪声模型的影响。Gazebo 很适合
验证几何可达性、控制逻辑、任务流程和故障恢复，但抓取成功率、轮胎打滑、结构振动和高速碰撞仍需
通过真机数据标定并进行实物验证。

## 建议的开发顺序

1. 完成场地尺寸、碰撞和颜色校对；
2. 导入 TR、BR 的 URDF/SDF 并接入 `ros2_control`；
3. 加入相机、雷达、IMU 等模拟传感器；
4. 接入导航、机械臂和任务状态机；
5. 实现裁判、计分、Retry 和整场自动回归测试；
6. 使用真机测量数据标定摩擦、质量、惯量、噪声和延迟。
