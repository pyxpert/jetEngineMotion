# 双节机器人运动规划系统

## 系统概述

本系统用于在航空发动机/燃气轮机内腔表面进行双节机器人的运动规划。

## 机器人结构

- **前后两节**：每节上表面为长方形
- **三台旋转步进电机**：
  - ω1: 前节中心旋转电机（控制前节电磁铁相对于前节的角度）
  - ω2: 后节中心旋转电机（控制后节电磁铁相对于后节的角度）
  - ω3: 两节间旋转电机（控制两节相对角度）
- **一台丝杠电机**：v（控制两节间相对位移s）
- **两个电磁铁**：前节和后节各一个，可独立控制吸附/脱附

## 状态码格式

运动状态用六位数字表示：`ω1 ω2 ω3 v m1 m2`

- **ω1, ω2, ω3, v**: 
  - `0`: 停止
  - `1`: 正转
  - `2`: 反转
- **m1, m2** (电磁铁):
  - `0`: 脱附
  - `1`: 吸附

## 运动策略

采用**交替步进**策略：
1. 前节吸附，后节脱附 → 移动后节
2. 后节吸附，前节脱附 → 移动前节
3. 重复上述步骤

## 使用方法

```python
from robot_motion_planner import EngineSurface, Robot, MotionPlanner

# 创建表面模型
surface = EngineSurface(z_min=0, z_max=100, r_base=50, r_variation=10)

# 创建机器人
robot = Robot(L1=10, W1=6, L2=10, W2=6, h=2, s_max=15)

# 创建规划器
planner = MotionPlanner(robot, surface)

# 设置起点和终点（表面坐标：theta, z）
start = (0.5, 20.0)
goal = (3.0, 80.0)

# 执行规划
plan = planner.plan_path(start, goal, step_size=2.0)

# 查看规划结果
for i, (state, duration) in enumerate(plan, 1):
    print(f"步骤 {i}: 状态={state}, 持续时间={duration:.2f}s")

# 保存规划结果
planner.save_plan('motion_plan.json')
```

## 输出格式

规划结果以JSON格式保存：
```json
[
  {
    "state": "000011",
    "duration": 0.2
  },
  {
    "state": "010011",
    "duration": 1.5
  },
  ...
]
```

## 参数说明

### EngineSurface参数
- `z_min`, `z_max`: 轴向范围
- `r_base`: 基础半径
- `r_variation`: 半径变化幅度

### Robot参数
- `L1, W1`: 前节长宽
- `L2, W2`: 后节长宽
- `h`: 上表面到电磁铁表面高度
- `s_max`: 丝杠最大行程
- `magnet_radius`: 电磁铁半径

### MotionPlanner参数
- `step_size`: 路径规划步长
