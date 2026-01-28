"""
从规定起点到终点的直线蠕动 - 完整运动规划

- 叶片障碍物全部去掉（blade_positions=[]）
- 起点：前足 X=0, Y=-900, Z=40；后足 X=0, Y=-900, Z=0
- 目标终点：前足 X=0, Y=-900, Z=1000（前足到达此点即可）
- 直线蠕动沿 Z 轴正方向，周向角不变
"""

from robot_motion_planner import EngineSurface, Robot, MotionPlanner
import numpy as np


def main():
    print("=" * 70)
    print("直线蠕动运动规划：规定起点 -> 终点（无障碍物）")
    print("=" * 70)

    # 1. 表面模型：无叶片
    surface = EngineSurface(
        z_min=0,
        z_max=1000,
        r_constant=900.0,
        blade_positions=[],
    )
    print(f"\n表面: z=[0,1000] mm, R=900 mm, 叶片数={len(surface.blades)}")

    # 2. 机器人
    robot = Robot(
        L1=40, W1=30, L2=40, W2=30,
        h=30, s_max=8,
        magnet_radius=2.5,
        theta3_min=-90, theta3_max=90,
    )
    planner = MotionPlanner(robot, surface)

    # 3. 规定起点、终点（用户指定）
    # 起点：前足 (0, -900, 40)，后足 (0, -900, 0)
    # 目标：前足 (0, -900, 1000)
    front_start = (0.0, -900.0, 40.0)
    rear_start = (0.0, -900.0, 0.0)
    front_goal = (0.0, -900.0, 1000.0)

    print(f"\n起点: 前足 X=0, Y=-900, Z=40")
    print(f"      后足 X=0, Y=-900, Z=0")
    print(f"终点: 前足 X=0, Y=-900, Z=1000")
    print(f"直线蠕动: 沿 Z 轴正方向 40 -> 1000 mm")

    # 4. 规划
    step_size = 20.0
    plan = planner.plan_path_3d(front_start, rear_start, front_goal, step_size=step_size)

    print(f"\n规划步数: {len(plan)}")
    print("\n" + "-" * 70)
    print("完整运动规划过程（状态码 + 持续时间）")
    print("-" * 70)

    t = 0.0
    for i, (state, dur) in enumerate(plan):
        t += dur
        w1, w2, w3, v, m1, m2 = state
        desc = f"w1={w1} w2={w2} w3={w3} v={v} M1={m1} M2={m2}"
        print(f"  {i+1:4d}  {state}  {dur:7.3f}s  (t={t:7.3f}s)  {desc}")

    print("-" * 70)
    print(f"总时间: {t:.3f} s")

    # 5. 保存（含表格与按运动逻辑更新的 motion_plan_visualization.png）
    planner.save_plan("motion_plan.json")
    planner.save_plan_table("motion_plan_table.txt")
    print("\n已保存: motion_plan.json, motion_plan_table.txt")
    print("  （完整过程含每步 M1/M2 吸附位置见 motion_plan_table.txt）")

    print("\n" + "=" * 70)
    print("直线蠕动运动规划完成")
    print("=" * 70)


if __name__ == "__main__":
    main()
