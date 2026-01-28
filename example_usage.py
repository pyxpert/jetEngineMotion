"""
双节机器人运动规划系统使用示例
"""

from robot_motion_planner import EngineSurface, Robot, MotionPlanner, visualize_plan, visualize_robot_structure, visualize_robot_state
import matplotlib.pyplot as plt
import numpy as np

def main():
    print("=" * 60)
    print("双节机器人运动规划系统 - 使用示例")
    print("=" * 60)
    
    # 1. 创建航空发动机流道表面模型
    print("\n1. 创建表面模型...")
    surface = EngineSurface(
        z_min=0,          # 轴向起始位置 (mm)
        z_max=1000,       # 轴向结束位置 (mm) - 圆柱内壁总长度1000mm
        r_constant=900.0, # 内表面恒定半径 (mm)
        blade_positions=[]  # 去掉所有叶片障碍物
    )
    print(f"   表面范围: z=[{surface.z_min}, {surface.z_max}] mm")
    print(f"   圆柱内壁总长度: {surface.z_max - surface.z_min} mm")
    print(f"   内表面半径: {surface.r_constant} mm (恒定)")
    print(f"   叶片级数: {len(surface.blade_positions)} 级（无障碍物）")
    print(f"   叶片位置: {surface.blade_positions} mm")
    print(f"   叶片障碍物总数: {len(surface.blades)}")
    
    # 2. 创建机器人模型
    print("\n2. 创建机器人模型...")
    robot = Robot(
        L1=40,              # 前节长度 (mm)
        W1=30,              # 前节宽度 (mm)
        L2=40,              # 后节长度 (mm)
        W2=30,              # 后节宽度 (mm)
        h=30,               # 电磁铁底面到上表面高度 (mm)
        s_max=8,            # 直线电机最大行程 (mm)
        magnet_radius=2.5,  # 电磁铁半径 (mm)，直径5mm
        theta3_min=0,       # 两节相对旋转最小角度 (度)，0°为共面
        theta3_max=180      # 两节相对旋转最大角度 (度)，增大为内折
    )
    print(f"   前节尺寸: {robot.L1} x {robot.W1} mm")
    print(f"   后节尺寸: {robot.L2} x {robot.W2} mm")
    print(f"   高度 (h): {robot.h} mm")
    print(f"   直线电机最大行程: {robot.s_max} mm")
    print(f"   两节相对旋转范围: {np.degrees(robot.theta3_min):.0f}° 到 {np.degrees(robot.theta3_max):.0f}°")
    
    # 3. 创建运动规划器
    print("\n3. 创建运动规划器...")
    planner = MotionPlanner(robot, surface)
    
    # 4. 设置起点和终点（使用3D坐标）
    # 坐标系说明：
    # - X, Y: 径向坐标，机器人在圆柱内壁上
    # - Z: 轴向坐标，范围0-1000mm
    # - 角度θ：相对于X轴正方向，从Z轴正方向看去顺时针为负
    # 
    # 本次任务：机器人从-45°位置移动到270°位置（Y=-900），同时Z从40移动到1000
    # 起点位置：θ = -45°（即315°）
    #   X = R × cos(-45°) = 900 × 0.7071 ≈ 636.4 mm
    #   Y = R × sin(-45°) = 900 × (-0.7071) ≈ -636.4 mm
    # 终点位置：θ = 270°（即-90°）
    #   X = R × cos(270°) = 0 mm
    #   Y = R × sin(270°) = -900 mm
    # 
    # 机器人需要同时进行周向移动（从315°到270°，即-45°的周向变化）和轴向移动（从40到1000mm）
    
    print("\n4. 设置规划任务（3D坐标）...")
    R = surface.r_constant  # 圆柱内壁半径 900mm
    
    # 起点：270°位置（即Y=-900的位置）
    # pyxpert要求：起始位置在圆柱底部
    theta_fixed_deg = 270  
    theta_fixed_rad = np.radians(theta_fixed_deg)
    X_start = 0
    Y_start = -900
    
    # 终点：270°位置（即Y=-900的位置）
    # 这是requirements_log中指定的目标点
    X_goal = 0
    Y_goal = -900
    
    # 起点3D坐标
    front_start = (X_start, Y_start, 40)   # 前足起始点
    rear_start = (X_start, Y_start, 0)     # 后足起始点
    # 终点3D坐标
    front_goal = (X_goal, Y_goal, 1000)    # 前足终点：(0, -900, 1000)
    
    print(f"   前足起始点: X={front_start[0]}, Y={front_start[1]}, Z={front_start[2]} mm")
    print(f"   后足起始点: X={rear_start[0]}, Y={rear_start[1]}, Z={rear_start[2]} mm")
    print(f"   前足终点:   X={front_goal[0]}, Y={front_goal[1]}, Z={front_goal[2]} mm")
    print(f"   移动距离: {front_goal[2] - front_start[2]} mm (轴向)")
    
    # 转换为表面坐标用于规划
    # 确保 theta 严格一致
    start_theta = theta_fixed_rad
    goal_theta = theta_fixed_rad
    start_z = 40.0
    goal_z = 1000.0
    
    start = (start_theta, start_z)
    goal = (goal_theta, goal_z)
    
    print(f"\n   转换为表面坐标:")
    print(f"   起点: theta={start[0]:.4f} rad ({np.degrees(start[0]):.1f}°), z={start[1]:.1f} mm")
    print(f"   终点: theta={goal[0]:.4f} rad ({np.degrees(goal[0]):.1f}°), z={goal[1]:.1f} mm")
    
    # 检查起点和终点是否有效
    if not surface.is_valid_position(start[0], start[1]):
        print(f"   警告: 起点位置无效（可能在叶片上）")
    if not surface.is_valid_position(goal[0], goal[1]):
        print(f"   警告: 终点位置无效（可能在叶片上）")
    
    # 5. 执行运动规划
    print("\n5. 执行运动规划...")
    # 调整步长以适应1000mm的移动距离
    step_size = 20.0  # 路径规划步长 (mm)
    print(f"   步长: {step_size} mm")
    # 使用plan_path_3d函数，这样可以正确记录初始的3D坐标位置
    plan = planner.plan_path_3d(front_start, rear_start, front_goal, step_size=step_size)
    
    # 6. 显示规划结果
    print(f"\n6. 规划结果:")
    print(f"   总步骤数: {len(plan)}")
    print(f"\n   运动序列详情:")
    print(f"   {'步骤':<6} {'状态码':<8} {'持续时间(s)':<12} {'说明'}")
    print(f"   {'-'*6} {'-'*8} {'-'*12} {'-'*30}")
    
    total_time = 0
    for i, (state_code, duration) in enumerate(plan, 1):
        total_time += duration
        # 解析状态码
        w1, w2, w3, v, m1, m2 = state_code
        desc = f"ω1={w1}, ω2={w2}, ω3={w3}, v={v}, 前磁={m1}, 后磁={m2}"
        print(f"   {i:<6} {state_code:<8} {duration:<12.2f} {desc}")
    
    print(f"\n   总运动时间: {total_time:.2f} 秒")
    
    # 7. 保存规划结果
    print("\n7. 保存规划结果...")
    output_file_json = 'motion_plan.json'
    planner.save_plan(output_file_json)
    print(f"   JSON格式已保存到: {output_file_json}")
    
    output_file_table = 'motion_plan_table.txt'
    planner.save_plan_table(output_file_table)
    print(f"   表格格式已保存到: {output_file_table}")
    # save_plan_table 会根据运动逻辑自动更新 motion_plan_visualization.png
    
    # 8. 可视化机器人结构
    try:
        print("\n8. 生成机器人结构图...")
        fig_robot = visualize_robot_structure(robot)
        robot_structure_file = 'robot_structure.png'
        plt.savefig(robot_structure_file, dpi=150, bbox_inches='tight')
        print(f"   机器人结构图已保存到: {robot_structure_file}")
        plt.close(fig_robot)
        
        # 生成特定状态的机器人图像
        print("\n   生成特定状态的机器人图像...")
        
        # 状态0：θ3 = 0°, s = 0（初始状态）
        theta3_0 = 0.0
        s_0 = 0.0
        fig_state0 = visualize_robot_state(robot, theta3_0, s_0, 
                                          f"θ3 = 0°, s = {s_0:.1f}mm (Initial State)")
        state0_file = 'robot_state_theta3_0_s_0.png'
        plt.savefig(state0_file, dpi=150, bbox_inches='tight')
        print(f"   状态0图像已保存到: {state0_file}")
        plt.close(fig_state0)
        
        # 状态1：θ3 = +45°, s = 0
        theta3_1 = np.radians(45)
        s_1 = 0.0
        fig_state1 = visualize_robot_state(robot, theta3_1, s_1, 
                                          f"θ3 = +45°, s = {s_1:.1f}mm")
        state1_file = 'robot_state_theta3_45_s_0.png'
        plt.savefig(state1_file, dpi=150, bbox_inches='tight')
        print(f"   状态1图像已保存到: {state1_file}")
        plt.close(fig_state1)
        
        # 状态2：θ3 = -45°, s = s_max
        theta3_2 = np.radians(-45)
        s_2 = robot.s_max
        fig_state2 = visualize_robot_state(robot, theta3_2, s_2,
                                          f"θ3 = -45°, s = {s_2:.1f}mm")
        state2_file = 'robot_state_theta3_-45_s_max.png'
        plt.savefig(state2_file, dpi=150, bbox_inches='tight')
        print(f"   状态2图像已保存到: {state2_file}")
        plt.close(fig_state2)
        
    except Exception as e:
        print(f"   机器人结构图生成失败: {e}")
    
    # 9. 可视化运动规划（可选）
    try:
        print("\n9. 生成运动规划可视化...")
        print(f"   前足起始: theta={start[0]:.4f} rad, Z={front_start[2]:.1f} mm")
        print(f"   后足起始: theta={start[0]:.4f} rad, Z={rear_start[2]:.1f} mm")
        print(f"   前足终点: theta={goal[0]:.4f} rad, Z={front_goal[2]:.1f} mm")
        
        # 使用正确的前后足起始Z坐标和终点Z坐标
        fig = visualize_plan(planner, surface, start, goal,
                            front_start_z=front_start[2],  # 前足起始Z=40
                            rear_start_z=rear_start[2],    # 后足起始Z=0
                            front_goal_z=front_goal[2])    # 前足终点Z=1000
        viz_file = 'motion_plan_visualization.png'
        plt.savefig(viz_file, dpi=150, bbox_inches='tight')
        print(f"   可视化结果已保存到: {viz_file}")
        plt.close()  # 关闭图窗口
        # 如果需要交互式查看，取消下面两行的注释：
        # print("   正在显示交互式窗口，您可以通过鼠标旋转、缩放查看3D图像...")
        # plt.show()  # 显示交互式窗口，支持鼠标旋转查看
    except Exception as e:
        print(f"   可视化生成失败: {e}")
        import traceback
        traceback.print_exc()
        print("   这可能是因为系统没有图形界面支持")
    
    print("\n" + "=" * 60)
    print("规划完成！")
    print("=" * 60)


def show_initial_state_3d():
    """
    单独显示机器人在初始状态（θ3=0°, s=0mm）的三维立体图
    使用窗口形式弹出，不保存到文件
    """
    print("=" * 60)
    print("显示机器人初始状态三维立体图")
    print("=" * 60)
    
    # 创建机器人模型（使用与主程序相同的参数）
    robot = Robot(
        L1=40,              # 前节长度 (mm)
        W1=30,              # 前节宽度 (mm)
        L2=40,              # 后节长度 (mm)
        W2=30,              # 后节宽度 (mm)
        h=30,               # 电磁铁底面到上表面高度 (mm)
        s_max=8,            # 直线电机最大行程 (mm)
        magnet_radius=2.5,  # 电磁铁半径 (mm)，直径5mm
        theta3_min=0,       # 两节相对旋转最小角度 (度)，0°为共面
        theta3_max=180      # 两节相对旋转最大角度 (度)，增大为内折
    )
    
    # 初始状态：θ3 = 0°, s = 0mm
    # 初始位置：前足(0, -900, 40)，后足(0, -900, 0)
    theta3_0 = 0.0
    s_0 = 0.0
    
    print(f"\n生成初始状态三维图...")
    print(f"   参数: θ3 = {np.degrees(theta3_0):.0f}°, s = {s_0:.1f}mm")
    print(f"   初始位置: 前足(0, -900, 40), 后足(0, -900, 0)")
    print(f"   正在显示交互式窗口，您可以通过鼠标旋转、缩放查看3D图像...")
    print(f"   关闭窗口后程序将退出")
    
    # 生成并显示3D图（不保存）
    fig = visualize_robot_state(robot, theta3_0, s_0, 
                                 f"Initial State: θ3 = 0°, s = {s_0:.1f}mm")
    plt.show()  # 弹出窗口显示，不保存到文件


if __name__ == "__main__":
    import sys
    # 如果命令行参数包含 '--show-initial-3d'，则只显示初始状态3D图
    if len(sys.argv) > 1 and sys.argv[1] == '--show-initial-3d':
        show_initial_state_3d()
    else:
        main()
