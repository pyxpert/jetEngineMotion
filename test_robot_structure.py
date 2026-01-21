"""
测试机器人结构可视化
"""

from robot_motion_planner import Robot, visualize_robot_structure
import matplotlib.pyplot as plt
import numpy as np

def main():
    print("生成机器人结构图...")
    
    # 创建机器人（新参数）
    robot = Robot(
        L1=40,            # 前节长度 (mm)
        W1=30,            # 前节宽度 (mm)
        L2=40,            # 后节长度 (mm)
        W2=30,            # 后节宽度 (mm)
        h=30,             # 电磁铁底面到上表面高度 (mm)
        s_max=8,          # 直线电机最大行程 (mm)
        magnet_radius=2,  # 电磁铁半径 (mm)
        theta3_min=-90,   # 两节相对旋转最小角度 (度)
        theta3_max=90     # 两节相对旋转最大角度 (度)
    )
    
    print(f"前节尺寸: {robot.L1} x {robot.W1} mm")
    print(f"后节尺寸: {robot.L2} x {robot.W2} mm")
    print(f"高度: {robot.h} mm")
    print(f"直线电机最大行程: {robot.s_max} mm")
    print(f"θ3范围: {robot.theta3_min*180/np.pi:.0f}° 到 {robot.theta3_max*180/np.pi:.0f}°")
    
    # 生成结构图
    fig = visualize_robot_structure(robot)
    plt.savefig('robot_structure.png', dpi=150, bbox_inches='tight')
    print("\n机器人结构图已保存到 robot_structure.png")
    plt.show()

if __name__ == "__main__":
    main()
