"""
单独显示机器人在不同状态的三维立体图
使用窗口形式弹出，不保存到文件
"""

from robot_motion_planner import Robot, visualize_robot_state
import matplotlib.pyplot as plt
import numpy as np

def create_robot():
    """创建机器人模型（使用标准参数）"""
    return Robot(
        L1=40,            # 前节长度 (mm)
        W1=30,            # 前节宽度 (mm)
        L2=40,            # 后节长度 (mm)
        W2=30,            # 后节宽度 (mm)
        h=30,             # 电磁铁底面到上表面高度 (mm)
        s_max=8,          # 直线电机最大行程 (mm)
        magnet_radius=2.5,  # 电磁铁半径 (mm)，直径5mm
        theta3_min=-90,   # 两节相对旋转最小角度 (度)
        theta3_max=90     # 两节相对旋转最大角度 (度)
    )

def show_initial_state():
    """显示初始状态（θ3=0°, s=0mm）"""
    print("=" * 60)
    print("显示机器人初始状态三维立体图")
    print("=" * 60)
    
    robot = create_robot()
    theta3_0 = 0.0
    s_0 = 0.0
    
    print(f"\n生成初始状态三维图...")
    print(f"   参数: θ3 = {np.degrees(theta3_0):.0f}°, s = {s_0:.1f}mm")
    print(f"   正在显示交互式窗口，您可以通过鼠标旋转、缩放查看3D图像...")
    print(f"   关闭窗口后程序将退出")
    
    fig = visualize_robot_state(robot, theta3_0, s_0, 
                                 f"Initial State: θ3 = 0°, s = {s_0:.1f}mm")
    plt.show()

def show_s_max_state():
    """显示丝杠电机行程为最大值（s=s_max）的状态"""
    print("=" * 60)
    print("显示机器人丝杠电机行程为最大值状态三维立体图")
    print("=" * 60)
    
    robot = create_robot()
    theta3_smax = 0.0
    s_smax = robot.s_max
    
    print(f"\n生成丝杠电机最大行程状态三维图...")
    print(f"   参数: θ3 = {np.degrees(theta3_smax):.0f}°, s = {s_smax:.1f}mm (最大值)")
    print(f"   正在显示交互式窗口，您可以通过鼠标旋转、缩放查看3D图像...")
    print(f"   关闭窗口后程序将退出")
    
    fig = visualize_robot_state(robot, theta3_smax, s_smax, 
                                 f"Linear Motor s = s_max: θ3 = 0°, s = {s_smax:.1f}mm")
    plt.show()

def show_theta3_90_state():
    """显示中间旋转电机角度为90°（θ3=90°）的状态"""
    print("=" * 60)
    print("显示机器人中间旋转电机角度为90°状态三维立体图")
    print("=" * 60)
    
    robot = create_robot()
    theta3_90 = np.radians(90)
    s_90 = 0.0
    
    print(f"\n生成中间旋转电机90°状态三维图...")
    print(f"   参数: θ3 = {np.degrees(theta3_90):.0f}°, s = {s_90:.1f}mm")
    print(f"   正在显示交互式窗口，您可以通过鼠标旋转、缩放查看3D图像...")
    print(f"   关闭窗口后程序将退出")
    
    fig = visualize_robot_state(robot, theta3_90, s_90, 
                                 f"Rotation Motor θ3 = 90°: θ3 = {np.degrees(theta3_90):.0f}°, s = {s_90:.1f}mm")
    plt.show()

def main():
    import sys
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        if mode == '--initial':
            show_initial_state()
        elif mode == '--s-max':
            show_s_max_state()
        elif mode == '--theta3-90':
            show_theta3_90_state()
        else:
            print(f"未知模式: {mode}")
            print("可用模式: --initial, --s-max, --theta3-90")
    else:
        # 默认显示初始状态
        show_initial_state()


if __name__ == "__main__":
    main()
