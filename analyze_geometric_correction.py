import numpy as np
import matplotlib.pyplot as plt

def visualize_geometric_relationship():
    """
    可视化几何关系，验证pyxpert提出的修正建议
    """
    # 机器人参数（来自requirements_log.txt）
    d1 = 20  # 前足到中间电机的水平距离
    d2 = 20  # 后足到中间电机的水平距离
    h = 30   # 中心电机转轴到上表面的高度
    a = 50   # 直线电机行程（示例值）
    R = 900  # 圆柱半径
    
    # 创建图形
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    
    # 情况1：后足吸附，前足将要落下（find_front=True）
    ax1.set_title('情况1：后足吸附，前足将要落下')
    
    # 固定后足位置（假设在底部）
    rear_foot_x = 0
    rear_foot_y = -R
    
    # 中间电机位置（相对于后足）
    motor_x = rear_foot_x + d2
    motor_y = rear_foot_y + h
    
    # 前足可能位置（以中间电机为中心，半径为√(d1²+h²)的圆）
    current_radius = np.sqrt(d1**2 + h**2)
    
    # 绘制几何关系
    ax1.plot(rear_foot_x, rear_foot_y, 'ro', markersize=8, label='后足（固定）')
    ax1.plot(motor_x, motor_y, 'bo', markersize=8, label='中间电机')
    
    # 绘制当前的小圆（错误的方法）
    current_circle = plt.Circle((motor_x, motor_y), d1 + h, fill=False, 
                               color='red', linestyle='--', label=f'当前小圆（半径={d1+h}）')
    ax1.add_patch(current_circle)
    
    # 绘制修正后的小圆（正确的方法）
    corrected_circle = plt.Circle((motor_x, motor_y), current_radius, fill=False, 
                                 color='green', linestyle='-', label=f'修正小圆（半径=√({d1}²+{h}²)={current_radius:.1f}）')
    ax1.add_patch(corrected_circle)
    
    # 绘制距离线
    ax1.plot([motor_x, motor_x + d1], [motor_y, motor_y], 'k-', alpha=0.5, label=f'水平距离d1={d1}')
    ax1.plot([motor_x, motor_x], [motor_y, motor_y - h], 'k--', alpha=0.5, label=f'垂直距离h={h}')
    
    ax1.set_xlim(motor_x - 100, motor_x + 100)
    ax1.set_ylim(motor_y - 100, motor_y + 50)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # 情况2：前足吸附，后足将要落下（find_front=False）
    ax2.set_title('情况2：前足吸附，后足将要落下')
    
    # 固定前足位置（假设在底部）
    front_foot_x = 0
    front_foot_y = -R
    
    # 中间电机位置（相对于前足）
    motor_x2 = front_foot_x - d1
    motor_y2 = front_foot_y + h
    
    # 后足可能位置（以中间电机为中心，半径为√((a+d2)²+h²)的圆）
    corrected_radius2 = np.sqrt((a + d2)**2 + h**2)
    
    # 绘制几何关系
    ax2.plot(front_foot_x, front_foot_y, 'go', markersize=8, label='前足（固定）')
    ax2.plot(motor_x2, motor_y2, 'bo', markersize=8, label='中间电机')
    
    # 绘制当前的小圆（错误的方法）
    current_circle2 = plt.Circle((motor_x2, motor_y2), a + d2 + h, fill=False, 
                                color='red', linestyle='--', label=f'当前小圆（半径={a+d2+h}）')
    ax2.add_patch(current_circle2)
    
    # 绘制修正后的小圆（正确的方法）
    corrected_circle2 = plt.Circle((motor_x2, motor_y2), corrected_radius2, fill=False, 
                                  color='green', linestyle='-', label=f'修正小圆（半径=√({a+d2}²+{h}²)={corrected_radius2:.1f}）')
    ax2.add_patch(corrected_circle2)
    
    # 绘制距离线
    ax2.plot([motor_x2, motor_x2 + a + d2], [motor_y2, motor_y2], 'k-', alpha=0.5, label=f'水平距离a+d2={a+d2}')
    ax2.plot([motor_x2, motor_x2], [motor_y2, motor_y2 - h], 'k--', alpha=0.5, label=f'垂直距离h={h}')
    
    ax2.set_xlim(motor_x2 - 150, motor_x2 + 150)
    ax2.set_ylim(motor_y2 - 100, motor_y2 + 50)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig('geometric_correction_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    # 输出分析结果
    print("几何关系分析结果：")
    print("="*50)
    print(f"情况1（后足固定，求前足）：")
    print(f"  当前方法：半径 = {d1 + h:.1f}")
    print(f"  修正方法：半径 = √({d1}²+{h}²) = {current_radius:.1f}")
    print(f"  差异：{abs(current_radius - (d1 + h)):.1f}")
    print()
    print(f"情况2（前足固定，求后足）：")
    print(f"  当前方法：半径 = {a + d2 + h:.1f}")
    print(f"  修正方法：半径 = √({a+d2}²+{h}²) = {corrected_radius2:.1f}")
    print(f"  差异：{abs(corrected_radius2 - (a + d2 + h)):.1f}")

if __name__ == "__main__":
    visualize_geometric_relationship()