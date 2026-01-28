import numpy as np
import matplotlib.pyplot as plt

def plot_robot_geometry_model():
    """
    绘制机器人的几何简化模型及椭圆/小圆求交点算法的几何关系
    """
    # 机器人参数
    d1 = 20  # 前足到中间电机的水平距离
    d2 = 20  # 后足到中间电机的水平距离
    h = 30   # 中心电机转轴到上表面的高度
    a = 50   # 直线电机行程（示例值）
    R = 900  # 圆柱半径
    
    # 创建图形
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    
    # 子图1：机器人几何简化模型
    ax1 = axes[0, 0]
    ax1.set_title('机器人几何简化模型\n(Robot Geometric Simplified Model)')
    
    # 绘制圆柱壁面（示意）
    ax1.add_patch(plt.Rectangle((-100, -R-20), 200, 20, fill=True, color='lightgray', alpha=0.5, label='圆柱壁面'))
    
    # 绘制机器人结构
    # 中间电机位置
    motor_x, motor_y = 0, -R + h
    ax1.plot(motor_x, motor_y, 'bo', markersize=8, label='中间电机 (Middle Motor)')
    
    # 前足位置（当a=0时）
    front_foot_x = motor_x - d1
    front_foot_y = motor_y
    ax1.plot(front_foot_x, front_foot_y, 'ro', markersize=8, label='前足 (Front Foot)')
    
    # 后足位置（当a=0时）
    rear_foot_x = motor_x + d2
    rear_foot_y = motor_y
    ax1.plot(rear_foot_x, rear_foot_y, 'go', markersize=8, label='后足 (Rear Foot)')
    
    # 当直线电机行程为a时的后足位置
    rear_foot_extended_x = motor_x + d2 + a
    ax1.plot(rear_foot_extended_x, rear_foot_y, 'mo', markersize=8, label='后足(延伸后) (Rear Foot Extended)')
    
    # 连接线
    ax1.plot([front_foot_x, motor_x], [front_foot_y, motor_y], 'r-', linewidth=2, label='d1连接')
    ax1.plot([rear_foot_x, motor_x], [rear_foot_y, motor_y], 'g-', linewidth=2, label='d2连接')
    ax1.plot([rear_foot_x, rear_foot_extended_x], [rear_foot_y, rear_foot_y], 'm--', linewidth=2, label='a行程 (Stroke a)')
    
    # 高度标注
    ax1.annotate(f'h={h}', xy=(motor_x+5, motor_y), xytext=(motor_x+5, -R), 
                arrowprops=dict(arrowstyle='<->', color='blue'), fontsize=10, color='blue')
    
    ax1.set_xlim(-100, 100)
    ax1.set_ylim(-R-50, -R+h+20)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper left', fontsize=8)
    ax1.invert_yaxis()  # 使y轴正方向向下
    
    # 子图2：椭圆方程示意图
    ax2 = axes[0, 1]
    ax2.set_title('椭圆约束方程\n(Ellipse Constraint Equation)')
    
    # 椭圆参数（以φ=45°为例）
    phi = np.pi/4  # 45度
    a_ellipse = abs(R / np.sin(phi)) if abs(np.sin(phi)) >= 0.01 else R/0.01
    b_ellipse = R
    
    # 绘制椭圆
    theta_ellipse = np.linspace(0, 2*np.pi, 1000)
    x_ellipse = a_ellipse * np.cos(theta_ellipse)
    y_ellipse = b_ellipse * np.sin(theta_ellipse)
    
    ax2.plot(x_ellipse, y_ellipse, 'purple', linewidth=2, label=f'椭圆: y²/{R}² + x²/{a_ellipse:.0f}² = 1')
    
    # 标注椭圆参数
    ax2.annotate(f'a_ellipse={a_ellipse:.1f}', xy=(a_ellipse, 0), xytext=(a_ellipse*0.7, 20),
                arrowprops=dict(arrowstyle='->', color='purple'), fontsize=10, color='purple')
    ax2.annotate(f'b_ellipse={b_ellipse}', xy=(0, b_ellipse), xytext=(20, b_ellipse*0.7),
                arrowprops=dict(arrowstyle='->', color='purple'), fontsize=10, color='purple')
    
    ax2.set_xlim(-a_ellipse*1.1, a_ellipse*1.1)
    ax2.set_ylim(-b_ellipse*1.1, b_ellipse*1.1)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_xlabel('X轴 (Axial Direction)')
    ax2.set_ylabel('Y轴 (Radial Direction)')
    
    # 子图3：小圆半径计算（后足固定，求前足）
    ax3 = axes[1, 0]
    ax3.set_title('小圆半径计算 - 后足固定，求前足\n(Small Circle - Rear Fixed, Find Front)')
    
    # 固定后足位置
    rear_fixed_x, rear_fixed_y = 0, -R
    motor_pos_x, motor_pos_y = rear_fixed_x + d2, rear_fixed_y + h
    
    # 小圆圆心和半径
    circle_center_x, circle_center_y = motor_pos_x, motor_pos_y
    circle_radius = np.sqrt(d1**2 + h**2)
    
    # 绘制几何元素
    ax3.plot(rear_fixed_x, rear_fixed_y, 'go', markersize=8, label='固定后足 (Fixed Rear Foot)')
    ax3.plot(circle_center_x, circle_center_y, 'bo', markersize=8, label='电机中心 (Motor Center)')
    
    # 绘制小圆
    circle = plt.Circle((circle_center_x, circle_center_y), circle_radius, 
                       fill=False, color='orange', linewidth=2, label=f'小圆 (半径={circle_radius:.1f})')
    ax3.add_patch(circle)
    
    # 绘制距离线
    ax3.plot([circle_center_x, circle_center_x-d1], [circle_center_y, circle_center_y], 
             'k-', linewidth=1, alpha=0.7)
    ax3.plot([circle_center_x, circle_center_x], [circle_center_y, circle_center_y-h], 
             'k-', linewidth=1, alpha=0.7)
    ax3.plot([circle_center_x-d1, circle_center_x], [circle_center_y-h, circle_center_y], 
             'r-', linewidth=2, alpha=0.8, label=f'实际距离={circle_radius:.1f}')
    
    # 标注
    ax3.annotate(f'd1={d1}', xy=(circle_center_x, circle_center_y), 
                xytext=(circle_center_x-d1/2, circle_center_y+5), fontsize=9)
    ax3.annotate(f'h={h}', xy=(circle_center_x, circle_center_y), 
                xytext=(circle_center_x+5, circle_center_y-h/2), fontsize=9)
    
    ax3.set_xlim(circle_center_x - circle_radius - 20, circle_center_x + circle_radius + 20)
    ax3.set_ylim(circle_center_y - circle_radius - 20, circle_center_y + circle_radius + 20)
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=8)
    
    # 子图4：小圆半径计算（前足固定，求后足）
    ax4 = axes[1, 1]
    ax4.set_title('小圆半径计算 - 前足固定，求后足\n(Small Circle - Front Fixed, Find Rear)')
    
    # 固定前足位置
    front_fixed_x, front_fixed_y = 0, -R
    motor_pos_x2, motor_pos_y2 = front_fixed_x - d1, front_fixed_y + h
    
    # 小圆圆心和半径
    circle_center_x2, circle_center_y2 = motor_pos_x2, motor_pos_y2
    circle_radius2 = np.sqrt((a + d2)**2 + h**2)
    
    # 绘制几何元素
    ax4.plot(front_fixed_x, front_fixed_y, 'ro', markersize=8, label='固定前足 (Fixed Front Foot)')
    ax4.plot(circle_center_x2, circle_center_y2, 'bo', markersize=8, label='电机中心 (Motor Center)')
    
    # 绘制小圆
    circle2 = plt.Circle((circle_center_x2, circle_center_y2), circle_radius2, 
                        fill=False, color='orange', linewidth=2, label=f'小圆 (半径={circle_radius2:.1f})')
    ax4.add_patch(circle2)
    
    # 绘制距离线
    ax4.plot([circle_center_x2, circle_center_x2+(a+d2)], [circle_center_y2, circle_center_y2], 
             'k-', linewidth=1, alpha=0.7)
    ax4.plot([circle_center_x2, circle_center_x2], [circle_center_y2, circle_center_y2-h], 
             'k-', linewidth=1, alpha=0.7)
    ax4.plot([circle_center_x2+(a+d2), circle_center_x2], [circle_center_y2-h, circle_center_y2], 
             'r-', linewidth=2, alpha=0.8, label=f'实际距离={circle_radius2:.1f}')
    
    # 标注
    ax4.annotate(f'a+d2={a+d2}', xy=(circle_center_x2, circle_center_y2), 
                 xytext=(circle_center_x2+(a+d2)/2, circle_center_y2+5), fontsize=9)
    ax4.annotate(f'h={h}', xy=(circle_center_x2, circle_center_y2), 
                 xytext=(circle_center_x2-15, circle_center_y2-h/2), fontsize=9)
    
    ax4.set_xlim(circle_center_x2 - circle_radius2 - 20, circle_center_x2 + circle_radius2 + 20)
    ax4.set_ylim(circle_center_y2 - circle_radius2 - 20, circle_center_y2 + circle_radius2 + 20)
    ax4.set_aspect('equal')
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=8)
    
    plt.tight_layout()
    plt.savefig('robot_geometry_detailed.png', dpi=150, bbox_inches='tight')
    plt.show()

def explain_physical_quantities():
    """
    详细解释物理量的含义
    """
    print("="*80)
    print("机器人运动规划中椭圆-圆求交点算法的物理量详解")
    print("="*80)
    print()
    print("【机器人结构参数】")
    print("d1: 前足吸盘中心到中间电机转轴的水平距离（当a=0时）")
    print("d2: 后足吸盘中心到中间电机转轴的水平距离（当a=0时）")
    print("h: 中心电机转轴到上表面（水平面）的高度")
    print("a: 直线电机行程，范围[0, s_max]，控制两足间的距离变化")
    print("φ: 机器人前进方向角（机器人所在平面与环境轴线的角度）")
    print("R: 圆柱形环境的内表面半径")
    print()
    print("【椭圆方程参数】")
    print("椭圆方程: y²/R² + x²/(R/sin(φ))² = 1")
    print("a_ellipse = |R/sin(φ)|: 椭圆在x方向的半轴长度")
    print("b_ellipse = R: 椭圆在y方向的半轴长度")
    print("椭圆代表了机器人运动的几何约束，φ角决定了椭圆的形状")
    print()
    print("【小圆方程参数】")
    print("cx, cy: 小圆的圆心坐标")
    print("r_circle: 小圆的半径，根据情况不同分别为：")
    print("  - 寻找前足时（后足固定）：r_circle = √(d1² + h²)")
    print("  - 寻找后足时（前足固定）：r_circle = √((a+d2)² + h²)")
    print("小圆代表了另一足相对于固定足的可能位置范围")
    print()
    print("【几何意义】")
    print("椭圆-圆求交点算法本质上是在寻找满足运动学约束的足部位置：")
    print("1. 椭圆约束：机器人整体运动的几何限制")
    print("2. 小圆约束：活动足相对于固定足的可达范围")
    print("3. 交点：同时满足两种约束的位置，即为可行的足部落点")
    print("="*80)

if __name__ == "__main__":
    plot_robot_geometry_model()
    explain_physical_quantities()