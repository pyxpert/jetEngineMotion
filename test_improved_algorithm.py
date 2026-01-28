import sys
sys.path.append('.')

from robot_motion_planner import MotionPlanner, Robot, EngineSurface
import numpy as np

def test_improved_algorithm():
    """
    测试改进后的算法
    """
    print("测试改进后的椭圆-圆求交点算法")
    print("="*60)
    
    # 创建机器人和表面实例
    robot = Robot()
    surface = EngineSurface()
    
    # 直接使用机器人实例进行测试
    # 测试不同角度下的计算
    test_cases = [
        # (fixed_foot_X, fixed_foot_theta, phi, a, R, find_front, description)
        (0.0, -np.pi/2, np.pi/4, 50.0, 900.0, True, "φ=45°，前足定位"),
        (0.0, -np.pi/2, np.pi/6, 40.0, 900.0, True, "φ=30°，前足定位"),
        (0.0, -np.pi/2, np.pi/3, 50.0, 900.0, True, "φ=60°，前足定位"),
        (0.0, -np.pi/2, 0.01, 50.0, 900.0, True, "φ≈0°，近似纯轴向"),
    ]
    
    for i, (fx, ftheta, phi, a, R, find_front, desc) in enumerate(test_cases):
        print(f"\n测试案例 {i+1}: {desc}")
        print(f"固定足: X={fx}, θ={ftheta:.3f}rad ({np.degrees(ftheta):.1f}°)")
        print(f"前进角 φ={phi:.3f}rad ({np.degrees(phi):.1f}°), 行程a={a}, 半径R={R}")
        
        try:
            delta_X, delta_theta = robot.calc_other_foot_position(fx, ftheta, phi, a, R, find_front)
            print(f"计算结果: ΔX={delta_X:.6f}, Δθ={delta_theta:.6f}rad ({np.degrees(delta_theta):.3f}°)")
            
            # 计算另一足的绝对位置
            other_X = fx + delta_X
            other_theta = ftheta + delta_theta
            print(f"另一足位置: X={other_X:.6f}, θ={other_theta:.6f}rad ({np.degrees(other_theta):.3f}°)")
            
        except Exception as e:
            print(f"计算出错: {e}")
    
    print("\n" + "="*60)
    print("测试完成")


if __name__ == "__main__":
    test_improved_algorithm()