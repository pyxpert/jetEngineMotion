import numpy as np
from robot_motion_planner import Robot, EngineSurface
import math

def validate_path_reasonableness(initial_front_pos, initial_rear_pos, path_positions, robot_params=None):
    """
    判断给定路径是否合理的函数
    
    参数:
    - initial_front_pos: 初始前足位置 (theta, z)
    - initial_rear_pos: 初始后足位置 (theta, z)
    - path_positions: 路径点列表 [(front_pos1, rear_pos1), (front_pos2, rear_pos2), ..., (front_pos_n, rear_pos_n)]
                     其中每个位置为 (theta, z) 形式
    - robot_params: 机器人参数字典，包含 d1, d2, h, s_max 等
    
    返回:
    - bool: 是否合理
    - str: 错误信息
    """
    
    # 默认机器人参数
    if robot_params is None:
        robot_params = {
            'd1': 20,      # 前足到中间电机的水平距离
            'd2': 20,      # 后足到中间电机的水平距离  
            'h': 30,       # 中心电机转轴到上表面的高度
            's_max': 8,    # 直线电机最大行程
            'R': 900       # 环境半径
        }
    
    # 创建机器人实例用于几何计算
    robot = Robot(
        d1=robot_params['d1'],
        d2=robot_params['d2'], 
        h=robot_params['h'],
        s_max=robot_params['s_max']
    )
    surface = EngineSurface(r_constant=robot_params['R'])
    
    # 验证初始位置
    if not validate_initial_positions(initial_front_pos, initial_rear_pos, robot, surface):
        return False, "初始位置不合理"
    
    # 验证路径连续性
    current_front_pos = initial_front_pos
    current_rear_pos = initial_rear_pos
    
    for i, (next_front_pos, next_rear_pos) in enumerate(path_positions):
        # 检查下一个位置是否满足几何约束
        is_valid, reason = validate_step_transition(
            current_front_pos, current_rear_pos, 
            next_front_pos, next_rear_pos, 
            robot, surface
        )
        
        if not is_valid:
            return False, f"第{i+1}步运动不合理: {reason}"
        
        # 更新当前位置
        current_front_pos = next_front_pos
        current_rear_pos = next_rear_pos
    
    return True, "路径合理"

def validate_initial_positions(front_pos, rear_pos, robot, surface):
    """
    验证初始位置是否合理
    """
    front_theta, front_z = front_pos
    rear_theta, rear_z = rear_pos
    
    # 检查是否在圆柱表面上
    if not is_on_cylinder_surface(front_theta, front_z, surface) or not is_on_cylinder_surface(rear_theta, rear_z, surface):
        return False
    
    # 检查两足间距是否在合理范围内
    distance = calculate_foot_distance(front_pos, rear_pos, surface.r_constant)
    max_distance = robot.d1 + robot.d2 + robot.s_max + 5  # 加一些容差
    
    if distance > max_distance:
        return False
    
    return True

def validate_step_transition(current_front_pos, current_rear_pos, next_front_pos, next_rear_pos, robot, surface):
    """
    验证从当前位置到下一位置的转换是否合理
    """
    curr_f_theta, curr_f_z = current_front_pos
    curr_r_theta, curr_r_z = current_rear_pos
    next_f_theta, next_f_z = next_front_pos
    next_r_theta, next_r_z = next_rear_pos
    
    # 计算运动方向和前进角φ
    # 根据蠕虫式爬行的特点，通常是一足固定，另一足移动
    delta_f_theta = next_f_theta - curr_f_theta
    delta_f_z = next_f_z - curr_f_z
    delta_r_theta = next_r_theta - curr_r_theta
    delta_r_z = next_r_z - curr_r_z
    
    # 检查是否符合蠕虫式爬行模式（一足固定，一足移动）
    front_stationary = np.sqrt(delta_f_theta**2 + (delta_f_z/surface.r_constant)**2) < 0.1  # 前足几乎不动
    rear_stationary = np.sqrt(delta_r_theta**2 + (delta_r_z/surface.r_constant)**2) < 0.1  # 后足几乎不动
    
    if not (front_stationary or rear_stationary):
        # 如果不是典型的蠕虫式爬行，检查是否满足几何约束
        return validate_general_movement(current_front_pos, current_rear_pos, next_front_pos, next_rear_pos, robot, surface)
    
    # 如果是典型的蠕虫式爬行模式
    if front_stationary:
        # 后足移动，前足固定
        # 验证后足的新位置是否在以前足为中心的可达圆内
        is_reachable, reason = is_position_reachable(next_rear_pos, current_front_pos, robot, surface, find_front=False)
        return is_reachable, reason
    elif rear_stationary:
        # 前足移动，后足固定
        # 验证前足的新位置是否在以后足为中心的可达圆内
        is_reachable, reason = is_position_reachable(next_front_pos, current_rear_pos, robot, surface, find_front=True)
        return is_reachable, reason
    
    return True, "运动合理"

def validate_general_movement(current_front_pos, current_rear_pos, next_front_pos, next_rear_pos, robot, surface):
    """
    验证一般性运动（非典型蠕虫式爬行）的合理性
    """
    # 检查前后足的最终间距是否在机器人机械限制内
    current_distance = calculate_foot_distance(current_front_pos, current_rear_pos, surface.r_constant)
    next_distance = calculate_foot_distance(next_front_pos, next_rear_pos, surface.r_constant)
    
    min_possible_distance = abs(robot.d1 - robot.d2) - robot.s_max  # 最小可能距离
    max_possible_distance = robot.d1 + robot.d2 + robot.s_max      # 最大可能距离
    
    if next_distance < min_possible_distance or next_distance > max_possible_distance:
        return False, f"目标间距超出机械限制 [{min_possible_distance}, {max_possible_distance}], 实际: {next_distance}"
    
    return True, "运动合理"

def is_position_reachable(target_pos, fixed_pos, robot, surface, find_front):
    """
    检查目标位置是否可通过椭圆-圆求交点算法到达
    """
    target_theta, target_z = target_pos
    fixed_theta, fixed_z = fixed_pos
    
    # 计算前进角φ
    if find_front:
        # 从前足角度看，计算到目标后足的方向
        delta_theta = target_theta - fixed_theta
        delta_z = target_z - fixed_z
    else:
        # 从后足角度看，计算到目标前足的方向
        delta_theta = target_theta - fixed_theta
        delta_z = target_z - fixed_z
    
    # 将直角坐标转换为适合椭圆-圆算法的坐标系统
    # 在机器人局部坐标系中，假设固定足在原点附近
    fixed_foot_X = fixed_z  # 轴向坐标
    fixed_foot_theta_rad = fixed_theta  # 周向角度
    
    target_X = target_z  # 目标轴向坐标
    target_theta_rad = target_theta  # 目标周向角度
    
    # 计算前进方向角φ
    dx = target_X - fixed_foot_X
    dy_circum = surface.r_constant * (target_theta_rad - fixed_foot_theta_rad)  # 周向弧长
    
    if abs(dx) < 1e-6 and abs(dy_circum) < 1e-6:
        return True, "位置不变"
    
    phi = np.arctan2(dy_circum, dx)  # 前进方向角
    
    # 计算直线电机行程a（近似）
    # 这里需要根据具体的几何关系反推
    distance_3d = calculate_foot_distance(target_pos, fixed_pos, surface.r_constant)
    
    # 简化的行程估算
    a_approx = max(0, min(robot.s_max, distance_3d - (robot.d1 + robot.d2) * 0.7))
    
    try:
        # 使用机器人的计算方法验证可达性
        # 这里模拟calc_other_foot_position的逆向验证
        delta_X, delta_theta = robot.calc_other_foot_position(
            fixed_foot_X, fixed_foot_theta_rad, phi, a_approx, surface.r_constant, find_front=find_front
        )
        
        # 计算预期的目标位置
        expected_X = fixed_foot_X + delta_X
        expected_theta = fixed_foot_theta_rad + delta_theta
        
        # 检查是否足够接近
        tolerance_pos = 5.0  # 位置容差(mm)
        tolerance_ang = 0.1  # 角度容差(rad)
        
        pos_match = abs(target_X - expected_X) <= tolerance_pos
        ang_match = abs(target_theta_rad - expected_theta) <= tolerance_ang
        
        if pos_match and ang_match:
            return True, "位置可达"
        else:
            return False, f"位置不可达: 期望({expected_X:.2f},{expected_theta:.3f}), 实际({target_X:.2f},{target_theta:.3f})"
    
    except Exception as e:
        return False, f"计算错误: {str(e)}"

def calculate_foot_distance(pos1, pos2, R):
    """
    计算两点间的距离（考虑圆柱表面）
    """
    theta1, z1 = pos1
    theta2, z2 = pos2
    
    # 计算周向弧长
    delta_theta = min(abs(theta2 - theta1), 2*np.pi - abs(theta2 - theta1))
    circum_dist = R * delta_theta
    
    # 计算3D欧几里得距离
    axial_dist = abs(z2 - z1)
    distance = np.sqrt(circum_dist**2 + axial_dist**2)
    
    return distance

def is_on_cylinder_surface(theta, z, surface):
    """
    检查点是否在圆柱表面上
    """
    # 检查z是否在范围内
    if z < surface.z_min or z > surface.z_max:
        return False
    
    return True

# 示例使用
if __name__ == "__main__":
    # 示例：测试路径合理性
    initial_front = (0.0, 100.0)  # (theta, z)
    initial_rear = (0.0, 50.0)    # (theta, z)
    
    # 路径：前足先移动到z=200，然后后足移动到z=150，等等
    path = [
        ((0.0, 200.0), (0.0, 50.0)),  # 前足移动
        ((0.0, 200.0), (0.0, 150.0)), # 后足移动
        ((0.0, 300.0), (0.0, 150.0)), # 前足移动
    ]
    
    is_reasonable, message = validate_path_reasonableness(initial_front, initial_rear, path)
    print(f"路径合理性: {is_reasonable}")
    print(f"说明: {message}")