import numpy as np
from robot_motion_planner import Robot, EngineSurface

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
            's_max': 8,    # 直线电机行程
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
        return False, "初始位置不合理：两足间距超出机械限制或不在圆柱表面"
    
    print(f"初始位置验证通过：前足{initial_front_pos}, 后足{initial_rear_pos}")
    
    # 验证路径中每一步的合理性
    prev_front_pos = initial_front_pos
    prev_rear_pos = initial_rear_pos
    
    for i, (next_front_pos, next_rear_pos) in enumerate(path_positions):
        print(f"\n验证第{i+1}步: 从前足{prev_front_pos},后足{prev_rear_pos}到前足{next_front_pos},后足{next_rear_pos}")
        
        # 检查每一步是否符合几何约束和运动学约束
        is_valid, reason = validate_single_step(
            prev_front_pos, prev_rear_pos, 
            next_front_pos, next_rear_pos,
            robot, surface
        )
        
        if not is_valid:
            return False, f"第{i+1}步运动不合理: {reason}"
        
        print(f"第{i+1}步验证通过: {reason}")
        
        # 更新当前位置
        prev_front_pos = next_front_pos
        prev_rear_pos = next_rear_pos
    
    return True, "路径完全合理"


def validate_initial_positions(front_pos, rear_pos, robot, surface):
    """
    验证初始位置是否合理
    """
    front_theta, front_z = front_pos
    rear_theta, rear_z = rear_pos
    
    # 检查两足在当前直线电机行程下的理论间距范围
    # 当前机器人配置下，两足间的距离受d1, d2, h和s_max的约束
    min_distance_theoretical = max(0, abs((robot.d1 + robot.h) - (robot.d2 + robot.h)) - robot.s_max)
    max_distance_theoretical = (robot.d1 + robot.h) + (robot.d2 + robot.h) + robot.s_max
    
    # 使用更现实的几何约束
    # 在圆柱表面，两足间最大距离约为它们在展开平面上的距离
    delta_theta = min(abs(front_theta - rear_theta), 2*np.pi - abs(front_theta - rear_theta))
    delta_z = abs(front_z - rear_z)
    
    # 使用展开平面的距离（更符合圆柱表面的实际距离）
    surface_distance = np.sqrt((delta_theta * surface.r_constant)**2 + delta_z**2)
    
    # 根据机器人结构，合理范围应该是
    min_distance = max(0, abs(robot.d1 - robot.d2) - robot.s_max - 10)  # 添加一些容差
    max_distance = robot.d1 + robot.d2 + robot.s_max + 2*robot.h + 10   # 添加一些容差
    
    if surface_distance < min_distance or surface_distance > max_distance:
        print(f"初始位置间距不合理: 实际表面距离={surface_distance:.2f}, 理论范围=[{min_distance:.2f}, {max_distance:.2f}]")
        return False
    
    return True


def validate_single_step(current_front_pos, current_rear_pos, next_front_pos, next_rear_pos, robot, surface):
    """
    验证单步运动的合理性
    """
    # 计算当前和目标状态下两足的表面距离
    current_distance = calculate_surface_distance(current_front_pos, current_rear_pos, surface.r_constant)
    next_distance = calculate_surface_distance(next_front_pos, next_rear_pos, surface.r_constant)
    
    # 检查目标间距是否在机械限制内
    min_possible_distance = max(0, abs(robot.d1 - robot.d2) - robot.s_max - 5)  # 加容差
    max_possible_distance = robot.d1 + robot.d2 + robot.s_max + 2*robot.h + 5  # 加容差
    
    if next_distance < min_possible_distance:
        return False, f"目标间距小于最小可能距离 [{min_possible_distance:.2f}, {max_possible_distance:.2f}], 实际: {next_distance:.2f}"
    
    if next_distance > max_possible_distance:
        return False, f"目标间距大于最大可能距离 [{min_possible_distance:.2f}, {max_possible_distance:.2f}], 实际: {next_distance:.2f}"
    
    # 分析运动模式：判断是前足移动还是后足移动
    delta_f = calculate_surface_distance(current_front_pos, next_front_pos, surface.r_constant)
    delta_r = calculate_surface_distance(current_rear_pos, next_rear_pos, surface.r_constant)
    
    print(f"  当前表面间距: {current_distance:.2f}, 目标表面间距: {next_distance:.2f}")
    print(f"  前足移动距离: {delta_f:.2f}, 后足移动距离: {delta_r:.2f}")
    print(f"  间距限制范围: [{min_possible_distance:.2f}, {max_possible_distance:.2f}]")
    
    # 蠕虫式爬行模式：通常是一足固定，另一足移动
    if delta_f < 1.0:  # 前足基本不动，主要是后足移动
        # 验证后足移动是否符合几何约束
        is_valid, reason = validate_foot_move_consistency(next_rear_pos, current_front_pos, current_rear_pos, robot, surface, moving_front=False)
        return is_valid, f"后足移动验证 - {reason}"
    elif delta_r < 1.0:  # 后足基本不动，主要是前足移动
        # 验证前足移动是否符合几何约束
        is_valid, reason = validate_foot_move_consistency(next_front_pos, current_front_pos, current_rear_pos, robot, surface, moving_front=True)
        return is_valid, f"前足移动验证 - {reason}"
    else:
        # 如果两足都在明显移动，验证总的几何约束是否满足
        return True, "双足协调移动（符合总体几何约束）"


def validate_foot_move_consistency(target_pos, current_front_pos, current_rear_pos, robot, surface, moving_front):
    """
    验证单足移动是否符合几何约束
    """
    if moving_front:
        # 前足移动，后足相对固定
        fixed_pos = current_rear_pos  # 后足视为固定
        moving_pos = target_pos       # 前足目标位置
        
        # 计算从后足看前足的运动参数
        fixed_theta, fixed_z = fixed_pos
        target_theta, target_z = moving_pos
        
        # 计算前进方向角φ
        delta_z = target_z - fixed_z
        avg_radius = surface.r_constant
        delta_theta = ((target_theta - fixed_theta + np.pi) % (2 * np.pi)) - np.pi  # 规范化到[-π, π]
        delta_circum = avg_radius * delta_theta  # 周向弧长
        
        if abs(delta_z) < 1e-6 and abs(delta_circum) < 1e-6:
            return True, "位置未发生变化"
        
        # 避免除零错误
        if abs(delta_z) < 1e-6:
            phi = np.pi/2 if delta_circum > 0 else -np.pi/2
        else:
            phi = np.arctan2(delta_circum, delta_z)  # 前进方向角
        
        # 计算目标距离
        target_distance = np.sqrt(delta_z**2 + delta_circum**2)
        
        # 尝试不同的a值找到匹配的解
        valid_a = None
        best_error = float('inf')
        
        for a_test in np.linspace(0, robot.s_max, 100):
            try:
                # 使用机器人的calc_other_foot_position计算预期位置
                delta_X, delta_theta_calc = robot.calc_other_foot_position(
                    fixed_z, fixed_theta, phi, a_test, surface.r_constant, find_front=True
                )
                
                expected_z = fixed_z + delta_X
                expected_theta = fixed_theta + delta_theta_calc
                expected_delta_z = expected_z - fixed_z
                expected_delta_circum = surface.r_constant * delta_theta_calc
                
                expected_distance = np.sqrt(expected_delta_z**2 + expected_delta_circum**2)
                
                # 检查是否接近目标位置
                pos_error = abs(target_distance - expected_distance)
                
                if pos_error < best_error and pos_error < 10.0:  # 位置容差10mm
                    best_error = pos_error
                    valid_a = a_test
            except:
                continue
        
        if valid_a is not None:
            return True, f"找到匹配的直线电机行程 a={valid_a:.2f}, 误差={best_error:.2f}mm"
        else:
            return False, f"无法通过调整直线电机行程达到目标位置，最佳匹配误差={best_error:.2f}mm"
    
    else:
        # 后足移动，前足相对固定
        fixed_pos = current_front_pos  # 前足视为固定
        moving_pos = target_pos       # 后足目标位置
        
        # 计算从前足看后足的运动参数
        fixed_theta, fixed_z = fixed_pos
        target_theta, target_z = moving_pos
        
        # 计算前进方向角φ
        delta_z = target_z - fixed_z
        avg_radius = surface.r_constant
        delta_theta = ((target_theta - fixed_theta + np.pi) % (2 * np.pi)) - np.pi  # 规范化到[-π, π]
        delta_circum = avg_radius * delta_theta  # 周向弧长
        
        if abs(delta_z) < 1e-6 and abs(delta_circum) < 1e-6:
            return True, "位置未发生变化"
        
        # 避免除零错误
        if abs(delta_z) < 1e-6:
            phi = np.pi/2 if delta_circum > 0 else -np.pi/2
        else:
            phi = np.arctan2(delta_circum, delta_z)  # 前进方向角
        
        # 计算目标距离
        target_distance = np.sqrt(delta_z**2 + delta_circum**2)
        
        # 尝试不同的a值找到匹配的解
        valid_a = None
        best_error = float('inf')
        
        for a_test in np.linspace(0, robot.s_max, 100):
            try:
                # 使用机器人的calc_other_foot_position计算预期位置
                delta_X, delta_theta_calc = robot.calc_other_foot_position(
                    fixed_z, fixed_theta, phi, a_test, surface.r_constant, find_front=False
                )
                
                expected_z = fixed_z + delta_X
                expected_theta = fixed_theta + delta_theta_calc
                expected_delta_z = expected_z - fixed_z
                expected_delta_circum = surface.r_constant * delta_theta_calc
                
                expected_distance = np.sqrt(expected_delta_z**2 + expected_delta_circum**2)
                
                # 检查是否接近目标位置
                pos_error = abs(target_distance - expected_distance)
                
                if pos_error < best_error and pos_error < 10.0:  # 位置容差10mm
                    best_error = pos_error
                    valid_a = a_test
            except:
                continue
        
        if valid_a is not None:
            return True, f"找到匹配的直线电机行程 a={valid_a:.2f}, 误差={best_error:.2f}mm"
        else:
            return False, f"无法通过调整直线电机行程达到目标位置，最佳匹配误差={best_error:.2f}mm"


def calculate_surface_distance(pos1, pos2, R):
    """
    计算圆柱表面两点间的距离
    """
    theta1, z1 = pos1
    theta2, z2 = pos2
    
    # 计算周向弧长（考虑角度跨越-π/π边界的情况）
    delta_theta_raw = theta2 - theta1
    delta_theta = ((delta_theta_raw + np.pi) % (2 * np.pi)) - np.pi  # 规范化到[-π, π]
    
    circum_dist = R * abs(delta_theta)
    
    # 计算表面距离
    axial_dist = abs(z2 - z1)
    distance = np.sqrt(circum_dist**2 + axial_dist**2)
    
    return distance


# 测试函数
def test_path_validation():
    print("="*60)
    print("路径合理性验证测试")
    print("="*60)
    
    # 测试案例1：小幅度移动的合理路径
    print("\n测试案例1：小幅度移动的合理路径")
    initial_front = (0.0, 100.0)  # (theta, z)
    initial_rear = (0.0, 50.0)    # (theta, z)
    
    path1 = [
        ((0.0, 150.0), (0.0, 50.0)),  # 前足移动到z=150 (移动50mm)
        ((0.0, 150.0), (0.0, 100.0)), # 后足移动到z=100 (移动50mm)
    ]
    
    is_reasonable1, message1 = validate_path_reasonableness(initial_front, initial_rear, path1)
    print(f"结果: {is_reasonable1}")
    print(f"说明: {message1}")
    
    # 测试案例2：包含小角度变化的路径
    print("\n测试案例2：包含小角度变化的路径")
    path2 = [
        ((0.1, 120.0), (0.0, 50.0)),  # 前足移动并小角度转向
    ]
    
    is_reasonable2, message2 = validate_path_reasonableness(initial_front, initial_rear, path2)
    print(f"结果: {is_reasonable2}")
    print(f"说明: {message2}")
    
    # 测试案例3：间距过大的不合理路径
    print("\n测试案例3：间距过大的不合理路径")
    path3 = [
        ((0.0, 500.0), (0.0, 50.0)),  # 前足移动太远，间距过大
    ]
    
    is_reasonable3, message3 = validate_path_reasonableness(initial_front, initial_rear, path3)
    print(f"结果: {is_reasonable3}")
    print(f"说明: {message3}")


if __name__ == "__main__":
    test_path_validation()