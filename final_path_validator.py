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
    if not validate_initial_positions(initial_front_pos, initial_rear_pos, robot):
        return False, "初始位置不合理：两足间距超出机械限制"
    
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


def validate_initial_positions(front_pos, rear_pos, robot):
    """
    验证初始位置是否合理
    根据机器人机械约束，两足间的轴向距离应在[d1+d2, d1+s_max+d2]范围内
    """
    front_theta, front_z = front_pos
    rear_theta, rear_z = rear_pos
    
    # 对于初始位置，主要检查轴向距离是否在合理范围内
    axial_distance = abs(front_z - rear_z)
    
    min_axial_distance = robot.d1 + robot.d2  # 直线电机行程为0时
    max_axial_distance = robot.d1 + robot.d2 + robot.s_max  # 直线电机行程最大时
    
    if axial_distance < min_axial_distance or axial_distance > max_axial_distance:
        print(f"初始位置轴向间距不合理: 实际={axial_distance:.2f}, 范围=[{min_axial_distance:.2f}, {max_axial_distance:.2f}]")
        return False
    
    return True


def validate_single_step(current_front_pos, current_rear_pos, next_front_pos, next_rear_pos, robot, surface):
    """
    验证单步运动的合理性
    """
    curr_f_theta, curr_f_z = current_front_pos
    curr_r_theta, curr_r_z = current_rear_pos
    next_f_theta, next_f_z = next_front_pos
    next_r_theta, next_r_z = next_rear_pos
    
    # 检查目标状态的轴向间距是否在合理范围内
    next_axial_distance = abs(next_f_z - next_r_z)
    min_axial_distance = robot.d1 + robot.d2
    max_axial_distance = robot.d1 + robot.d2 + robot.s_max
    
    if next_axial_distance < min_axial_distance or next_axial_distance > max_axial_distance:
        return False, f"目标轴向间距超出机械限制 [{min_axial_distance:.2f}, {max_axial_distance:.2f}], 实际: {next_axial_distance:.2f}"
    
    # 分析运动模式：判断是前足移动还是后足移动
    delta_f_z = abs(next_f_z - curr_f_z)
    delta_r_z = abs(next_r_z - curr_r_z)
    
    print(f"  当前轴向间距: {abs(curr_f_z - curr_r_z):.2f}, 目标轴向间距: {next_axial_distance:.2f}")
    print(f"  前足轴向移动: {delta_f_z:.2f}mm, 后足轴向移动: {delta_r_z:.2f}mm")
    print(f"  轴向间距限制范围: [{min_axial_distance:.2f}, {max_axial_distance:.2f}]")
    
    # 蠕虫式爬行模式：通常是一足固定，另一足移动
    if delta_f_z < 1.0:  # 前足基本不动，主要是后足移动
        # 验证后足移动是否符合几何约束（从前足角度看）
        is_valid, reason = validate_foot_move_consistency(next_rear_pos, current_front_pos, current_rear_pos, robot, surface, moving_front=False)
        return is_valid, f"后足移动验证 - {reason}"
    elif delta_r_z < 1.0:  # 后足基本不动，主要是前足移动
        # 验证前足移动是否符合几何约束（从后足角度看）
        is_valid, reason = validate_foot_move_consistency(next_front_pos, current_front_pos, current_rear_pos, robot, surface, moving_front=True)
        return is_valid, f"前足移动验证 - {reason}"
    else:
        # 如果两足都在移动，检查是否符合协调运动约束
        return True, "双足协调移动（符合轴向间距约束）"


def validate_foot_move_consistency(target_pos, current_front_pos, current_rear_pos, robot, surface, moving_front):
    """
    验证单足移动是否符合几何约束
    """
    if moving_front:
        # 前足移动，后足相对固定
        fixed_pos = current_rear_pos  # 后足视为固定
        moving_pos = target_pos       # 前足目标位置
        
        # 从前足角度看后足位置，验证前足移动的可达性
        fixed_theta, fixed_z = fixed_pos
        target_theta, target_z = moving_pos
        
        # 计算从固定后足到目标前足的轴向距离
        target_axial_dist = abs(target_z - fixed_z)
        
        # 检查轴向距离是否在机器人可达范围内
        min_dist = robot.d1 + robot.d2  # a=0时
        max_dist = robot.d1 + robot.d2 + robot.s_max  # a=s_max时
        
        if target_axial_dist < min_dist or target_axial_dist > max_dist:
            return False, f"轴向距离超出可达范围 [{min_dist:.2f}, {max_dist:.2f}], 目标距离: {target_axial_dist:.2f}"
        
        # 尝试通过椭圆-圆求交点算法验证可达性
        # 计算前进角
        delta_z = target_z - fixed_z
        delta_theta_circum = surface.r_constant * ((target_theta - fixed_theta + np.pi) % (2 * np.pi) - np.pi)
        
        if abs(delta_z) < 1e-6:
            phi = np.pi/2 if delta_theta_circum > 0 else -np.pi/2
        else:
            phi = np.arctan2(delta_theta_circum, delta_z)
        
        # 尝试不同的直线电机行程a值，找到能够达到目标位置的值
        for a_test in np.linspace(0, robot.s_max, 50):
            try:
                # 使用机器人算法计算另一足位置
                delta_X, delta_theta = robot.calc_other_foot_position(
                    fixed_z, fixed_theta, phi, a_test, surface.r_constant, find_front=True
                )
                
                expected_z = fixed_z + delta_X
                expected_theta = fixed_theta + delta_theta
                
                # 检查是否接近目标位置
                z_error = abs(target_z - expected_z)
                theta_error = abs(target_theta - expected_theta)
                
                if z_error < 10.0 and theta_error < 0.1:  # 位置容差
                    return True, f"找到可达路径，直线电机行程 a={a_test:.2f}"
            except:
                continue
        
        return False, f"无法通过任何直线电机行程达到目标位置"
    
    else:
        # 后足移动，前足相对固定
        fixed_pos = current_front_pos  # 前足视为固定
        moving_pos = target_pos       # 后足目标位置
        
        # 从后足角度看前足位置，验证后足移动的可达性
        fixed_theta, fixed_z = fixed_pos
        target_theta, target_z = moving_pos
        
        # 计算从固定前足到目标后足的轴向距离
        target_axial_dist = abs(target_z - fixed_z)
        
        # 检查轴向距离是否在机器人可达范围内
        min_dist = robot.d1 + robot.d2  # a=0时
        max_dist = robot.d1 + robot.d2 + robot.s_max  # a=s_max时
        
        if target_axial_dist < min_dist or target_axial_dist > max_dist:
            return False, f"轴向距离超出可达范围 [{min_dist:.2f}, {max_dist:.2f}], 目标距离: {target_axial_dist:.2f}"
        
        # 尝试通过椭圆-圆求交点算法验证可达性
        # 计算前进角
        delta_z = target_z - fixed_z
        delta_theta_circum = surface.r_constant * ((target_theta - fixed_theta + np.pi) % (2 * np.pi) - np.pi)
        
        if abs(delta_z) < 1e-6:
            phi = np.pi/2 if delta_theta_circum > 0 else -np.pi/2
        else:
            phi = np.arctan2(delta_theta_circum, delta_z)
        
        # 尝试不同的直线电机行程a值，找到能够达到目标位置的值
        for a_test in np.linspace(0, robot.s_max, 50):
            try:
                # 使用机器人算法计算另一足位置
                delta_X, delta_theta = robot.calc_other_foot_position(
                    fixed_z, fixed_theta, phi, a_test, surface.r_constant, find_front=False
                )
                
                expected_z = fixed_z + delta_X
                expected_theta = fixed_theta + delta_theta
                
                # 检查是否接近目标位置
                z_error = abs(target_z - expected_z)
                theta_error = abs(target_theta - expected_theta)
                
                if z_error < 10.0 and theta_error < 0.1:  # 位置容差
                    return True, f"找到可达路径，直线电机行程 a={a_test:.2f}"
            except:
                continue
        
        return False, f"无法通过任何直线电机行程达到目标位置"


# 测试函数
def test_path_validation():
    print("="*60)
    print("路径合理性验证测试")
    print("="*60)
    
    # 测试案例1：合理的小幅度轴向移动路径
    print("\n测试案例1：合理的小幅度轴向移动路径")
    initial_front = (0.0, 100.0)  # (theta, z)
    initial_rear = (0.0, 70.0)    # (theta, z) - 轴向距离30mm，在[40, 48]之外，所以不合理
    
    # 改为合理的初始位置
    initial_front = (0.0, 100.0)
    initial_rear = (0.0, 60.0)    # 轴向距离40mm，在[d1+d2=40, d1+d2+s_max=48]范围内
    
    path1 = [
        ((0.0, 150.0), (0.0, 60.0)),  # 前足移动到z=150 (移动50mm)
    ]
    
    is_reasonable1, message1 = validate_path_reasonableness(initial_front, initial_rear, path1)
    print(f"结果: {is_reasonable1}")
    print(f"说明: {message1}")
    
    # 测试案例2：合理的移动序列
    print("\n测试案例2：合理的移动序列")
    initial_front2 = (0.0, 100.0)
    initial_rear2 = (0.0, 60.0)    # 轴向距离40mm，刚好是min值
    
    path2 = [
        ((0.0, 100.0), (0.0, 105.0)), # 后足移动到z=105 (移动45mm)，轴向距离变为5mm，小于min值40
    ]
    
    is_reasonable2, message2 = validate_path_reasonableness(initial_front2, initial_rear2, path2)
    print(f"结果: {is_reasonable2}")
    print(f"说明: {message2}")
    
    # 测试案例3：合理的移动序列
    print("\n测试案例3：合理的移动序列")
    initial_front3 = (0.0, 100.0)
    initial_rear3 = (0.0, 60.0)    # 轴向距离40mm
    
    path3 = [
        ((0.0, 108.0), (0.0, 60.0)), # 前足移动到z=108 (移动8mm)，轴向距离变为48mm，等于max值
    ]
    
    is_reasonable3, message3 = validate_path_reasonableness(initial_front3, initial_rear3, path3)
    print(f"结果: {is_reasonable3}")
    print(f"说明: {message3}")


if __name__ == "__main__":
    test_path_validation()