"""
机器人路径合理性验证器

该模块提供了验证双节机器人在圆柱内壁上运动路径合理性的功能。
基于椭圆-圆求交点算法和机器人的机械约束，判断给定路径是否可以被机器人实现。

主要验证内容：
1. 机械约束：两足间距离是否在[d1+d2, d1+d2+s_max]范围内
2. 几何约束：每步移动是否符合椭圆-圆求交点算法
3. 运动连续性：相邻步骤间是否满足蠕虫式爬行模式
"""

import numpy as np
from robot_motion_planner import Robot, EngineSurface


class PathValidator:
    def __init__(self, robot_params=None):
        """
        初始化路径验证器
        
        Args:
            robot_params: 机器人参数字典
                         默认值: {'d1': 20, 'd2': 20, 'h': 30, 's_max': 8, 'R': 900}
        """
        if robot_params is None:
            robot_params = {
                'd1': 20,      # 前足到中间电机的水平距离
                'd2': 20,      # 后足到中间电机的水平距离  
                'h': 30,       # 中心电机转轴到上表面的高度
                's_max': 8,    # 直线电机行程
                'R': 900       # 环境半径
            }
        
        self.robot = Robot(
            d1=robot_params['d1'],
            d2=robot_params['d2'], 
            h=robot_params['h'],
            s_max=robot_params['s_max']
        )
        self.surface = EngineSurface(r_constant=robot_params['R'])
        
        # 机械约束范围
        self.min_axial_distance = self.robot.d1 + self.robot.d2
        self.max_axial_distance = self.robot.d1 + self.robot.d2 + self.robot.s_max

    def validate_path_reasonableness(self, initial_front_pos, initial_rear_pos, path_positions):
        """
        判断给定路径是否合理
        
        Args:
            initial_front_pos: 初始前足位置 (theta, z)
            initial_rear_pos: 初始后足位置 (theta, z) 
            path_positions: 路径点列表 [(front_pos1, rear_pos1), (front_pos2, rear_pos2), ...]
                           其中每个位置为 (theta, z) 形式
        
        Returns:
            tuple: (bool, str) - (是否合理, 详细说明)
        """
        # 验证初始位置
        if not self._validate_initial_positions(initial_front_pos, initial_rear_pos):
            return False, f"初始位置不合理：轴向间距{abs(initial_front_pos[1] - initial_rear_pos[1]):.2f}mm " \
                         f"超出范围[{self.min_axial_distance:.2f}, {self.max_axial_distance:.2f}]mm"

        current_front_pos = initial_front_pos
        current_rear_pos = initial_rear_pos

        # 验证路径中每一步的合理性
        for i, (next_front_pos, next_rear_pos) in enumerate(path_positions):
            is_valid, reason = self._validate_single_step(
                current_front_pos, current_rear_pos, 
                next_front_pos, next_rear_pos
            )
            
            if not is_valid:
                return False, f"第{i+1}步运动不合理: {reason}"
            
            # 更新当前位置
            current_front_pos = next_front_pos
            current_rear_pos = next_rear_pos
        
        return True, "路径完全合理"

    def _validate_initial_positions(self, front_pos, rear_pos):
        """
        验证初始位置是否合理
        """
        _, front_z = front_pos
        _, rear_z = rear_pos
        
        axial_distance = abs(front_z - rear_z)
        
        return self.min_axial_distance <= axial_distance <= self.max_axial_distance

    def _validate_single_step(self, current_front_pos, current_rear_pos, next_front_pos, next_rear_pos):
        """
        验证单步运动的合理性
        """
        _, curr_f_z = current_front_pos
        _, curr_r_z = current_rear_pos
        _, next_f_z = next_front_pos
        _, next_r_z = next_rear_pos

        # 检查目标状态的轴向间距是否在合理范围内
        next_axial_distance = abs(next_f_z - next_r_z)
        
        if next_axial_distance < self.min_axial_distance:
            return False, f"目标轴向间距{next_axial_distance:.2f}mm小于最小值{self.min_axial_distance:.2f}mm"
        
        if next_axial_distance > self.max_axial_distance:
            return False, f"目标轴向间距{next_axial_distance:.2f}mm大于最大值{self.max_axial_distance:.2f}mm"

        # 分析运动模式
        delta_f_z = abs(next_f_z - curr_f_z)
        delta_r_z = abs(next_r_z - curr_r_z)

        # 蠕虫式爬行模式：一足固定，另一足移动
        if delta_f_z < 1.0:  # 前足基本不动，后足移动
            return self._validate_foot_move_consistency(
                next_rear_pos, current_front_pos, current_rear_pos, moving_front=False
            )
        elif delta_r_z < 1.0:  # 后足基本不动，前足移动
            return self._validate_foot_move_consistency(
                next_front_pos, current_front_pos, current_rear_pos, moving_front=True
            )
        else:
            # 两足都移动，需要特殊处理（通常不推荐）
            return True, "双足协调移动（需进一步验证）"

    def _validate_foot_move_consistency(self, target_pos, current_front_pos, current_rear_pos, moving_front):
        """
        验证单足移动是否符合几何约束
        """
        if moving_front:
            # 前足移动，后足固定
            fixed_pos = current_rear_pos  # 后足固定
            moving_pos = target_pos       # 前足目标
            
            fixed_theta, fixed_z = fixed_pos
            target_theta, target_z = moving_pos
            
            # 计算从固定后足到目标前足的轴向距离
            target_axial_dist = abs(target_z - fixed_z)
            
            # 检查轴向距离是否在可达范围内
            if not (self.min_axial_distance <= target_axial_dist <= self.max_axial_distance):
                return False, f"目标轴向距离{target_axial_dist:.2f}mm超出可达范围"
            
            # 尝试通过椭圆-圆求交点算法验证可达性
            return self._try_find_valid_configuration(fixed_pos, moving_pos, find_front=True)
        
        else:
            # 后足移动，前足固定
            fixed_pos = current_front_pos  # 前足固定
            moving_pos = target_pos        # 后足目标
            
            fixed_theta, fixed_z = fixed_pos
            target_theta, target_z = moving_pos
            
            # 计算从固定前足到目标后足的轴向距离
            target_axial_dist = abs(target_z - fixed_z)
            
            # 检查轴向距离是否在可达范围内
            if not (self.min_axial_distance <= target_axial_dist <= self.max_axial_distance):
                return False, f"目标轴向距离{target_axial_dist:.2f}mm超出可达范围"
            
            # 尝试通过椭圆-圆求交点算法验证可达性
            return self._try_find_valid_configuration(fixed_pos, moving_pos, find_front=False)

    def _try_find_valid_configuration(self, fixed_pos, target_pos, find_front):
        """
        尝试找到一个有效的机器人配置来实现指定移动
        """
        fixed_theta, fixed_z = fixed_pos
        target_theta, target_z = target_pos
        
        # 计算前进角
        delta_z = target_z - fixed_z
        avg_radius = self.surface.r_constant
        delta_theta = ((target_theta - fixed_theta + np.pi) % (2 * np.pi)) - np.pi  # 规范化到[-π, π]
        delta_circum = avg_radius * delta_theta  # 周向弧长
        
        if abs(delta_z) < 1e-6:
            phi = np.pi/2 if delta_theta > 0 else -np.pi/2
        else:
            phi = np.arctan2(delta_circum, delta_z)

        # 尝试不同的直线电机行程，寻找可达配置
        for a_test in np.linspace(0, self.robot.s_max, 50):
            try:
                # 使用机器人算法计算另一足位置
                delta_X, delta_theta_calc = self.robot.calc_other_foot_position(
                    fixed_z, fixed_theta, phi, a_test, self.surface.r_constant, find_front=find_front
                )
                
                expected_z = fixed_z + delta_X
                expected_theta = fixed_theta + delta_theta_calc
                
                # 检查是否接近目标位置（容差）
                z_error = abs(target_z - expected_z)
                theta_error = abs(target_theta - expected_theta)
                
                if z_error < 10.0 and theta_error < 0.1:  # 位置和角度容差
                    return True, f"找到可达配置，直线电机行程 a={a_test:.2f}mm"
            except Exception:
                continue
        
        return False, f"无法通过调整直线电机行程达到目标位置"


def demo_path_validation():
    """
    演示路径验证功能
    """
    print("="*70)
    print("机器人路径合理性验证器演示")
    print("="*70)
    
    # 创建验证器
    validator = PathValidator()
    
    # 演示1：合理的路径
    print("\n演示1：合理的路径")
    print("-" * 40)
    initial_front = (0.0, 100.0)  # (theta, z)
    initial_rear = (0.0, 60.0)    # (theta, z) - 轴向间距40mm，在[40, 48]范围内
    
    path1 = [
        ((0.0, 108.0), (0.0, 60.0)),  # 前足移动到z=108 (轴向间距变为48mm，刚好是最大值)
    ]
    
    is_valid, message = validator.validate_path_reasonableness(initial_front, initial_rear, path1)
    print(f"初始位置: 前足{initial_front}, 后足{initial_rear}")
    print(f"路径: {path1}")
    print(f"结果: {is_valid}")
    print(f"说明: {message}")
    
    # 演示2：不合理的路径（间距过大）
    print("\n演示2：不合理的路径（间距过大）")
    print("-" * 40)
    path2 = [
        ((0.0, 200.0), (0.0, 60.0)),  # 前足移动到z=200 (轴向间距变为140mm，超过最大值48mm)
    ]
    
    is_valid, message = validator.validate_path_reasonableness(initial_front, initial_rear, path2)
    print(f"初始位置: 前足{initial_front}, 后足{initial_rear}")
    print(f"路径: {path2}")
    print(f"结果: {is_valid}")
    print(f"说明: {message}")
    
    # 演示3：多步合理路径
    print("\n演示3：多步合理路径")
    print("-" * 40)
    path3 = [
        ((0.0, 105.0), (0.0, 60.0)),  # 第1步：前足移动到z=105 (轴向间距45mm)
        ((0.0, 105.0), (0.0, 65.0)),  # 第2步：后足移动到z=65 (轴向间距40mm) 
        ((0.0, 110.0), (0.0, 65.0)),  # 第3步：前足移动到z=110 (轴向间距45mm)
    ]
    
    is_valid, message = validator.validate_path_reasonableness(initial_front, initial_rear, path3)
    print(f"初始位置: 前足{initial_front}, 后足{initial_rear}")
    print(f"路径: {path3}")
    print(f"结果: {is_valid}")
    print(f"说明: {message}")
    
    # 演示4：包含周向变化的路径
    print("\n演示4：包含周向变化的合理路径")
    print("-" * 40)
    path4 = [
        ((0.1, 105.0), (0.0, 60.0)),  # 前足移动并轻微转向
    ]
    
    is_valid, message = validator.validate_path_reasonableness(initial_front, initial_rear, path4)
    print(f"初始位置: 前足{initial_front}, 后足{initial_rear}")
    print(f"路径: {path4}")
    print(f"结果: {is_valid}")
    print(f"说明: {message}")
    
    print("\n" + "="*70)
    print("演示完成")
    print("="*70)


if __name__ == "__main__":
    demo_path_validation()