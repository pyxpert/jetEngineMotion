"""
生成并验证机器人运动轨迹的合理性
初始位置：前足(0, 40)，后足(0, 0)  # 圆柱面坐标系(θ, Z)
目标位置：前足(0, 1000)           # 圆柱面坐标系(θ, Z)
"""

import numpy as np
from path_validator_complete import PathValidator

def generate_trajectory():
    """
    生成从初始位置到目标位置的运动轨迹
    """
    # 机器人参数
    robot_params = {
        'd1': 20,      # 前足到中间电机的水平距离
        'd2': 20,      # 后足到中间电机的水平距离  
        'h': 30,       # 中心电机转轴到上表面的高度
        's_max': 8,    # 直线电机行程
        'R': 900       # 环境半径
    }
    
    # 初始位置（圆柱面坐标系）
    initial_front_pos = (0.0, 40.0)  # (theta, z)
    initial_rear_pos = (0.0, 0.0)    # (theta, z)
    
    # 目标位置：前足到达(0, 1000)
    target_front_z = 1000.0
    
    # 机器人机械约束
    min_distance = robot_params['d1'] + robot_params['d2']  # 40mm
    max_distance = robot_params['d1'] + robot_params['d2'] + robot_params['s_max']  # 48mm
    
    print(f"机器人机械约束: 轴向间距范围 [{min_distance}, {max_distance}] mm")
    print(f"初始位置: 前足{initial_front_pos}, 后足{initial_rear_pos}")
    print(f"目标位置: 前足(0, {target_front_z}), 后足位置待定")
    
    # 生成轨迹 - 使用蠕虫式爬行模式
    trajectory = []
    
    # 当前位置
    current_front = list(initial_front_pos)
    current_rear = list(initial_rear_pos)
    
    print(f"\n当前间距: {abs(current_front[1] - current_rear[1]):.2f}mm")
    
    # 设定步长（根据机器人s_max和机械约束确定）
    step_size = 6.0  # mm，略小于s_max以确保安全
    
    # 循环直到前足到达目标位置
    step_count = 0
    max_steps = 200  # 防止无限循环
    
    while current_front[1] < target_front_z and step_count < max_steps:
        print(f"\n步骤 {step_count + 1}:")
        print(f"  当前位置: 前足{tuple(current_front)}, 后足{tuple(current_rear)}")
        
        # 计算当前轴向间距
        current_spacing = abs(current_front[1] - current_rear[1])
        print(f"  当前轴向间距: {current_spacing:.2f}mm")
        
        # 蠕虫式爬行：交替移动前后足
        if step_count % 2 == 0:  # 移动前足
            # 计算前足的新位置
            new_front_z = min(current_front[1] + step_size, target_front_z)
            
            # 检查新位置是否满足间距约束
            new_spacing = abs(new_front_z - current_rear[1])
            
            if min_distance <= new_spacing <= max_distance:
                # 更新前足位置
                new_front_pos = (current_front[0], new_front_z)  # 保持theta不变
                trajectory.append((new_front_pos, tuple(current_rear)))
                current_front = [current_front[0], new_front_z]
                print(f"  移动前足到: {new_front_pos}")
            else:
                print(f"  前足移动会导致间距超出范围，尝试后足移动")
                # 如果前足移动会导致间距超出范围，则移动后足
                new_rear_z = current_rear[1] + step_size
                new_spacing = abs(current_front[1] - new_rear_z)
                
                if min_distance <= new_spacing <= max_distance:
                    new_rear_pos = (current_rear[0], new_rear_z)
                    trajectory.append((tuple(current_front), new_rear_pos))
                    current_rear = [current_rear[0], new_rear_z]
                    print(f"  移动后足到: {new_rear_pos}")
                else:
                    print(f"  调整步长...")
                    # 尝试更小的步长
                    adjusted_step = step_size * 0.5
                    new_rear_z = min(current_rear[1] + adjusted_step, current_front[1] - min_distance)
                    new_spacing = abs(current_front[1] - new_rear_z)
                    
                    if min_distance <= new_spacing <= max_distance:
                        new_rear_pos = (current_rear[0], new_rear_z)
                        trajectory.append((tuple(current_front), new_rear_pos))
                        current_rear = [current_rear[0], new_rear_z]
                        print(f"  移动后足到: {new_rear_pos} (小步长)")
        
        else:  # 移动后足
            # 计算后足的新位置
            # 为了让前足能够继续前进，后足需要跟上前足
            new_rear_z = min(current_rear[1] + step_size, current_front[1] - min_distance)
            
            # 检查新位置是否满足间距约束
            new_spacing = abs(current_front[1] - new_rear_z)
            
            if min_distance <= new_spacing <= max_distance:
                # 更新后足位置
                new_rear_pos = (current_rear[0], new_rear_z)
                trajectory.append((tuple(current_front), new_rear_pos))
                current_rear = [current_rear[0], new_rear_z]
                print(f"  移动后足到: {new_rear_pos}")
            else:
                print(f"  后足移动会导致间距超出范围，尝试前足移动")
                # 如果后足移动会导致间距超出范围，则移动前足
                new_front_z = min(current_front[1] + step_size, target_front_z)
                new_spacing = abs(new_front_z - current_rear[1])
                
                if min_distance <= new_spacing <= max_distance:
                    new_front_pos = (current_front[0], new_front_z)
                    trajectory.append((new_front_pos, tuple(current_rear)))
                    current_front = [current_front[0], new_front_z]
                    print(f"  移动前足到: {new_front_pos}")
        
        print(f"  新间距: {abs(current_front[1] - current_rear[1]):.2f}mm")
        step_count += 1
    
    print(f"\n轨迹生成完成，共{len(trajectory)}步")
    print(f"最终位置: 前足{tuple(current_front)}, 后足{tuple(current_rear)}")
    
    return initial_front_pos, initial_rear_pos, trajectory

if __name__ == "__main__":
    initial_front, initial_rear, trajectory = generate_trajectory()
    
    print("\n生成的轨迹点:")
    for i, (front_pos, rear_pos) in enumerate(trajectory[:10]):  # 只打印前10个点
        print(f"  步骤{i+1}: 前足{front_pos}, 后足{rear_pos}")
    
    if len(trajectory) > 10:
        print(f"  ... 还有{len(trajectory) - 10}个点")