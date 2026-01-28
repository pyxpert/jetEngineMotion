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
    max_steps = 1000  # 防止无限循环
    
    while current_front[1] < target_front_z and step_count < max_steps:
        if step_count % 50 == 0:  # 每50步打印一次状态
            print(f"\n步骤 {step_count + 1}:")
            print(f"  当前位置: 前足{tuple(current_front)}, 后足{tuple(current_rear)}")
        
        # 计算当前轴向间距
        current_spacing = abs(current_front[1] - current_rear[1])
        
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
            else:
                # 如果前足移动会导致间距超出范围，则移动后足
                new_rear_z = current_rear[1] + step_size
                new_spacing = abs(current_front[1] - new_rear_z)
                
                if min_distance <= new_spacing <= max_distance:
                    new_rear_pos = (current_rear[0], new_rear_z)
                    trajectory.append((tuple(current_front), new_rear_pos))
                    current_rear = [current_rear[0], new_rear_z]
                else:
                    # 尝试更小的步长
                    adjusted_step = step_size * 0.5
                    new_rear_z = min(current_rear[1] + adjusted_step, current_front[1] - min_distance)
                    new_spacing = abs(current_front[1] - new_rear_z)
                    
                    if min_distance <= new_spacing <= max_distance:
                        new_rear_pos = (current_rear[0], new_rear_z)
                        trajectory.append((tuple(current_front), new_rear_pos))
                        current_rear = [current_rear[0], new_rear_z]
        
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
            else:
                # 如果后足移动会导致间距超出范围，则移动前足
                new_front_z = min(current_front[1] + step_size, target_front_z)
                new_spacing = abs(new_front_z - current_rear[1])
                
                if min_distance <= new_spacing <= max_distance:
                    new_front_pos = (current_front[0], new_front_z)
                    trajectory.append((new_front_pos, tuple(current_rear)))
                    current_front = [current_front[0], new_front_z]
        
        step_count += 1
    
    print(f"\n轨迹生成完成，共{len(trajectory)}步")
    print(f"最终位置: 前足{tuple(current_front)}, 后足{tuple(current_rear)}")
    
    return initial_front_pos, initial_rear_pos, trajectory

def validate_trajectory():
    """
    验证生成的轨迹是否合理
    """
    initial_front, initial_rear, trajectory = generate_trajectory()
    
    print(f"\n开始验证轨迹合理性...")
    print(f"轨迹长度: {len(trajectory)} 步")
    
    # 创建验证器
    validator = PathValidator()
    
    # 验证轨迹（只取前几步进行测试，因为完整轨迹可能很长）
    if len(trajectory) > 100:  # 如果轨迹太长，只验证前100步
        test_trajectory = trajectory[:100]
        print(f"由于轨迹较长({len(trajectory)}步)，将分段验证，先验证前100步")
        
        # 分段验证
        is_valid = True
        for i in range(0, len(test_trajectory), 10):
            end_idx = min(i + 10, len(test_trajectory))
            segment = test_trajectory[i:end_idx]
            
            # 获取段的起始位置
            if i == 0:
                seg_start_front = initial_front
                seg_start_rear = initial_rear
            else:
                # 从前一段的最终位置开始
                prev_end_pos = trajectory[i-1]
                seg_start_front = prev_end_pos[0]
                seg_start_rear = prev_end_pos[1]
            
            print(f"验证段 {i//10 + 1} ({i+1}-{end_idx}步):")
            is_seg_valid, msg = validator.validate_path_reasonableness(seg_start_front, seg_start_rear, segment)
            print(f"  结果: {is_seg_valid}")
            print(f"  说明: {msg}")
            
            if not is_seg_valid:
                is_valid = False
                break
        
        if is_valid and len(trajectory) > 100:
            # 验证剩余部分
            remaining_trajectory = trajectory[100:]
            seg_start_front = trajectory[99][0]  # 上一段结束时的前足位置
            seg_start_rear = trajectory[99][1]  # 上一段结束时的后足位置
            
            print(f"验证剩余部分 ({len(remaining_trajectory)}步):")
            is_remaining_valid, msg = validator.validate_path_reasonableness(seg_start_front, seg_start_rear, remaining_trajectory)
            print(f"  结果: {is_remaining_valid}")
            print(f"  说明: {msg}")
            
            is_valid = is_valid and is_remaining_valid
    else:
        # 验证完整轨迹
        is_valid, message = validator.validate_path_reasonableness(initial_front, initial_rear, trajectory)
        print(f"验证结果: {is_valid}")
        print(f"说明: {message}")
    
    print(f"\n轨迹验证完成")
    print(f"总体结果: {'合理' if is_valid else '不合理'}")
    
    # 打印一些轨迹点作为示例
    print(f"\n轨迹示例 (前10个点):")
    for i, (front_pos, rear_pos) in enumerate(trajectory[:10]):
        spacing = abs(front_pos[1] - rear_pos[1])
        print(f"  步骤{i+1}: 前足{front_pos}, 后足{rear_pos}, 间距{spacing:.2f}mm")
    
    if len(trajectory) > 10:
        print(f"  ... 还有{len(trajectory) - 10}个点")
        print(f"最后3个点:")
        for i, (front_pos, rear_pos) in enumerate(trajectory[-3:], len(trajectory)-2):
            spacing = abs(front_pos[1] - rear_pos[1])
            print(f"  步骤{i}: 前足{front_pos}, 后足{rear_pos}, 间距{spacing:.2f}mm")

if __name__ == "__main__":
    validate_trajectory()