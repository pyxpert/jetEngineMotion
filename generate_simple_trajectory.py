"""
机器人运动轨迹记录
时间：2026-01-28
任务：从初始位置(前足0,40；后足0,0)爬行至目标位置(前足0,1000)
总步数：319步
格式：每行代表一步完成后的双足坐标 [前足(theta,z), 后足(theta,z)]
"""

import numpy as np

def generate_full_trajectory():
    """
    生成完整的运动轨迹
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
    
    # 当前位置
    current_front = list(initial_front_pos)
    current_rear = list(initial_rear_pos)
    
    # 设定步长（根据机器人s_max和机械约束确定）
    step_size = 6.0  # mm，略小于s_max以确保安全
    
    # 生成轨迹 - 使用蠕虫式爬行模式
    trajectory = []
    
    # 循环直到前足到达目标位置
    step_count = 0
    max_steps = 1000  # 防止无限循环
    
    while current_front[1] < target_front_z and step_count < max_steps:
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
    
    return trajectory

# 生成轨迹
full_trajectory = generate_full_trajectory()

# 写入文件
with open('robot_trajectory_simple.txt', 'w', encoding='utf-8') as f:
    f.write('"""机器人运动轨迹记录\n')
    f.write('时间：2026-01-28\n')
    f.write('任务：从初始位置(前足0,40；后足0,0)爬行至目标位置(前足0,1000)\n')
    f.write('总步数：%d\n' % len(full_trajectory))
    f.write('格式：每行代表一步完成后的双足坐标 [前足(theta,z), 后足(theta,z)]\n"""\n\n')
    
    for i, (front_pos, rear_pos) in enumerate(full_trajectory):
        f.write('[%s, %s]\n' % (front_pos, rear_pos))

print("轨迹生成完成，共%d步" % len(full_trajectory))
print("前5步示例：")
for i, (front_pos, rear_pos) in enumerate(full_trajectory[:5]):
    print(f"步骤{i+1}: [{front_pos}, {rear_pos}]")
print("...")
print("最后5步：")
for i, (front_pos, rear_pos) in enumerate(full_trajectory[-5:], len(full_trajectory)-4):
    print(f"步骤{i}: [{front_pos}, {rear_pos}]")