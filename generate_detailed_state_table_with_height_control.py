"""
机器人运动状态详细记录表（更新版 - 考虑磁铁高度控制）
时间：2026-01-28
任务：从初始位置(前足0,40；后足0,0)爬行至目标位置(前足0,1000)
基于优化轨迹（255步）
使用六位数字状态码：ω1ω2ω3vm1m2
考虑磁铁脱附后需抬起，吸附前需放下
"""

import numpy as np

def generate_motor_positions_and_states_with_height_control():
    """
    生成机器人运动过程中的详细状态表（使用六位数字状态码并考虑磁铁高度控制）
    状态码格式：ω1ω2ω3vm1m2
    - ω1, ω2, ω3: 旋转电机状态 (0=停止, 1=正转, 2=反转)
    - v: 直线电机状态 (0=停止, 1=正转, 2=反转) 
    - m1, m2: 电磁铁状态 (0=脱附, 1=吸附)
    
    考虑磁铁高度控制：
    - 脱附后：磁铁需抬起一定高度以避免摩擦
    - 吸附前：磁铁需放下以贴紧壁面
    """
    # 机器人参数
    robot_params = {
        'd1': 20,      # 前足到中间电机的水平距离
        'd2': 20,      # 后足到中间电机的水平距离  
        'h': 30,       # 中心电机转轴到上表面的高度
        's_max': 8,    # 直线电机行程
        'R': 900       # 环境半径
    }
    
    # 磁铁高度控制参数
    base_height = robot_params['h']  # 基础高度
    lift_height = base_height + 5    # 抬起时的高度（高出5mm以避免摩擦）
    
    # 初始位置（圆柱面坐标系）
    initial_front_pos = (0.0, 40.0)  # (theta, z)
    initial_rear_pos = (0.0, 0.0)    # (theta, z)
    
    # 目标位置：前足到达(0, 1000)
    target_front_z = 1000.0
    
    # 当前位置
    current_front = list(initial_front_pos)
    current_rear = list(initial_rear_pos)
    
    # 使用接近直线电机最大行程的步长
    step_size = 7.5  # mm，接近s_max但留有余量以确保安全
    
    # 生成轨迹和状态 - 使用蠕虫式爬行模式
    states_list = []
    
    # 初始状态：前足吸附，后足吸附，磁铁处于基础高度
    # 初始状态：所有电机停止，前后磁铁吸附，磁铁在基础高度
    current_state = {
        'step': 0,
        'state_code': '000011',  # ω1ω2ω3vm1m2: 旋转电机停止，直线电机停止，前后磁铁吸附
        'front_magnet_status': '吸附',
        'rear_magnet_status': '吸附',
        'front_position': current_front.copy(),
        'rear_position': current_rear.copy(),
        'center_motor_angle': 0.0,  # 初始角度
        'linear_motor_pos': 0.0,    # 初始直线电机位置
        'front_magnet_height': base_height,  # 磁铁高度
        'rear_magnet_height': base_height,   # 磁铁高度
        'front_magnet_attach_pos': current_front.copy(),  # 前足磁铁吸附位置
        'rear_magnet_attach_pos': current_rear.copy()     # 后足磁铁吸附位置
    }
    states_list.append(current_state.copy())
    
    # 循环直到前足到达目标位置
    step_count = 0
    max_steps = 1000  # 防止无限循环
    
    while current_front[1] < target_front_z and step_count < max_steps:
        # 计算当前轴向间距
        current_spacing = abs(current_front[1] - current_rear[1])
        
        # 蠕虫式爬行：交替移动前后足
        if step_count % 2 == 0:  # 移动前足
            # 状态变化：后足保持吸附，前足准备脱离
            # ω1ω2ω3vm1m2: 前磁铁脱附，其他不变
            detach_state = {
                'step': step_count + 1,
                'state_code': '000001',  # 前磁铁脱附，后磁铁仍吸附
                'front_magnet_status': '脱附',
                'rear_magnet_status': '吸附',
                'front_position': current_front.copy(),
                'rear_position': current_rear.copy(),
                'center_motor_angle': 0.0,
                'linear_motor_pos': abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_height': lift_height,  # 抬起前磁铁以避免摩擦
                'rear_magnet_height': base_height,   # 后磁铁保持基础高度
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': current_rear.copy()
            }
            states_list.append(detach_state.copy())
            
            # 计算前足的新位置
            new_front_z = min(current_front[1] + step_size, target_front_z)
            
            # 检查新位置是否满足间距约束
            new_spacing = abs(new_front_z - current_rear[1])
            
            if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                # 更新前足位置
                new_front_pos = (current_front[0], new_front_z)  # 保持theta不变
                
                # 折叠状态：前足脱附，后足吸附，直线电机收缩
                # ω1ω2ω3vm1m2: 直线电机收缩(2=反转)，其他不变
                fold_state = {
                    'step': step_count + 1,
                    'state_code': '000201',  # 直线电机收缩，前磁铁脱附，后磁铁吸附
                    'front_magnet_status': '脱附',
                    'rear_magnet_status': '吸附',
                    'front_position': new_front_pos,
                    'rear_position': current_rear.copy(),
                    'center_motor_angle': 0.0,
                    'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                    'front_magnet_height': lift_height,  # 前磁铁保持抬起状态
                    'rear_magnet_height': base_height,   # 后磁铁保持基础高度
                    'front_magnet_attach_pos': current_front.copy(),  # 旧吸附位置
                    'rear_magnet_attach_pos': current_rear.copy()
                }
                states_list.append(fold_state.copy())
                
                # 前足到达新位置，准备吸附（降低磁铁高度）
                prep_attach_state = {
                    'step': step_count + 1,
                    'state_code': '000001',  # 直线电机停止，前磁铁仍脱附，后磁铁吸附
                    'front_magnet_status': '脱附',
                    'rear_magnet_status': '吸附',
                    'front_position': new_front_pos,
                    'rear_position': current_rear.copy(),
                    'center_motor_angle': 0.0,
                    'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                    'front_magnet_height': base_height,  # 降低前磁铁以贴紧壁面
                    'rear_magnet_height': base_height,   # 后磁铁保持基础高度
                    'front_magnet_attach_pos': new_front_pos,  # 即将吸附的位置
                    'rear_magnet_attach_pos': current_rear.copy()
                }
                states_list.append(prep_attach_state.copy())
                
                # 前足吸附
                attach_state = {
                    'step': step_count + 1,
                    'state_code': '000011',  # 前磁铁吸附，后磁铁吸附
                    'front_magnet_status': '吸附',
                    'rear_magnet_status': '吸附',
                    'front_position': new_front_pos,
                    'rear_position': current_rear.copy(),
                    'center_motor_angle': 0.0,
                    'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                    'front_magnet_height': base_height,  # 前磁铁吸附时保持基础高度
                    'rear_magnet_height': base_height,   # 后磁铁保持基础高度
                    'front_magnet_attach_pos': new_front_pos,
                    'rear_magnet_attach_pos': current_rear.copy()
                }
                states_list.append(attach_state.copy())
                
                current_front = [current_front[0], new_front_z]
            else:
                # 如果前足移动会导致间距超出范围，则移动后足
                new_rear_z = current_rear[1] + step_size
                new_spacing = abs(current_front[1] - new_rear_z)
                
                if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                    new_rear_pos = (current_rear[0], new_rear_z)
                    
                    # 后足脱附（抬起后磁铁）
                    detach_state = {
                        'step': step_count + 1,
                        'state_code': '000010',  # 后磁铁脱附，前磁铁仍吸附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '脱附',
                        'front_position': current_front.copy(),
                        'rear_position': new_rear_pos,
                        'center_motor_angle': 0.0,
                        'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                        'front_magnet_height': base_height,   # 前磁铁保持基础高度
                        'rear_magnet_height': lift_height,    # 抬起后磁铁以避免摩擦
                        'front_magnet_attach_pos': current_front.copy(),
                        'rear_magnet_attach_pos': current_rear.copy()
                    }
                    states_list.append(detach_state.copy())
                    
                    # 扩展状态：前足吸附，后足脱附，直线电机伸展
                    extend_state = {
                        'step': step_count + 1,
                        'state_code': '000110',  # 直线电机伸展(1=正转)，前磁铁吸附，后磁铁脱附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '脱附',
                        'front_position': current_front.copy(),
                        'rear_position': new_rear_pos,
                        'center_motor_angle': 0.0,
                        'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                        'front_magnet_height': base_height,   # 前磁铁保持基础高度
                        'rear_magnet_height': lift_height,    # 后磁铁保持抬起状态
                        'front_magnet_attach_pos': current_front.copy(),
                        'rear_magnet_attach_pos': current_rear.copy()
                    }
                    states_list.append(extend_state.copy())
                    
                    # 后足到达新位置，准备吸附（降低磁铁高度）
                    prep_attach_state = {
                        'step': step_count + 1,
                        'state_code': '000010',  # 直线电机停止，前磁铁吸附，后磁铁仍脱附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '脱附',
                        'front_position': current_front.copy(),
                        'rear_position': new_rear_pos,
                        'center_motor_angle': 0.0,
                        'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                        'front_magnet_height': base_height,   # 前磁铁保持基础高度
                        'rear_magnet_height': base_height,    # 降低后磁铁以贴紧壁面
                        'front_magnet_attach_pos': current_front.copy(),
                        'rear_magnet_attach_pos': new_rear_pos  # 即将吸附的位置
                    }
                    states_list.append(prep_attach_state.copy())
                    
                    # 后足吸附
                    attach_state = {
                        'step': step_count + 1,
                        'state_code': '000011',  # 前后磁铁都吸附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '吸附',
                        'front_position': current_front.copy(),
                        'rear_position': new_rear_pos,
                        'center_motor_angle': 0.0,
                        'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                        'front_magnet_height': base_height,   # 前磁铁吸附时保持基础高度
                        'rear_magnet_height': base_height,    # 后磁铁吸附时保持基础高度
                        'front_magnet_attach_pos': current_front.copy(),
                        'rear_magnet_attach_pos': new_rear_pos
                    }
                    states_list.append(attach_state.copy())
                    
                    current_rear = [current_rear[0], new_rear_z]
                else:
                    # 尝试较小的步长
                    adjusted_step = step_size * 0.7
                    new_rear_z = min(current_rear[1] + adjusted_step, current_front[1] - (robot_params['d1'] + robot_params['d2']))
                    new_spacing = abs(current_front[1] - new_rear_z)
                    
                    if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                        new_rear_pos = (current_rear[0], new_rear_z)
                        
                        # 后足脱附（抬起后磁铁）
                        detach_state = {
                            'step': step_count + 1,
                            'state_code': '000010',  # 后磁铁脱附，前磁铁仍吸附
                            'front_magnet_status': '吸附',
                            'rear_magnet_status': '脱附',
                            'front_position': current_front.copy(),
                            'rear_position': new_rear_pos,
                            'center_motor_angle': 0.0,
                            'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                            'front_magnet_height': base_height,   # 前磁铁保持基础高度
                            'rear_magnet_height': lift_height,    # 抬起后磁铁以避免摩擦
                            'front_magnet_attach_pos': current_front.copy(),
                            'rear_magnet_attach_pos': current_rear.copy()
                        }
                        states_list.append(detach_state.copy())
                        
                        # 扩展状态：前足吸附，后足脱附，直线电机伸展
                        extend_state = {
                            'step': step_count + 1,
                            'state_code': '000110',  # 直线电机伸展(1=正转)，前磁铁吸附，后磁铁脱附
                            'front_magnet_status': '吸附',
                            'rear_magnet_status': '脱附',
                            'front_position': current_front.copy(),
                            'rear_position': new_rear_pos,
                            'center_motor_angle': 0.0,
                            'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                            'front_magnet_height': base_height,   # 前磁铁保持基础高度
                            'rear_magnet_height': lift_height,    # 后磁铁保持抬起状态
                            'front_magnet_attach_pos': current_front.copy(),
                            'rear_magnet_attach_pos': current_rear.copy()
                        }
                        states_list.append(extend_state.copy())
                        
                        # 后足到达新位置，准备吸附（降低磁铁高度）
                        prep_attach_state = {
                            'step': step_count + 1,
                            'state_code': '000010',  # 直线电机停止，前磁铁吸附，后磁铁仍脱附
                            'front_magnet_status': '吸附',
                            'rear_magnet_status': '脱附',
                            'front_position': current_front.copy(),
                            'rear_position': new_rear_pos,
                            'center_motor_angle': 0.0,
                            'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                            'front_magnet_height': base_height,   # 前磁铁保持基础高度
                            'rear_magnet_height': base_height,    # 降低后磁铁以贴紧壁面
                            'front_magnet_attach_pos': current_front.copy(),
                            'rear_magnet_attach_pos': new_rear_pos  # 即将吸附的位置
                        }
                        states_list.append(prep_attach_state.copy())
                        
                        # 后足吸附
                        attach_state = {
                            'step': step_count + 1,
                            'state_code': '000011',  # 前后磁铁都吸附
                            'front_magnet_status': '吸附',
                            'rear_magnet_status': '吸附',
                            'front_position': current_front.copy(),
                            'rear_position': new_rear_pos,
                            'center_motor_angle': 0.0,
                            'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                            'front_magnet_height': base_height,   # 前磁铁吸附时保持基础高度
                            'rear_magnet_height': base_height,    # 后磁铁吸附时保持基础高度
                            'front_magnet_attach_pos': current_front.copy(),
                            'rear_magnet_attach_pos': new_rear_pos
                        }
                        states_list.append(attach_state.copy())
                        
                        current_rear = [current_rear[0], new_rear_z]
        
        else:  # 移动后足
            # 状态变化：前足保持吸附，后足准备脱离（抬起后磁铁）
            detach_state = {
                'step': step_count + 1,
                'state_code': '000010',  # 后磁铁脱附，前磁铁仍吸附
                'front_magnet_status': '吸附',
                'rear_magnet_status': '脱附',
                'front_position': current_front.copy(),
                'rear_position': current_rear.copy(),
                'center_motor_angle': 0.0,
                'linear_motor_pos': abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_height': base_height,   # 前磁铁保持基础高度
                'rear_magnet_height': lift_height,    # 抬起后磁铁以避免摩擦
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': current_rear.copy()
            }
            states_list.append(detach_state.copy())
            
            # 计算后足的新位置
            # 为了让前足能够继续前进，后足需要跟上前足
            new_rear_z = min(current_rear[1] + step_size, current_front[1] - (robot_params['d1'] + robot_params['d2']))
            
            # 检查新位置是否满足间距约束
            new_spacing = abs(current_front[1] - new_rear_z)
            
            if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                # 更新后足位置
                new_rear_pos = (current_rear[0], new_rear_z)
                
                # 扩展状态：前足吸附，后足脱附，直线电机伸展
                extend_state = {
                    'step': step_count + 1,
                    'state_code': '000110',  # 直线电机伸展(1=正转)，前磁铁吸附，后磁铁脱附
                    'front_magnet_status': '吸附',
                    'rear_magnet_status': '脱附',
                    'front_position': current_front.copy(),
                    'rear_position': new_rear_pos,
                    'center_motor_angle': 0.0,
                    'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                    'front_magnet_height': base_height,   # 前磁铁保持基础高度
                    'rear_magnet_height': lift_height,    # 后磁铁保持抬起状态
                    'front_magnet_attach_pos': current_front.copy(),
                    'rear_magnet_attach_pos': current_rear.copy()
                }
                states_list.append(extend_state.copy())
                
                # 后足到达新位置，准备吸附（降低磁铁高度）
                prep_attach_state = {
                    'step': step_count + 1,
                    'state_code': '000010',  # 直线电机停止，前磁铁吸附，后磁铁仍脱附
                    'front_magnet_status': '吸附',
                    'rear_magnet_status': '脱附',
                    'front_position': current_front.copy(),
                    'rear_position': new_rear_pos,
                    'center_motor_angle': 0.0,
                    'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                    'front_magnet_height': base_height,   # 前磁铁保持基础高度
                    'rear_magnet_height': base_height,    # 降低后磁铁以贴紧壁面
                    'front_magnet_attach_pos': current_front.copy(),
                    'rear_magnet_attach_pos': new_rear_pos  # 即将吸附的位置
                }
                states_list.append(prep_attach_state.copy())
                
                # 后足吸附
                attach_state = {
                    'step': step_count + 1,
                    'state_code': '000011',  # 前后磁铁都吸附
                    'front_magnet_status': '吸附',
                    'rear_magnet_status': '吸附',
                    'front_position': current_front.copy(),
                    'rear_position': new_rear_pos,
                    'center_motor_angle': 0.0,
                    'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                    'front_magnet_height': base_height,   # 前磁铁吸附时保持基础高度
                    'rear_magnet_height': base_height,    # 后磁铁吸附时保持基础高度
                    'front_magnet_attach_pos': current_front.copy(),
                    'rear_magnet_attach_pos': new_rear_pos
                }
                states_list.append(attach_state.copy())
                
                current_rear = [current_rear[0], new_rear_z]
            else:
                # 如果后足移动会导致间距超出范围，则移动前足
                new_front_z = min(current_front[1] + step_size, target_front_z)
                new_spacing = abs(new_front_z - current_rear[1])
                
                if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                    new_front_pos = (current_front[0], new_front_z)
                    
                    # 前足脱附（抬起前磁铁）
                    detach_state = {
                        'step': step_count + 1,
                        'state_code': '000001',  # 前磁铁脱附，后磁铁仍吸附
                        'front_magnet_status': '脱附',
                        'rear_magnet_status': '吸附',
                        'front_position': new_front_pos,
                        'rear_position': current_rear.copy(),
                        'center_motor_angle': 0.0,
                        'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                        'front_magnet_height': lift_height,   # 抬起前磁铁以避免摩擦
                        'rear_magnet_height': base_height,    # 后磁铁保持基础高度
                        'front_magnet_attach_pos': current_front.copy(),  # 旧吸附位置
                        'rear_magnet_attach_pos': current_rear.copy()
                    }
                    states_list.append(detach_state.copy())
                    
                    # 折叠状态：前足脱附，后足吸附，直线电机收缩
                    fold_state = {
                        'step': step_count + 1,
                        'state_code': '000201',  # 直线电机收缩(2=反转)，前磁铁脱附，后磁铁吸附
                        'front_magnet_status': '脱附',
                        'rear_magnet_status': '吸附',
                        'front_position': new_front_pos,
                        'rear_position': current_rear.copy(),
                        'center_motor_angle': 0.0,
                        'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                        'front_magnet_height': lift_height,   # 前磁铁保持抬起状态
                        'rear_magnet_height': base_height,    # 后磁铁保持基础高度
                        'front_magnet_attach_pos': current_front.copy(),  # 旧吸附位置
                        'rear_magnet_attach_pos': current_rear.copy()
                    }
                    states_list.append(fold_state.copy())
                    
                    # 前足到达新位置，准备吸附（降低磁铁高度）
                    prep_attach_state = {
                        'step': step_count + 1,
                        'state_code': '000001',  # 直线电机停止，前磁铁仍脱附，后磁铁吸附
                        'front_magnet_status': '脱附',
                        'rear_magnet_status': '吸附',
                        'front_position': new_front_pos,
                        'rear_position': current_rear.copy(),
                        'center_motor_angle': 0.0,
                        'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                        'front_magnet_height': base_height,   # 降低前磁铁以贴紧壁面
                        'rear_magnet_height': base_height,    # 后磁铁保持基础高度
                        'front_magnet_attach_pos': new_front_pos,  # 即将吸附的位置
                        'rear_magnet_attach_pos': current_rear.copy()
                    }
                    states_list.append(prep_attach_state.copy())
                    
                    # 前足吸附
                    attach_state = {
                        'step': step_count + 1,
                        'state_code': '000011',  # 前后磁铁都吸附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '吸附',
                        'front_position': new_front_pos,
                        'rear_position': current_rear.copy(),
                        'center_motor_angle': 0.0,
                        'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                        'front_magnet_height': base_height,   # 前磁铁吸附时保持基础高度
                        'rear_magnet_height': base_height,    # 后磁铁吸附时保持基础高度
                        'front_magnet_attach_pos': new_front_pos,
                        'rear_magnet_attach_pos': current_rear.copy()
                    }
                    states_list.append(attach_state.copy())
                    
                    current_front = [current_front[0], new_front_z]
        
        step_count += 1
    
    return states_list

# 生成状态列表
states_list = generate_motor_positions_and_states_with_height_control()

# 写入详细状态表到文件
with open('robot_detailed_state_table_with_height_control.txt', 'w', encoding='utf-8') as f:
    f.write('"""机器人运动状态详细记录表（更新版 - 考虑磁铁高度控制）\n')
    f.write('时间：2026-01-28\n')
    f.write('任务：从初始位置(前足0,40；后足0,0)爬行至目标位置(前足0,1000)\n')
    f.write('基于优化轨迹（255步）\n')
    f.write('使用六位数字状态码：ω1ω2ω3vm1m2\n')
    f.write('考虑磁铁脱附后需抬起，吸附前需放下以避免摩擦并确保良好接触\n')
    f.write('"""\n\n')
    
    # 表头
    f.write(f"{'步数':<6} {'状态码':<10} {'前足磁铁':<10} {'后足磁铁':<10} {'前足位置':<15} {'后足位置':<15} {'前磁铁高度':<10} {'后磁铁高度':<10} {'直线电机':<10} {'前足吸附位置':<15} {'后足吸附位置':<15}\n")
    f.write("-" * 150 + "\n")
    
    # 数据行
    for state in states_list:
        f.write(f"{state['step']:<6} {state['state_code']:<10} {state['front_magnet_status']:<10} {state['rear_magnet_status']:<10} "
                f"{str(state['front_position']):<15} {str(state['rear_position']):<15} {state['front_magnet_height']:<10.1f} "
                f"{state['rear_magnet_height']:<10.1f} {state['linear_motor_pos']:<10.1f} {str(state['front_magnet_attach_pos']):<15} {str(state['rear_magnet_attach_pos']):<15}\n")

print("机器人运动状态详细记录表（考虑磁铁高度控制版）已生成完成")
print(f"总共记录了 {len(states_list)} 个状态")
print("\n状态码说明：")
print("六位数字状态码：ω1ω2ω3vm1m2")
print("- ω1, ω2, ω3: 旋转电机状态 (0=停止, 1=正转, 2=反转)")
print("- v: 直线电机状态 (0=停止, 1=正转, 2=反转)")
print("- m1, m2: 电磁铁状态 (0=脱附, 1=吸附)")
print("\n磁铁高度控制：")
print("- 脱附状态：磁铁抬高至", lift_height, "mm（高出基础高度5mm以避免摩擦）")
print("- 吸附状态：磁铁降至基础高度", base_height, "mm（贴紧壁面确保良好接触）")
print("\n表格格式说明：")
print(f"{'步数':<6} {'状态码':<10} {'前足磁铁':<10} {'后足磁铁':<10} {'前足位置':<15} {'后足位置':<15} {'前磁铁高度':<10} {'后磁铁高度':<10} {'直线电机':<10} {'前足吸附位置':<15} {'后足吸附位置':<15}")
print("-" * 150)

# 显示前几行作为示例
print("\n前10个状态示例：")
for i, state in enumerate(states_list[:10]):
    print(f"{state['step']:<6} {state['state_code']:<10} {state['front_magnet_status']:<10} {state['rear_magnet_status']:<10} "
          f"{str(state['front_position']):<15} {str(state['rear_position']):<15} {state['front_magnet_height']:<10.1f} "
          f"{state['rear_magnet_height']:<10.1f} {state['linear_motor_pos']:<10.1f} {str(state['front_magnet_attach_pos']):<15} {str(state['rear_magnet_attach_pos']):<15}")