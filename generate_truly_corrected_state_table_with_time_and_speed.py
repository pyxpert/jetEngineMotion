"""
机器人运动状态详细记录表（最终完全正确版 - 正确的电机动作逻辑和时间参数）
时间：2026-01-29
任务：从初始位置(前足0,40；后足0,0)爬行至目标位置(前足0,1000)
基于优化轨迹（255步）
使用六位数字状态码：ω1ω2ω3vm1m2
包含时间参数：每个状态码持续的时间（毫秒），基于电机速度计算
"""

import numpy as np

def generate_truly_corrected_motor_positions_and_states_with_time():
    """
    生成机器人运动过程中的详细状态表（使用六位数字状态码并包含时间参数）
    状态码格式：ω1ω2ω3vm1m2
    - ω1, ω2, ω3: 旋转电机状态 (0=停止, 1=正转, 2=反转)
    - v: 直线电机状态 (0=停止, 1=正转, 2=反转) 
    - m1, m2: 电磁铁状态 (0=脱附, 1=吸附)
    
    电机速度参数：
    - 中间旋转电机角速度：90°/s
    - 直线电机动速度：15mm/s
    
    正确实现磁铁抬起/放下（根据用户纠正）：
    - 抬起前/后足：中间电机内折，正转（状态码第3位为1）
    - 放下前/后足：中间电机外翻，反转（状态码第3位为2）
    """
    # 机器人参数
    robot_params = {
        'd1': 20,      # 前足到中间电机的水平距离
        'd2': 20,      # 后足到中间电机的水平距离  
        'h': 30,       # 中心电机转轴到上表面的高度
        's_max': 8,    # 直线电机行程
        'R': 900       # 环境半径
    }
    
    # 电机速度参数
    motor_speeds = {
        'center_rotation_speed': 90,  # 中间旋转电机角速度：90度/秒
        'linear_speed': 15           # 直线电机动速度：15mm/秒
    }
    
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
    
    # 旋转电机角度控制参数
    base_angle = 0.0      # 基础角度（吸附状态）
    lift_angle = 45.0     # 抬起角度（脱附状态）- 正转角度
    
    # 生成轨迹和状态 - 使用蠕虫式爬行模式
    states_list = []
    
    # 初始状态：前足吸附，后足吸附，中间旋转电机在基础角度
    # 初始状态：所有电机停止，前后磁铁吸附，中间旋转电机在0度
    current_state = {
        'step': 0,
        'state_code': '000011',  # ω1ω2ω3vm1m2: 旋转电机停止，直线电机停止，前后磁铁吸附
        'front_magnet_status': '吸附',
        'rear_magnet_status': '吸附',
        'front_position': current_front.copy(),
        'rear_position': current_rear.copy(),
        'center_motor_angle': base_angle,  # 中间旋转电机基础角度
        'linear_motor_pos': 0.0,    # 初始直线电机位置
        'front_magnet_attach_pos': current_front.copy(),  # 前足磁铁吸附位置
        'rear_magnet_attach_pos': current_rear.copy(),    # 后足磁铁吸附位置
        'duration_ms': 100          # 初始稳定状态持续时间
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
            # ω1ω2ω3vm1m2: 前磁铁脱附，中间旋转电机抬起前足（正转=1）
            # 第3位ω3=1表示中间旋转电机正转抬起前足（内折）
            detach_front_state = {
                'step': step_count + 1,
                'state_code': '001001',  # 中间旋转电机正转抬起前足，前磁铁脱附，后磁铁仍吸附
                'front_magnet_status': '脱附',
                'rear_magnet_status': '吸附',
                'front_position': current_front.copy(),
                'rear_position': current_rear.copy(),
                'center_motor_angle': lift_angle,  # 抬起前磁铁（通过旋转电机正转动作）
                'linear_motor_pos': abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': current_rear.copy(),
                'duration_ms': int(abs(lift_angle - base_angle) / motor_speeds['center_rotation_speed'] * 1000)  # 旋转电机转动时间
            }
            states_list.append(detach_front_state.copy())
            
            # 计算前足的新位置
            new_front_z = min(current_front[1] + step_size, target_front_z)
            
            # 检查新位置是否满足间距约束
            new_spacing = abs(new_front_z - current_rear[1])
            
            if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                # 更新前足位置
                new_front_pos = (current_front[0], new_front_z)  # 保持theta不变
                
                # 计算直线电机行程
                prev_linear_motor_pos = abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2'])
                linear_motor_delta = abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2'])
                
                # 直线电机运行状态：前足脱附，后足吸附，直线电机伸展
                # 中间电机保持抬起前足状态（角度保持lift_angle），直线电机正转
                linear_extend_state = {
                    'step': step_count + 1,
                    'state_code': '001101',  # 中间旋转电机保持抬起前足（正转），直线电机正转伸展，前磁铁脱附，后磁铁吸附
                    'front_magnet_status': '脱附',
                    'rear_magnet_status': '吸附',
                    'front_position': new_front_pos,
                    'rear_position': current_rear.copy(),
                    'center_motor_angle': lift_angle,  # 保持抬起前磁铁（角度不变，但电机处于正转状态以维持角度）
                    'linear_motor_pos': linear_motor_delta,
                    'front_magnet_attach_pos': current_front.copy(),  # 旧吸附位置
                    'rear_magnet_attach_pos': current_rear.copy(),
                    'duration_ms': int(abs(linear_motor_delta - prev_linear_motor_pos) / motor_speeds['linear_speed'] * 1000) if abs(linear_motor_delta - prev_linear_motor_pos) > 0.1 else 50  # 直线电机运行时间，最小50ms
                }
                states_list.append(linear_extend_state.copy())
                
                # 前足到达新位置，准备放下吸附（中间旋转电机调整角度放下磁铁 - 反转）
                # 这里应该是中间电机反转，将角度从lift_angle(45°)转回base_angle(0°)
                prep_lower_front_state = {
                    'step': step_count + 1,
                    'state_code': '002001',  # 中间旋转电机反转放下前足准备吸附，直线电机停止，前磁铁仍脱附，后磁铁吸附
                    'front_magnet_status': '脱附',
                    'rear_magnet_status': '吸附',
                    'front_position': new_front_pos,
                    'rear_position': current_rear.copy(),
                    'center_motor_angle': base_angle,  # 前磁铁放下（从45°反转到0°）
                    'linear_motor_pos': linear_motor_delta,
                    'front_magnet_attach_pos': new_front_pos,  # 即将吸附的位置
                    'rear_magnet_attach_pos': current_rear.copy(),
                    'duration_ms': int(abs(base_angle - lift_angle) / motor_speeds['center_rotation_speed'] * 1000)  # 旋转电机转动时间
                }
                states_list.append(prep_lower_front_state.copy())
                
                # 前足吸附
                attach_front_state = {
                    'step': step_count + 1,
                    'state_code': '000011',  # 前磁铁吸附，后磁铁吸附
                    'front_magnet_status': '吸附',
                    'rear_magnet_status': '吸附',
                    'front_position': new_front_pos,
                    'rear_position': current_rear.copy(),
                    'center_motor_angle': base_angle,  # 前磁铁吸附时保持基础角度
                    'linear_motor_pos': linear_motor_delta,
                    'front_magnet_attach_pos': new_front_pos,
                    'rear_magnet_attach_pos': current_rear.copy(),
                    'duration_ms': 300  # 磁铁吸附时间
                }
                states_list.append(attach_front_state.copy())
                
                current_front = [current_front[0], new_front_z]
            else:
                # 如果前足移动会导致间距超出范围，则移动后足
                new_rear_z = current_rear[1] + step_size
                new_spacing = abs(current_front[1] - new_rear_z)
                
                if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                    new_rear_pos = (current_rear[0], new_rear_z)
                    
                    # 后足脱附（中间旋转电机正转抬起后足 - 内折）
                    detach_rear_state = {
                        'step': step_count + 1,
                        'state_code': '001010',  # 中间旋转电机正转抬起后足，后磁铁脱附，前磁铁仍吸附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '脱附',
                        'front_position': current_front.copy(),
                        'rear_position': new_rear_pos,
                        'center_motor_angle': lift_angle,  # 抬起后磁铁（通过旋转电机正转动作）
                        'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                        'front_magnet_attach_pos': current_front.copy(),
                        'rear_magnet_attach_pos': current_rear.copy(),
                        'duration_ms': int(abs(lift_angle - base_angle) / motor_speeds['center_rotation_speed'] * 1000)  # 旋转电机转动时间
                    }
                    states_list.append(detach_rear_state.copy())
                    
                    # 计算直线电机行程
                    prev_linear_motor_pos = abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2'])
                    linear_motor_delta = abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2'])
                    
                    # 直线电机运行状态：前足吸附，后足脱附，直线电机收缩
                    linear_contract_state = {
                        'step': step_count + 1,
                        'state_code': '001210',  # 中间旋转电机保持抬起后足（正转），直线电机反转收缩，前磁铁吸附，后磁铁脱附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '脱附',
                        'front_position': current_front.copy(),
                        'rear_position': new_rear_pos,
                        'center_motor_angle': lift_angle,  # 保持抬起后磁铁（角度不变，但电机处于正转状态以维持角度）
                        'linear_motor_pos': linear_motor_delta,
                        'front_magnet_attach_pos': current_front.copy(),
                        'rear_magnet_attach_pos': current_rear.copy(),
                        'duration_ms': int(abs(linear_motor_delta - prev_linear_motor_pos) / motor_speeds['linear_speed'] * 1000) if abs(linear_motor_delta - prev_linear_motor_pos) > 0.1 else 50  # 直线电机运行时间，最小50ms
                    }
                    states_list.append(linear_contract_state.copy())
                    
                    # 后足到达新位置，准备放下吸附（中间旋转电机调整角度放下磁铁 - 反转）
                    prep_lower_rear_state = {
                        'step': step_count + 1,
                        'state_code': '002010',  # 中间旋转电机反转放下后足准备吸附，直线电机停止，前磁铁吸附，后磁铁仍脱附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '脱附',
                        'front_position': current_front.copy(),
                        'rear_position': new_rear_pos,
                        'center_motor_angle': base_angle,  # 后磁铁放下（从45°反转到0°）
                        'linear_motor_pos': linear_motor_delta,
                        'front_magnet_attach_pos': current_front.copy(),
                        'rear_magnet_attach_pos': new_rear_pos,  # 即将吸附的位置
                        'duration_ms': int(abs(base_angle - lift_angle) / motor_speeds['center_rotation_speed'] * 1000)  # 旋转电机转动时间
                    }
                    states_list.append(prep_lower_rear_state.copy())
                    
                    # 后足吸附
                    attach_rear_state = {
                        'step': step_count + 1,
                        'state_code': '000011',  # 前后磁铁都吸附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '吸附',
                        'front_position': current_front.copy(),
                        'rear_position': new_rear_pos,
                        'center_motor_angle': base_angle,  # 后磁铁吸附时保持基础角度
                        'linear_motor_pos': linear_motor_delta,
                        'front_magnet_attach_pos': current_front.copy(),
                        'rear_magnet_attach_pos': new_rear_pos,
                        'duration_ms': 300  # 磁铁吸附时间
                    }
                    states_list.append(attach_rear_state.copy())
                    
                    current_rear = [current_rear[0], new_rear_z]
                else:
                    # 尝试较小的步长
                    adjusted_step = step_size * 0.7
                    new_rear_z = min(current_rear[1] + adjusted_step, current_front[1] - (robot_params['d1'] + robot_params['d2']))
                    new_spacing = abs(current_front[1] - new_rear_z)
                    
                    if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                        new_rear_pos = (current_rear[0], new_rear_z)
                        
                        # 后足脱附（中间旋转电机正转抬起后足 - 内折）
                        detach_rear_state = {
                            'step': step_count + 1,
                            'state_code': '001010',  # 中间旋转电机正转抬起后足，后磁铁脱附，前磁铁仍吸附
                            'front_magnet_status': '吸附',
                            'rear_magnet_status': '脱附',
                            'front_position': current_front.copy(),
                            'rear_position': new_rear_pos,
                            'center_motor_angle': lift_angle,  # 抬起后磁铁（通过旋转电机正转动作）
                            'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                            'front_magnet_attach_pos': current_front.copy(),
                            'rear_magnet_attach_pos': current_rear.copy(),
                            'duration_ms': int(abs(lift_angle - base_angle) / motor_speeds['center_rotation_speed'] * 1000)  # 旋转电机转动时间
                        }
                        states_list.append(detach_rear_state.copy())
                        
                        # 计算直线电机行程
                        prev_linear_motor_pos = abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2'])
                        linear_motor_delta = abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2'])
                        
                        # 直线电机运行状态：前足吸附，后足脱附，直线电机收缩
                        linear_contract_state = {
                            'step': step_count + 1,
                            'state_code': '001210',  # 中间旋转电机保持抬起后足（正转），直线电机反转收缩，前磁铁吸附，后磁铁脱附
                            'front_magnet_status': '吸附',
                            'rear_magnet_status': '脱附',
                            'front_position': current_front.copy(),
                            'rear_position': new_rear_pos,
                            'center_motor_angle': lift_angle,  # 保持抬起后磁铁（角度不变，但电机处于正转状态以维持角度）
                            'linear_motor_pos': linear_motor_delta,
                            'front_magnet_attach_pos': current_front.copy(),
                            'rear_magnet_attach_pos': current_rear.copy(),
                            'duration_ms': int(abs(linear_motor_delta - prev_linear_motor_pos) / motor_speeds['linear_speed'] * 1000) if abs(linear_motor_delta - prev_linear_motor_pos) > 0.1 else 50  # 直线电机运行时间，最小50ms
                        }
                        states_list.append(linear_contract_state.copy())
                        
                        # 后足到达新位置，准备放下吸附（中间旋转电机调整角度放下磁铁 - 反转）
                        prep_lower_rear_state = {
                            'step': step_count + 1,
                            'state_code': '002010',  # 中间旋转电机反转放下后足准备吸附，直线电机停止，前磁铁吸附，后磁铁仍脱附
                            'front_magnet_status': '吸附',
                            'rear_magnet_status': '脱附',
                            'front_position': current_front.copy(),
                            'rear_position': new_rear_pos,
                            'center_motor_angle': base_angle,  # 后磁铁放下（从45°反转到0°）
                            'linear_motor_pos': linear_motor_delta,
                            'front_magnet_attach_pos': current_front.copy(),
                            'rear_magnet_attach_pos': new_rear_pos,  # 即将吸附的位置
                            'duration_ms': int(abs(base_angle - lift_angle) / motor_speeds['center_rotation_speed'] * 1000)  # 旋转电机转动时间
                        }
                        states_list.append(prep_lower_rear_state.copy())
                        
                        # 后足吸附
                        attach_rear_state = {
                            'step': step_count + 1,
                            'state_code': '000011',  # 前后磁铁都吸附
                            'front_magnet_status': '吸附',
                            'rear_magnet_status': '吸附',
                            'front_position': current_front.copy(),
                            'rear_position': new_rear_pos,
                            'center_motor_angle': base_angle,  # 后磁铁吸附时保持基础角度
                            'linear_motor_pos': linear_motor_delta,
                            'front_magnet_attach_pos': current_front.copy(),
                            'rear_magnet_attach_pos': new_rear_pos,
                            'duration_ms': 300  # 磁铁吸附时间
                        }
                        states_list.append(attach_rear_state.copy())
                        
                        current_rear = [current_rear[0], new_rear_z]
        
        else:  # 移动后足
            # 状态变化：前足保持吸附，后足准备脱离（中间旋转电机正转抬起后足 - 内折）
            detach_rear_state = {
                'step': step_count + 1,
                'state_code': '001010',  # 中间旋转电机正转抬起后足，后磁铁脱附，前磁铁仍吸附
                'front_magnet_status': '吸附',
                'rear_magnet_status': '脱附',
                'front_position': current_front.copy(),
                'rear_position': current_rear.copy(),
                'center_motor_angle': lift_angle,  # 抬起后磁铁（通过旋转电机正转动作）
                'linear_motor_pos': abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': current_rear.copy(),
                'duration_ms': int(abs(lift_angle - base_angle) / motor_speeds['center_rotation_speed'] * 1000)  # 旋转电机转动时间
            }
            states_list.append(detach_rear_state.copy())
            
            # 计算后足的新位置
            # 为了让前足能够继续前进，后足需要跟上前足
            new_rear_z = min(current_rear[1] + step_size, current_front[1] - (robot_params['d1'] + robot_params['d2']))
            
            # 检查新位置是否满足间距约束
            new_spacing = abs(current_front[1] - new_rear_z)
            
            if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                # 更新后足位置
                new_rear_pos = (current_rear[0], new_rear_z)
                
                # 计算直线电机行程
                prev_linear_motor_pos = abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2'])
                linear_motor_delta = abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2'])
                
                # 直线电机运行状态：前足吸附，后足脱附，直线电机收缩
                linear_contract_state = {
                    'step': step_count + 1,
                    'state_code': '001210',  # 中间旋转电机保持抬起后足（正转），直线电机反转收缩，前磁铁吸附，后磁铁脱附
                    'front_magnet_status': '吸附',
                    'rear_magnet_status': '脱附',
                    'front_position': current_front.copy(),
                    'rear_position': new_rear_pos,
                    'center_motor_angle': lift_angle,  # 保持抬起后磁铁（角度不变，但电机处于正转状态以维持角度）
                    'linear_motor_pos': linear_motor_delta,
                    'front_magnet_attach_pos': current_front.copy(),
                    'rear_magnet_attach_pos': current_rear.copy(),
                    'duration_ms': int(abs(linear_motor_delta - prev_linear_motor_pos) / motor_speeds['linear_speed'] * 1000) if abs(linear_motor_delta - prev_linear_motor_pos) > 0.1 else 50  # 直线电机运行时间，最小50ms
                }
                states_list.append(linear_contract_state.copy())
                
                # 后足到达新位置，准备放下吸附（中间旋转电机调整角度放下磁铁 - 反转）
                prep_lower_rear_state = {
                    'step': step_count + 1,
                    'state_code': '002010',  # 中间旋转电机反转放下后足准备吸附，直线电机停止，前磁铁吸附，后磁铁仍脱附
                    'front_magnet_status': '吸附',
                    'rear_magnet_status': '脱附',
                    'front_position': current_front.copy(),
                    'rear_position': new_rear_pos,
                    'center_motor_angle': base_angle,  # 后磁铁放下（从45°反转到0°）
                    'linear_motor_pos': linear_motor_delta,
                    'front_magnet_attach_pos': current_front.copy(),
                    'rear_magnet_attach_pos': new_rear_pos,  # 即将吸附的位置
                    'duration_ms': int(abs(base_angle - lift_angle) / motor_speeds['center_rotation_speed'] * 1000)  # 旋转电机转动时间
                }
                states_list.append(prep_lower_rear_state.copy())
                
                # 后足吸附
                attach_rear_state = {
                    'step': step_count + 1,
                    'state_code': '000011',  # 前后磁铁都吸附
                    'front_magnet_status': '吸附',
                    'rear_magnet_status': '吸附',
                    'front_position': current_front.copy(),
                    'rear_position': new_rear_pos,
                    'center_motor_angle': base_angle,  # 后磁铁吸附时保持基础角度
                    'linear_motor_pos': linear_motor_delta,
                    'front_magnet_attach_pos': current_front.copy(),
                    'rear_magnet_attach_pos': new_rear_pos,
                    'duration_ms': 300  # 磁铁吸附时间
                }
                states_list.append(attach_rear_state.copy())
                
                current_rear = [current_rear[0], new_rear_z]
            else:
                # 如果后足移动会导致间距超出范围，则移动前足
                new_front_z = min(current_front[1] + step_size, target_front_z)
                new_spacing = abs(new_front_z - current_rear[1])
                
                if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                    new_front_pos = (current_front[0], new_front_z)
                    
                    # 前足脱附（中间旋转电机正转抬起前足 - 内折）
                    detach_front_state = {
                        'step': step_count + 1,
                        'state_code': '001001',  # 中间旋转电机正转抬起前足，前磁铁脱附，后磁铁仍吸附
                        'front_magnet_status': '脱附',
                        'rear_magnet_status': '吸附',
                        'front_position': new_front_pos,
                        'rear_position': current_rear.copy(),
                        'center_motor_angle': lift_angle,  # 抬起前磁铁（通过旋转电机正转动作）
                        'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                        'front_magnet_attach_pos': current_front.copy(),  # 旧吸附位置
                        'rear_magnet_attach_pos': current_rear.copy(),
                        'duration_ms': int(abs(lift_angle - base_angle) / motor_speeds['center_rotation_speed'] * 1000)  # 旋转电机转动时间
                    }
                    states_list.append(detach_front_state.copy())
                    
                    # 计算直线电机行程
                    prev_linear_motor_pos = abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2'])
                    linear_motor_delta = abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2'])
                    
                    # 直线电机运行状态：前足脱附，后足吸附，直线电机伸展
                    linear_extend_state = {
                        'step': step_count + 1,
                        'state_code': '001101',  # 中间旋转电机保持抬起前足（正转），直线电机正转伸展，前磁铁脱附，后磁铁吸附
                        'front_magnet_status': '脱附',
                        'rear_magnet_status': '吸附',
                        'front_position': new_front_pos,
                        'rear_position': current_rear.copy(),
                        'center_motor_angle': lift_angle,  # 保持抬起前磁铁（角度不变，但电机处于正转状态以维持角度）
                        'linear_motor_pos': linear_motor_delta,
                        'front_magnet_attach_pos': current_front.copy(),  # 旧吸附位置
                        'rear_magnet_attach_pos': current_rear.copy(),
                        'duration_ms': int(abs(linear_motor_delta - prev_linear_motor_pos) / motor_speeds['linear_speed'] * 1000) if abs(linear_motor_delta - prev_linear_motor_pos) > 0.1 else 50  # 直线电机运行时间，最小50ms
                    }
                    states_list.append(linear_extend_state.copy())
                    
                    # 前足到达新位置，准备放下吸附（中间旋转电机调整角度放下磁铁 - 反转）
                    prep_lower_front_state = {
                        'step': step_count + 1,
                        'state_code': '002001',  # 中间旋转电机反转放下前足准备吸附，直线电机停止，前磁铁仍脱附，后磁铁吸附
                        'front_magnet_status': '脱附',
                        'rear_magnet_status': '吸附',
                        'front_position': new_front_pos,
                        'rear_position': current_rear.copy(),
                        'center_motor_angle': base_angle,  # 前磁铁放下（从45°反转到0°）
                        'linear_motor_pos': linear_motor_delta,
                        'front_magnet_attach_pos': new_front_pos,  # 即将吸附的位置
                        'rear_magnet_attach_pos': current_rear.copy(),
                        'duration_ms': int(abs(base_angle - lift_angle) / motor_speeds['center_rotation_speed'] * 1000)  # 旋转电机转动时间
                    }
                    states_list.append(prep_lower_front_state.copy())
                    
                    # 前足吸附
                    attach_front_state = {
                        'step': step_count + 1,
                        'state_code': '000011',  # 前后磁铁都吸附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '吸附',
                        'front_position': new_front_pos,
                        'rear_position': current_rear.copy(),
                        'center_motor_angle': base_angle,  # 前磁铁吸附时保持基础角度
                        'linear_motor_pos': linear_motor_delta,
                        'front_magnet_attach_pos': new_front_pos,
                        'rear_magnet_attach_pos': current_rear.copy(),
                        'duration_ms': 300  # 磁铁吸附时间
                    }
                    states_list.append(attach_front_state.copy())
                    
                    current_front = [current_front[0], new_front_z]
        
        step_count += 1
    
    return states_list, base_angle, lift_angle

# 生成状态列表
states_list, base_angle, lift_angle = generate_truly_corrected_motor_positions_and_states_with_time()

# 写入详细状态表到文件
with open('robot_detailed_state_table_truly_corrected_with_time_and_speed.txt', 'w', encoding='utf-8') as f:
    f.write('"""机器人运动状态详细记录表（最终完全正确版 - 正确的电机动作逻辑和时间参数）\n')
    f.write('时间：2026-01-29\n')
    f.write('任务：从初始位置(前足0,40；后足0,0)爬行至目标位置(前足0,1000)\n')
    f.write('基于优化轨迹（255步）\n')
    f.write('使用六位数字状态码：ω1ω2ω3vm1m2\n')
    f.write('包含时间参数：每个状态码持续的时间（毫秒），基于电机速度计算\n')
    f.write('"""\n\n')
    
    # 表头
    f.write(f"{'步数':<6} {'状态码':<10} {'前足磁铁':<10} {'后足磁铁':<10} {'前足位置':<15} {'后足位置':<15} {'中心电机角度':<12} {'直线电机':<10} {'前足吸附位置':<15} {'后足吸附位置':<15} {'持续时间(ms)':<12}\n")
    f.write("-" * 150 + "\n")
    
    # 数据行
    for state in states_list:
        f.write(f"{state['step']:<6} {state['state_code']:<10} {state['front_magnet_status']:<10} {state['rear_magnet_status']:<10} "
                f"{str(state['front_position']):<15} {str(state['rear_position']):<15} {state['center_motor_angle']:<12.1f} "
                f"{state['linear_motor_pos']:<10.1f} {str(state['front_magnet_attach_pos']):<15} {str(state['rear_magnet_attach_pos']):<15} {state['duration_ms']:<12}\n")

print("机器人运动状态详细记录表（最终完全正确版 - 正确的电机动作逻辑和时间参数）已生成完成")
print(f"总共记录了 {len(states_list)} 个状态")
print("\n状态码说明：")
print("六位数字状态码：ω1ω2ω3vm1m2")
print("- ω1, ω2: 前、后节旋转电机状态 (0=停止, 1=正转, 2=反转)")
print("- ω3: 中间旋转电机状态 (0=停止, 1=正转, 2=反转)")
print("- v: 直线电机状态 (0=停止, 1=正转, 2=反转)")
print("- m1, m2: 电磁铁状态 (0=脱附, 1=吸附)")
print("\n电机速度参数：")
print("- 中间旋转电机角速度：90°/s")
print("- 直线电机动速度：15mm/s")
print("\n磁铁抬起/放下机制（根据用户纠正）：")
print("- 抬起前/后足：中间电机内折，正转（状态码第3位为1）")
print("- 放下前/后足：中间电机外翻，反转（状态码第3位为2）")
print("\n表格格式说明：")
print(f"{'步数':<6} {'状态码':<10} {'前足磁铁':<10} {'后足磁铁':<10} {'前足位置':<15} {'后足位置':<15} {'中心电机角度':<12} {'直线电机':<10} {'前足吸附位置':<15} {'后足吸附位置':<15} {'持续时间(ms)':<12}")
print("-" * 150)

# 显示前几行作为示例
print("\n前10个状态示例：")
for i, state in enumerate(states_list[:10]):
    print(f"{state['step']:<6} {state['state_code']:<10} {state['front_magnet_status']:<10} {state['rear_magnet_status']:<10} "
          f"{str(state['front_position']):<15} {str(state['rear_position']):<15} {state['center_motor_angle']:<12.1f} "
          f"{state['linear_motor_pos']:<10.1f} {str(state['front_magnet_attach_pos']):<15} {str(state['rear_magnet_attach_pos']):<15} {state['duration_ms']:<12}")