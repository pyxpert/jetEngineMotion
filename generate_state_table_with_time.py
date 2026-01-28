"""
机器人运动状态详细记录表（带时间参数）
时间：2026-01-29
任务：从初始位置(前足0,40；后足0,0)爬行至目标位置(前足0,1000)
基于优化轨迹（255步）
使用六位数字状态码：ω1ω2ω3vm1m2
增加时间参数：每个状态码持续的时间（毫秒）
"""

import numpy as np

def generate_motor_positions_and_states_with_time():
    """
    生成机器人运动过程中的详细状态表（使用六位数字状态码并包含时间参数）
    状态码格式：ω1ω2ω3vm1m2
    - ω1, ω2, ω3: 旋转电机状态 (0=停止, 1=正转, 2=反转)
    - v: 直线电机状态 (0=停止, 1=正转, 2=反转) 
    - m1, m2: 电磁铁状态 (0=脱附, 1=吸附)
    
    时间参数：
    - 旋转电机启动/停止：50ms
    - 直线电机启动/停止：100ms
    - 中心旋转电机动作（抬起/放下磁铁）：200ms
    - 磁铁脱附过程：300ms
    - 磁铁吸附过程：300ms
    - 直线电机运行时间与行程相关：每毫米50ms
    - 旋转电机运行时间与角度相关：每度2ms
    """
    # 机器人参数
    robot_params = {
        'd1': 20,      # 前足到中间电机的水平距离
        'd2': 20,      # 后足到中间电机的水平距离  
        'h': 30,       # 中心电机转轴到上表面的高度
        's_max': 8,    # 直线电机行程
        'R': 900       # 环境半径
    }
    
    # 时间参数
    time_params = {
        'rotation_motor_switch': 50,      # 旋转电机启动/停止时间(ms)
        'linear_motor_switch': 100,       # 直线电机启动/停止时间(ms)
        'center_motor_action': 200,       # 中心旋转电机动作时间(ms)
        'magnet_detach': 300,             # 磁铁脱附过程时间(ms)
        'magnet_attach': 300,             # 磁铁吸附过程时间(ms)
        'linear_motor_per_mm': 50,        # 直线电机每毫米运行时间(ms/mm)
        'rotation_motor_per_degree': 2    # 旋转电机每度运行时间(ms/degree)
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
    lift_angle = 45.0     # 抬起角度（脱附状态）
    
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
            # ω1ω2ω3vm1m2: 前磁铁脱附，中间旋转电机抬起前足
            detach_state = {
                'step': step_count + 1,
                'state_code': '001001',  # 中间旋转电机正转抬起前足，前磁铁脱附，后磁铁仍吸附
                'front_magnet_status': '脱附',
                'rear_magnet_status': '吸附',
                'front_position': current_front.copy(),
                'rear_position': current_rear.copy(),
                'center_motor_angle': lift_angle,  # 抬起前磁铁（通过旋转电机动作）
                'linear_motor_pos': abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': current_rear.copy(),
                'duration_ms': time_params['center_motor_action'] + time_params['magnet_detach']  # 中心电机动作+磁铁脱附时间
            }
            states_list.append(detach_state.copy())
            
            # 计算前足的新位置
            new_front_z = min(current_front[1] + step_size, target_front_z)
            
            # 检查新位置是否满足间距约束
            new_spacing = abs(new_front_z - current_rear[1])
            
            if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                # 更新前足位置
                new_front_pos = (current_front[0], new_front_z)  # 保持theta不变
                
                # 计算直线电机行程
                linear_motor_delta = abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2'])
                
                # 折叠状态：前足脱附，后足吸附，直线电机收缩
                # ω1ω2ω3vm1m2: 直线电机收缩(2=反转)，中间旋转电机保持抬起状态
                fold_state = {
                    'step': step_count + 1,
                    'state_code': '002201',  # 中间旋转电机反转放下前足准备吸附，直线电机收缩，前磁铁脱附，后磁铁吸附
                    'front_magnet_status': '脱附',
                    'rear_magnet_status': '吸附',
                    'front_position': new_front_pos,
                    'rear_position': current_rear.copy(),
                    'center_motor_angle': base_angle,  # 准备放下前磁铁（通过旋转电机动作）
                    'linear_motor_pos': linear_motor_delta,
                    'front_magnet_attach_pos': current_front.copy(),  # 旧吸附位置
                    'rear_magnet_attach_pos': current_rear.copy(),
                    'duration_ms': time_params['linear_motor_switch'] + max(abs(linear_motor_delta) * time_params['linear_motor_per_mm'], 100)  # 直线电机运行时间
                }
                states_list.append(fold_state.copy())
                
                # 前足到达新位置，准备吸附（中间旋转电机调整角度放下磁铁）
                prep_attach_state = {
                    'step': step_count + 1,
                    'state_code': '000001',  # 中间旋转电机停止，直线电机停止，前磁铁仍脱附，后磁铁吸附
                    'front_magnet_status': '脱附',
                    'rear_magnet_status': '吸附',
                    'front_position': new_front_pos,
                    'rear_position': current_rear.copy(),
                    'center_motor_angle': base_angle,  # 前磁铁放下（通过旋转电机动作）
                    'linear_motor_pos': linear_motor_delta,
                    'front_magnet_attach_pos': new_front_pos,  # 即将吸附的位置
                    'rear_magnet_attach_pos': current_rear.copy(),
                    'duration_ms': time_params['center_motor_action']  # 中心电机放下磁铁时间
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
                    'center_motor_angle': base_angle,  # 前磁铁吸附时保持基础角度
                    'linear_motor_pos': linear_motor_delta,
                    'front_magnet_attach_pos': new_front_pos,
                    'rear_magnet_attach_pos': current_rear.copy(),
                    'duration_ms': time_params['magnet_attach']  # 磁铁吸附时间
                }
                states_list.append(attach_state.copy())
                
                current_front = [current_front[0], new_front_z]
            else:
                # 如果前足移动会导致间距超出范围，则移动后足
                new_rear_z = current_rear[1] + step_size
                new_spacing = abs(current_front[1] - new_rear_z)
                
                if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                    new_rear_pos = (current_rear[0], new_rear_z)
                    
                    # 后足脱附（中间旋转电机抬起后足）
                    detach_state = {
                        'step': step_count + 1,
                        'state_code': '002010',  # 中间旋转电机反转抬起后足，后磁铁脱附，前磁铁仍吸附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '脱附',
                        'front_position': current_front.copy(),
                        'rear_position': new_rear_pos,
                        'center_motor_angle': -lift_angle,  # 抬起后磁铁（通过旋转电机反向动作）
                        'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                        'front_magnet_attach_pos': current_front.copy(),
                        'rear_magnet_attach_pos': current_rear.copy(),
                        'duration_ms': time_params['center_motor_action'] + time_params['magnet_detach']  # 中心电机动作+磁铁脱附时间
                    }
                    states_list.append(detach_state.copy())
                    
                    # 计算直线电机行程
                    linear_motor_delta = abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2'])
                    
                    # 扩展状态：前足吸附，后足脱附，直线电机伸展
                    extend_state = {
                        'step': step_count + 1,
                        'state_code': '001110',  # 中间旋转电机正转放下后足准备吸附，直线电机伸展，前磁铁吸附，后磁铁脱附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '脱附',
                        'front_position': current_front.copy(),
                        'rear_position': new_rear_pos,
                        'center_motor_angle': base_angle,  # 准备放下后磁铁（通过旋转电机动作）
                        'linear_motor_pos': linear_motor_delta,
                        'front_magnet_attach_pos': current_front.copy(),
                        'rear_magnet_attach_pos': current_rear.copy(),
                        'duration_ms': time_params['linear_motor_switch'] + max(abs(linear_motor_delta) * time_params['linear_motor_per_mm'], 100)  # 直线电机运行时间
                    }
                    states_list.append(extend_state.copy())
                    
                    # 后足到达新位置，准备吸附（中间旋转电机调整角度放下磁铁）
                    prep_attach_state = {
                        'step': step_count + 1,
                        'state_code': '000010',  # 中间旋转电机停止，直线电机停止，前磁铁吸附，后磁铁仍脱附
                        'front_magnet_status': '吸附',
                        'rear_magnet_status': '脱附',
                        'front_position': current_front.copy(),
                        'rear_position': new_rear_pos,
                        'center_motor_angle': base_angle,  # 后磁铁放下（通过旋转电机动作）
                        'linear_motor_pos': linear_motor_delta,
                        'front_magnet_attach_pos': current_front.copy(),
                        'rear_magnet_attach_pos': new_rear_pos,  # 即将吸附的位置
                        'duration_ms': time_params['center_motor_action']  # 中心电机放下磁铁时间
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
                        'center_motor_angle': base_angle,  # 后磁铁吸附时保持基础角度
                        'linear_motor_pos': linear_motor_delta,
                        'front_magnet_attach_pos': current_front.copy(),
                        'rear_magnet_attach_pos': new_rear_pos,
                        'duration_ms': time_params['magnet_attach']  # 磁铁吸附时间
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
                        
                        # 后足脱附（中间旋转电机抬起后足）
                        detach_state = {
                            'step': step_count + 1,
                            'state_code': '002010',  # 中间旋转电机反转抬起后足，后磁铁脱附，前磁铁仍吸附
                            'front_magnet_status': '吸附',
                            'rear_magnet_status': '脱附',
                            'front_position': current_front.copy(),
                            'rear_position': new_rear_pos,
                            'center_motor_angle': -lift_angle,  # 抬起后磁铁（通过旋转电机反向动作）
                            'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                            'front_magnet_attach_pos': current_front.copy(),
                            'rear_magnet_attach_pos': current_rear.copy(),
                            'duration_ms': time_params['center_motor_action'] + time_params['magnet_detach']  # 中心电机动作+磁铁脱附时间
                        }
                        states_list.append(detach_state.copy())
                        
                        # 计算直线电机行程
                        linear_motor_delta = abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2'])
                        
                        # 扩展状态：前足吸附，后足脱附，直线电机伸展
                        extend_state = {
                            'step': step_count + 1,
                            'state_code': '001110',  # 中间旋转电机正转放下后足准备吸附，直线电机伸展，前磁铁吸附，后磁铁脱附
                            'front_magnet_status': '吸附',
                            'rear_magnet_status': '脱附',
                            'front_position': current_front.copy(),
                            'rear_position': new_rear_pos,
                            'center_motor_angle': base_angle,  # 准备放下后磁铁（通过旋转电机动作）
                            'linear_motor_pos': linear_motor_delta,
                            'front_magnet_attach_pos': current_front.copy(),
                            'rear_magnet_attach_pos': current_rear.copy(),
                            'duration_ms': time_params['linear_motor_switch'] + max(abs(linear_motor_delta) * time_params['linear_motor_per_mm'], 100)  # 直线电机运行时间
                        }
                        states_list.append(extend_state.copy())
                        
                        # 后足到达新位置，准备吸附（中间旋转电机调整角度放下磁铁）
                        prep_attach_state = {
                            'step': step_count + 1,
                            'state_code': '000010',  # 中间旋转电机停止，直线电机停止，前磁铁吸附，后磁铁仍脱附
                            'front_magnet_status': '吸附',
                            'rear_magnet_status': '脱附',
                            'front_position': current_front.copy(),
                            'rear_position': new_rear_pos,
                            'center_motor_angle': base_angle,  # 后磁铁放下（通过旋转电机动作）
                            'linear_motor_pos': linear_motor_delta,
                            'front_magnet_attach_pos': current_front.copy(),
                            'rear_magnet_attach_pos': new_rear_pos,  # 即将吸附的位置
                            'duration_ms': time_params['center_motor_action']  # 中心电机放下磁铁时间
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
                            'center_motor_angle': base_angle,  # 后磁铁吸附时保持基础角度
                            'linear_motor_pos': linear_motor_delta,
                            'front_magnet_attach_pos': current_front.copy(),
                            'rear_magnet_attach_pos': new_rear_pos,
                            'duration_ms': time_params['magnet_attach']  # 磁铁吸附时间
                        }
                        states_list.append(attach_state.copy())
                        
                        current_rear = [current_rear[0], new_rear_z]
        
        else:  # 移动后足
            # 状态变化：前足保持吸附，后足准备脱离（中间旋转电机抬起后足）
            detach_state = {
                'step': step_count + 1,
                'state_code': '002010',  # 中间旋转电机反转抬起后足，后磁铁脱附，前磁铁仍吸附
                'front_magnet_status': '吸附',
                'rear_magnet_status': '脱附',
                'front_position': current_front.copy(),
                'rear_position': current_rear.copy(),
                'center_motor_angle': -lift_angle,  # 抬起后磁铁（通过旋转电机反向动作）
                'linear_motor_pos': abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': current_rear.copy(),
                'duration_ms': time_params['center_motor_action'] + time_params['magnet_detach']  # 中心电机动作+磁铁脱附时间
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
                
                # 计算直线电机行程
                linear_motor_delta = abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2'])
                
                # 扩展状态：前足吸附，后足脱附，直线电机伸展
                extend_state = {
                    'step': step_count + 1,
                    'state_code': '001110',  # 中间旋转电机正转放下后足准备吸附，直线电机伸展，前磁铁吸附，后磁铁脱附
                    'front_magnet_status': '吸附',
                    'rear_magnet_status': '脱附',
                    'front_position': current_front.copy(),
                    'rear_position': new_rear_pos,
                    'center_motor_angle': base_angle,  # 准备放下后磁铁（通过旋转电机动作）
                    'linear_motor_pos': linear_motor_delta,
                    'front_magnet_attach_pos': current_front.copy(),
                    'rear_magnet_attach_pos': current_rear.copy(),
                    'duration_ms': time_params['linear_motor_switch'] + max(abs(linear_motor_delta) * time_params['linear_motor_per_mm'], 100)  # 直线电机运行时间
                }
                states_list.append(extend_state.copy())
                
                # 后足到达新位置，准备吸附（中间旋转电机调整角度放下磁铁）
                prep_attach_state = {
                    'step': step_count + 1,
                    'state_code': '000010',  # 中间旋转电机停止，直线电机停止，前磁铁吸附，后磁铁仍脱附
                    'front_magnet_status': '吸附',
                    'rear_magnet_status': '脱附',
                    'front_position': current_front.copy(),
                    'rear_position': new_rear_pos,
                    'center_motor_angle': base_angle,  # 后磁铁放下（通过旋转电机动作）
                    'linear_motor_pos': linear_motor_delta,
                    'front_magnet_attach_pos': current_front.copy(),
                    'rear_magnet_attach_pos': new_rear_pos,  # 即将吸附的位置
                    'duration_ms': time_params['center_motor_action']  # 中心电机放下磁铁时间
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
                    'center_motor_angle': base_angle,  # 后磁铁吸附时保持基础角度
                    'linear_motor_pos': linear_motor_delta,
                    'front_magnet_attach_pos': current_front.copy(),
                    'rear_magnet_attach_pos': new_rear_pos,
                    'duration_ms': time_params['magnet_attach']  # 磁铁吸附时间
                }
                states_list.append(attach_state.copy())
                
                current_rear = [current_rear[0], new_rear_z]
            else:
                # 如果后足移动会导致间距超出范围，则移动前足
                new_front_z = min(current_front[1] + step_size, target_front_z)
                new_spacing = abs(new_front_z - current_rear[1])
                
                if min(robot_params['d1'] + robot_params['d2'], new_spacing) <= robot_params['d1'] + robot_params['d2'] + robot_params['s_max']:
                    new_front_pos = (current_front[0], new_front_z)
                    
                    # 前足脱附（中间旋转电机抬起前足）
                    detach_state = {
                        'step': step_count + 1,
                        'state_code': '001001',  # 中间旋转电机正转抬起前足，前磁铁脱附，后磁铁仍吸附
                        'front_magnet_status': '脱附',
                        'rear_magnet_status': '吸附',
                        'front_position': new_front_pos,
                        'rear_position': current_rear.copy(),
                        'center_motor_angle': lift_angle,  # 抬起前磁铁（通过旋转电机动作）
                        'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                        'front_magnet_attach_pos': current_front.copy(),  # 旧吸附位置
                        'rear_magnet_attach_pos': current_rear.copy(),
                        'duration_ms': time_params['center_motor_action'] + time_params['magnet_detach']  # 中心电机动作+磁铁脱附时间
                    }
                    states_list.append(detach_state.copy())
                    
                    # 计算直线电机行程
                    linear_motor_delta = abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2'])
                    
                    # 折叠状态：前足脱附，后足吸附，直线电机收缩
                    fold_state = {
                        'step': step_count + 1,
                        'state_code': '002201',  # 中间旋转电机反转放下前足准备吸附，直线电机收缩，前磁铁脱附，后磁铁吸附
                        'front_magnet_status': '脱附',
                        'rear_magnet_status': '吸附',
                        'front_position': new_front_pos,
                        'rear_position': current_rear.copy(),
                        'center_motor_angle': base_angle,  # 准备放下前磁铁（通过旋转电机动作）
                        'linear_motor_pos': linear_motor_delta,
                        'front_magnet_attach_pos': current_front.copy(),  # 旧吸附位置
                        'rear_magnet_attach_pos': current_rear.copy(),
                        'duration_ms': time_params['linear_motor_switch'] + max(abs(linear_motor_delta) * time_params['linear_motor_per_mm'], 100)  # 直线电机运行时间
                    }
                    states_list.append(fold_state.copy())
                    
                    # 前足到达新位置，准备吸附（中间旋转电机调整角度放下磁铁）
                    prep_attach_state = {
                        'step': step_count + 1,
                        'state_code': '000001',  # 中间旋转电机停止，直线电机停止，前磁铁仍脱附，后磁铁吸附
                        'front_magnet_status': '脱附',
                        'rear_magnet_status': '吸附',
                        'front_position': new_front_pos,
                        'rear_position': current_rear.copy(),
                        'center_motor_angle': base_angle,  # 前磁铁放下（通过旋转电机动作）
                        'linear_motor_pos': linear_motor_delta,
                        'front_magnet_attach_pos': new_front_pos,  # 即将吸附的位置
                        'rear_magnet_attach_pos': current_rear.copy(),
                        'duration_ms': time_params['center_motor_action']  # 中心电机放下磁铁时间
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
                        'center_motor_angle': base_angle,  # 前磁铁吸附时保持基础角度
                        'linear_motor_pos': linear_motor_delta,
                        'front_magnet_attach_pos': new_front_pos,
                        'rear_magnet_attach_pos': current_rear.copy(),
                        'duration_ms': time_params['magnet_attach']  # 磁铁吸附时间
                    }
                    states_list.append(attach_state.copy())
                    
                    current_front = [current_front[0], new_front_z]
        
        step_count += 1
    
    return states_list, base_angle, lift_angle

# 生成状态列表
states_list, base_angle, lift_angle = generate_motor_positions_and_states_with_time()

# 写入详细状态表到文件
with open('robot_detailed_state_table_with_time.txt', 'w', encoding='utf-8') as f:
    f.write('"""机器人运动状态详细记录表（带时间参数）\n')
    f.write('时间：2026-01-29\n')
    f.write('任务：从初始位置(前足0,40；后足0,0)爬行至目标位置(前足0,1000)\n')
    f.write('基于优化轨迹（255步）\n')
    f.write('使用六位数字状态码：ω1ω2ω3vm1m2\n')
    f.write('包含时间参数：每个状态码持续的时间（毫秒）\n')
    f.write('"""\n\n')
    
    # 表头
    f.write(f"{'步数':<6} {'状态码':<10} {'前足磁铁':<10} {'后足磁铁':<10} {'前足位置':<15} {'后足位置':<15} {'中心电机角度':<12} {'直线电机':<10} {'前足吸附位置':<15} {'后足吸附位置':<15} {'持续时间(ms)':<12}\n")
    f.write("-" * 150 + "\n")
    
    # 数据行
    for state in states_list:
        f.write(f"{state['step']:<6} {state['state_code']:<10} {state['front_magnet_status']:<10} {state['rear_magnet_status']:<10} "
                f"{str(state['front_position']):<15} {str(state['rear_position']):<15} {state['center_motor_angle']:<12.1f} "
                f"{state['linear_motor_pos']:<10.1f} {str(state['front_magnet_attach_pos']):<15} {str(state['rear_magnet_attach_pos']):<15} {state['duration_ms']:<12}\n")

print("机器人运动状态详细记录表（带时间参数）已生成完成")
print(f"总共记录了 {len(states_list)} 个状态")
print("\n状态码说明：")
print("六位数字状态码：ω1ω2ω3vm1m2")
print("- ω1, ω2: 前、后节旋转电机状态 (0=停止, 1=正转, 2=反转)")
print("- ω3: 中间旋转电机状态 (0=停止, 1=正转, 2=反转)")
print("- v: 直线电机状态 (0=停止, 1=正转, 2=反转)")
print("- m1, m2: 电磁铁状态 (0=脱附, 1=吸附)")
print("\n磁铁抬起机制：")
print("- 通过中间旋转电机(ω3)动作来抬起/放下磁铁")
print("- 脱附时：旋转电机转动一定角度抬起对应磁铁")
print("- 吸附前：旋转电机转回原位放下磁铁")
print("\n时间参数说明：")
print("- 旋转电机启动/停止：50ms")
print("- 直线电机启动/停止：100ms")
print("- 中心旋转电机动作（抬起/放下磁铁）：200ms")
print("- 磁铁脱附过程：300ms")
print("- 磁铁吸附过程：300ms")
print("- 直线电机运行时间与行程相关：每毫米50ms")
print("- 旋转电机运行时间与角度相关：每度2ms")
print("\n表格格式说明：")
print(f"{'步数':<6} {'状态码':<10} {'前足磁铁':<10} {'后足磁铁':<10} {'前足位置':<15} {'后足位置':<15} {'中心电机角度':<12} {'直线电机':<10} {'前足吸附位置':<15} {'后足吸附位置':<15} {'持续时间(ms)':<12}")
print("-" * 150)

# 显示前几行作为示例
print("\n前10个状态示例：")
for i, state in enumerate(states_list[:10]):
    print(f"{state['step']:<6} {state['state_code']:<10} {state['front_magnet_status']:<10} {state['rear_magnet_status']:<10} "
          f"{str(state['front_position']):<15} {str(state['rear_position']):<15} {state['center_motor_angle']:<12.1f} "
          f"{state['linear_motor_pos']:<10.1f} {str(state['front_magnet_attach_pos']):<15} {str(state['rear_magnet_attach_pos']):<15} {state['duration_ms']:<12}")