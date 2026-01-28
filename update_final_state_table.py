"""
机器人运动状态详细记录表（按标准运动逻辑）
时间：2026-01-28
任务：从初始位置(前足0,40；后足0,0)爬行至目标位置(前足0,1000)
基于优化轨迹（255步）
使用六位数字状态码：ω1ω2ω3vm1m2
按照标准运动逻辑：双足吸附→一足脱附、另一足吸附→运动各电机使机器人能够实现运动，
中间电机内折使脱附一足抬起，并运动各旋转电机、直线电机使电磁铁落下时能够吸附到规划位置→
中间电机外翻使机器人脱附一足紧贴壁面→双足吸附→原吸附足脱附，原脱附足吸附，完成下一轮动作循环
"""

import numpy as np

def generate_motor_positions_and_states_standard_logic():
    """
    生成机器人运动过程中的详细状态表（使用六位数字状态码并按标准运动逻辑）
    状态码格式：ω1ω2ω3vm1m2
    - ω1, ω2, ω3: 旋转电机状态 (0=停止, 1=正转, 2=反转)
    - v: 直线电机状态 (0=停止, 1=正转, 2=反转) 
    - m1, m2: 电磁铁状态 (0=脱附, 1=吸附)
    
    标准运动逻辑：
    1. 双足吸附
    2. 一足脱附、另一足吸附
    3. 运动各电机使机器人能够实现运动，中间电机内折使脱附一足抬起，并运动各旋转电机、直线电机使电磁铁落下时能够吸附到规划位置
    4. 中间电机外翻使机器人脱附一足紧贴壁面
    5. 双足吸附
    6. 原吸附足脱附，原脱附足吸附，完成下一轮动作循环
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
    
    # 当前位置
    current_front = list(initial_front_pos)
    current_rear = list(initial_rear_pos)
    
    # 使用接近直线电机最大行程的步长
    step_size = 7.5  # mm，接近s_max但留有余量以确保安全
    
    # 中心电机角度控制参数
    base_angle = 0.0      # 基础角度（两臂共线）
    fold_angle = 45.0     # 折叠角度（内折用于抬起磁铁）
    unfold_angle = 90.0   # 展开角度（外翻用于放下磁铁）
    
    # 生成轨迹和状态 - 使用标准运动逻辑
    states_list = []
    
    # 初始状态：双足吸附
    current_state = {
        'step': 0,
        'state_code': '000011',  # 双足吸附
        'front_magnet_status': '吸附',
        'rear_magnet_status': '吸附',
        'front_position': current_front.copy(),
        'rear_position': current_rear.copy(),
        'center_motor_angle': base_angle,  # 中心电机基础角度
        'linear_motor_pos': 0.0,    # 初始直线电机位置
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
        
        # 标准运动逻辑：交替移动前后足
        if step_count % 2 == 0:  # 移动前足（当前前足为活动足，后足为固定足）
            # 1. 双足吸附（已在上一步完成）
            # 2. 前足脱附、后足吸附
            front_detach_state = {
                'step': step_count + 1,
                'state_code': '000001',  # 前足脱附，后足吸附
                'front_magnet_status': '脱附',
                'rear_magnet_status': '吸附',
                'front_position': current_front.copy(),
                'rear_position': current_rear.copy(),
                'center_motor_angle': base_angle,  # 保持基础角度
                'linear_motor_pos': abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': current_rear.copy()
            }
            states_list.append(front_detach_state.copy())
            
            # 3. 中间电机内折使前足抬起
            lift_front_state = {
                'step': step_count + 1,
                'state_code': '001001',  # 中间电机正转(内折)抬起前足，前足脱附，后足吸附
                'front_magnet_status': '脱附',
                'rear_magnet_status': '吸附',
                'front_position': current_front.copy(),
                'rear_position': current_rear.copy(),
                'center_motor_angle': fold_angle,  # 内折抬起前足
                'linear_motor_pos': abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': current_rear.copy()
            }
            states_list.append(lift_front_state.copy())
            
            # 计算前足的新位置
            new_front_z = min(current_front[1] + step_size, target_front_z)
            new_front_pos = (current_front[0], new_front_z)  # 保持theta不变
            
            # 4. 运动直线电机使机器人实现运动
            move_linear_state = {
                'step': step_count + 1,
                'state_code': '001201',  # 直线电机收缩，中间电机保持内折抬起前足
                'front_magnet_status': '脱附',
                'rear_magnet_status': '吸附',
                'front_position': new_front_pos,
                'rear_position': current_rear.copy(),
                'center_motor_angle': fold_angle,  # 保持前足抬起
                'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),  # 旧吸附位置
                'rear_magnet_attach_pos': current_rear.copy()
            }
            states_list.append(move_linear_state.copy())
            
            # 5. 中间电机外翻使前足紧贴壁面（准备吸附）
            lower_front_state = {
                'step': step_count + 1,
                'state_code': '002001',  # 中间电机反转(外翻)放下前足准备吸附，直线电机停止
                'front_magnet_status': '脱附',
                'rear_magnet_status': '吸附',
                'front_position': new_front_pos,
                'rear_position': current_rear.copy(),
                'center_motor_angle': base_angle,  # 外翻放下前足
                'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': new_front_pos,  # 即将吸附的位置
                'rear_magnet_attach_pos': current_rear.copy()
            }
            states_list.append(lower_front_state.copy())
            
            # 6. 前足吸附，形成双足吸附状态
            front_attach_state = {
                'step': step_count + 1,
                'state_code': '000011',  # 前足吸附，后足吸附（双足吸附）
                'front_magnet_status': '吸附',
                'rear_magnet_status': '吸附',
                'front_position': new_front_pos,
                'rear_position': current_rear.copy(),
                'center_motor_angle': base_angle,  # 回到基础角度
                'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': new_front_pos,
                'rear_magnet_attach_pos': current_rear.copy()
            }
            states_list.append(front_attach_state.copy())
            
            current_front = [current_front[0], new_front_z]
            
            # 7. 原吸附足(后足)脱附，原脱附足(前足)吸附（为下一轮做准备）
            rear_detach_state = {
                'step': step_count + 1,
                'state_code': '000010',  # 前足吸附，后足脱附
                'front_magnet_status': '吸附',
                'rear_magnet_status': '脱附',
                'front_position': new_front_pos,
                'rear_position': current_rear.copy(),
                'center_motor_angle': base_angle,  # 保持基础角度
                'linear_motor_pos': abs(new_front_pos[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': new_front_pos,
                'rear_magnet_attach_pos': current_rear.copy()
            }
            states_list.append(rear_detach_state.copy())
            
        else:  # 移动后足（当前后足为活动足，前足为固定足）
            # 1. 双足吸附（已在上一步完成）
            # 2. 后足脱附、前足吸附
            rear_detach_state = {
                'step': step_count + 1,
                'state_code': '000010',  # 后足脱附，前足吸附
                'front_magnet_status': '吸附',
                'rear_magnet_status': '脱附',
                'front_position': current_front.copy(),
                'rear_position': current_rear.copy(),
                'center_motor_angle': base_angle,  # 保持基础角度
                'linear_motor_pos': abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': current_rear.copy()
            }
            states_list.append(rear_detach_state.copy())
            
            # 3. 中间电机内折使后足抬起
            lift_rear_state = {
                'step': step_count + 1,
                'state_code': '001010',  # 中间电机正转(内折)抬起后足，前足吸附，后足脱附
                'front_magnet_status': '吸附',
                'rear_magnet_status': '脱附',
                'front_position': current_front.copy(),
                'rear_position': current_rear.copy(),
                'center_motor_angle': fold_angle,  # 内折抬起后足
                'linear_motor_pos': abs(current_front[1] - current_rear[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': current_rear.copy()
            }
            states_list.append(lift_rear_state.copy())
            
            # 计算后足的新位置
            new_rear_z = min(current_rear[1] + step_size, current_front[1] - (robot_params['d1'] + robot_params['d2']))
            new_rear_pos = (current_rear[0], new_rear_z)  # 保持theta不变
            
            # 4. 运动直线电机使机器人实现运动
            move_linear_state = {
                'step': step_count + 1,
                'state_code': '001110',  # 直线电机伸展，中间电机保持内折抬起后足
                'front_magnet_status': '吸附',
                'rear_magnet_status': '脱附',
                'front_position': current_front.copy(),
                'rear_position': new_rear_pos,
                'center_motor_angle': fold_angle,  # 保持后足抬起
                'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': current_rear.copy()
            }
            states_list.append(move_linear_state.copy())
            
            # 5. 中间电机外翻使后足紧贴壁面（准备吸附）
            lower_rear_state = {
                'step': step_count + 1,
                'state_code': '002010',  # 中间电机反转(外翻)放下后足准备吸附，直线电机停止
                'front_magnet_status': '吸附',
                'rear_magnet_status': '脱附',
                'front_position': current_front.copy(),
                'rear_position': new_rear_pos,
                'center_motor_angle': base_angle,  # 外翻放下后足
                'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': new_rear_pos  # 即将吸附的位置
            }
            states_list.append(lower_rear_state.copy())
            
            # 6. 后足吸附，形成双足吸附状态
            rear_attach_state = {
                'step': step_count + 1,
                'state_code': '000011',  # 前足吸附，后足吸附（双足吸附）
                'front_magnet_status': '吸附',
                'rear_magnet_status': '吸附',
                'front_position': current_front.copy(),
                'rear_position': new_rear_pos,
                'center_motor_angle': base_angle,  # 回到基础角度
                'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': new_rear_pos
            }
            states_list.append(rear_attach_state.copy())
            
            current_rear = [current_rear[0], new_rear_z]
            
            # 7. 原吸附足(前足)脱附，原脱附足(后足)吸附（为下一轮做准备）
            front_detach_state = {
                'step': step_count + 1,
                'state_code': '000001',  # 前足脱附，后足吸附
                'front_magnet_status': '脱附',
                'rear_magnet_status': '吸附',
                'front_position': current_front.copy(),
                'rear_position': new_rear_pos,
                'center_motor_angle': base_angle,  # 保持基础角度
                'linear_motor_pos': abs(current_front[1] - new_rear_pos[1]) - (robot_params['d1'] + robot_params['d2']),
                'front_magnet_attach_pos': current_front.copy(),
                'rear_magnet_attach_pos': new_rear_pos
            }
            states_list.append(front_detach_state.copy())
        
        step_count += 1
    
    return states_list, base_angle, fold_angle, unfold_angle

# 生成状态列表
states_list, base_angle, fold_angle, unfold_angle = generate_motor_positions_and_states_standard_logic()

# 写入详细状态表到文件（覆盖原文件）
with open('robot_detailed_state_table_final.txt', 'w', encoding='utf-8') as f:
    f.write('"""机器人运动状态详细记录表（按标准运动逻辑）\n')
    f.write('时间：2026-01-28\n')
    f.write('任务：从初始位置(前足0,40；后足0,0)爬行至目标位置(前足0,1000)\n')
    f.write('基于优化轨迹（255步）\n')
    f.write('使用六位数字状态码：ω1ω2ω3vm1m2\n')
    f.write('按照标准运动逻辑：双足吸附→一足脱附、另一足吸附→运动各电机使机器人能够实现运动，\n')
    f.write('中间电机内折使脱附一足抬起，并运动各旋转电机、直线电机使电磁铁落下时能够吸附到规划位置→\n')
    f.write('中间电机外翻使机器人脱附一足紧贴壁面→双足吸附→原吸附足脱附，原脱附足吸附，完成下一轮动作循环\n')
    f.write('"""\n\n')
    
    # 表头
    f.write(f"{'步数':<6} {'状态码':<10} {'前足磁铁':<10} {'后足磁铁':<10} {'前足位置':<15} {'后足位置':<15} {'中心电机角度':<12} {'直线电机':<10} {'前足吸附位置':<15} {'后足吸附位置':<15}\n")
    f.write("-" * 130 + "\n")
    
    # 数据行
    for state in states_list:
        f.write(f"{state['step']:<6} {state['state_code']:<10} {state['front_magnet_status']:<10} {state['rear_magnet_status']:<10} "
                f"{str(state['front_position']):<15} {str(state['rear_position']):<15} {state['center_motor_angle']:<12.1f} "
                f"{state['linear_motor_pos']:<10.1f} {str(state['front_magnet_attach_pos']):<15} {str(state['rear_magnet_attach_pos']):<15}\n")

print("机器人运动状态详细记录表（按标准运动逻辑）已生成完成")
print(f"总共记录了 {len(states_list)} 个状态")
print("\n状态码说明：")
print("六位数字状态码：ω1ω2ω3vm1m2")
print("- ω1, ω2: 前、后节旋转电机状态 (0=停止, 1=正转, 2=反转)")
print("- ω3: 中心旋转电机状态 (0=停止, 1=正转(内折), 2=反转(外翻))")
print("- v: 直线电机状态 (0=停止, 1=正转, 2=反转)")
print("- m1, m2: 电磁铁状态 (0=脱附, 1=吸附)")
print("\n标准运动逻辑：")
print("1. 双足吸附")
print("2. 一足脱附、另一足吸附")
print("3. 运动各电机使机器人能够实现运动，中间电机内折使脱附一足抬起，并运动各旋转电机、直线电机使电磁铁落下时能够吸附到规划位置")
print("4. 中间电机外翻使机器人脱附一足紧贴壁面")
print("5. 双足吸附")
print("6. 原吸附足脱附，原脱附足吸附，完成下一轮动作循环")
print("\n表格格式说明：")
print(f"{'步数':<6} {'状态码':<10} {'前足磁铁':<10} {'后足磁铁':<10} {'前足位置':<15} {'后足位置':<15} {'中心电机角度':<12} {'直线电机':<10} {'前足吸附位置':<15} {'后足吸附位置':<15}")
print("-" * 130)

# 显示前几行作为示例
print("\n前14个状态示例（展示一个完整循环）：")
for i, state in enumerate(states_list[:14]):
    print(f"{state['step']:<6} {state['state_code']:<10} {state['front_magnet_status']:<10} {state['rear_magnet_status']:<10} "
          f"{str(state['front_position']):<15} {str(state['rear_position']):<15} {state['center_motor_angle']:<12.1f} "
          f"{state['linear_motor_pos']:<10.1f} {str(state['front_magnet_attach_pos']):<15} {str(state['rear_magnet_attach_pos']):<15}")