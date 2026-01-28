"""
双节机器人运动规划系统
用于航空发动机/燃气轮机内腔表面的运动规划
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Tuple, Dict, Optional
import json
import re
import os


def set_axes_equal_3d(ax):
    """
    设置3D坐标轴的比例尺相等
    确保X、Y、Z轴的单位长度在图像中相同，保持真实的几何比例
    不会为了使图像呈正方体而扭曲比例
    
    参数:
        ax: matplotlib 3D轴对象
    """
    # 获取当前轴的范围
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    
    # 计算各轴的范围
    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])
    
    # 找到最大范围
    max_range = max(x_range, y_range, z_range)
    
    # 计算各轴的中点
    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)
    
    # 设置新的轴范围，使各轴范围相等（以最大范围为准）
    ax.set_xlim3d([x_middle - max_range/2, x_middle + max_range/2])
    ax.set_ylim3d([y_middle - max_range/2, y_middle + max_range/2])
    ax.set_zlim3d([z_middle - max_range/2, z_middle + max_range/2])
    
    # 尝试设置box_aspect（matplotlib 3.3+）
    try:
        ax.set_box_aspect([1, 1, 1])
    except AttributeError:
        pass  # 旧版本matplotlib不支持


class EngineSurface:
    """航空发动机流道外表面模型"""
    
    def __init__(self, z_min: float = 0, z_max: float = 1000, 
                 r_constant: float = 900.0,
                 blade_positions: List[float] = None):
        """
        初始化发动机表面
        z_min, z_max: 轴向范围（默认0-1000mm）
        r_constant: 内表面半径（恒定，单位：mm，默认900mm）
        blade_positions: 叶片位置列表（默认[200, 400, 600, 800]mm）
        """
        self.z_min = z_min
        self.z_max = z_max
        self.r_constant = r_constant
        self.blade_positions = blade_positions if blade_positions is not None else [200, 400, 600, 800]
        
        # 生成叶片障碍物
        self.blades = self._generate_blades()
    
    def _generate_blades(self) -> List[Dict]:
        """
        生成叶片障碍物
        叶片共四级，分布在200mm、400mm、600mm、800mm四个位置
        """
        blades = []
        # 在指定的z轴位置分布叶片
        for z_pos in self.blade_positions:
            # 每个位置有8个叶片，但留出通道
            num_blades = 8
            theta_step = 2 * np.pi / num_blades
            for i in range(num_blades):
                if i % 2 == 0:  # 偶数位置为叶片，奇数位置为通道
                    blades.append({
                        'z': z_pos,
                        'theta': i * theta_step,
                        'width': theta_step * 0.3,  # 叶片角度宽度
                        'height': 15,  # 叶片径向高度
                    })
        return blades
    
    def get_radius(self, z: float) -> float:
        """
        获取z位置的内表面半径（机器人爬行的表面）
        注意：这是旋转体的内表面半径，不是外表面
        返回恒定半径900mm
        """
        return self.r_constant
    
    def point_to_surface_coords(self, x: float, y: float, z: float) -> Tuple[float, float]:
        """
        将3D点转换为表面坐标 (theta, z)
        theta: 角度 (0-2π)
        z: 轴向位置
        """
        theta = np.arctan2(y, x)
        if theta < 0:
            theta += 2 * np.pi
        return theta, z
    
    def is_valid_position(self, theta: float, z: float, tolerance: float = 0.1) -> bool:
        """检查位置是否有效（不在叶片上）"""
        # 边界检查
        if z < self.z_min or z > self.z_max:
            return False
        
        # 检查是否在叶片上
        for blade in self.blades:
            if abs(z - blade['z']) < tolerance:
                theta_diff = abs(theta - blade['theta'])
                # 处理角度周期性
                theta_diff = min(theta_diff, 2 * np.pi - theta_diff)
                if theta_diff < blade['width'] / 2:
                    return False
        
        return True
    
    def visualize(self, ax, show_bottom_only=False):
        """
        可视化表面和障碍物（水平放置，口朝水平方向）
        
        参数:
            ax: matplotlib 3D轴
            show_bottom_only: 如果为True，只显示底部半圆（Y<0的部分），便于查看机器人轨迹
        """
        z_vals = np.linspace(self.z_min, self.z_max, 80)
        
        if show_bottom_only:
            # 只显示底部半圆（theta从180°到360°，即pi到2*pi）
            # 这对应于Y值为负的区域，机器人在这里运动
            theta_vals = np.linspace(np.pi, 2 * np.pi, 40)
        else:
            theta_vals = np.linspace(0, 2 * np.pi, 80)
        
        Z, Theta = np.meshgrid(z_vals, theta_vals)
        R = self.get_radius(Z)
        # 原始坐标：X, Y为径向，Z为轴向
        X_orig = R * np.cos(Theta)
        Y_orig = R * np.sin(Theta)
        Z_orig = Z
        
        # 转换为水平放置：Z轴（轴向）-> X轴（水平），Y轴（径向）-> Z轴（垂直），X轴（径向）-> Y轴（水平）
        X = Z_orig  # 轴向变成X轴（水平）
        Y = X_orig   # 径向X变成Y轴（水平）
        Z = Y_orig   # 径向Y变成Z轴（垂直）
        
        # 绘制内表面（机器人爬行的表面）
        ax.plot_surface(X, Y, Z, alpha=0.2, color='lightblue', 
                       edgecolor='gray', linewidth=0.1, label='Inner Surface')
        
        # 绘制叶片（在内腔中，从内表面向内延伸）
        blade_label_added = False  # 标记是否已添加叶片图例
        for blade in self.blades:
            z = blade['z']
            r_surface = self.get_radius(z)  # 内表面半径（机器人吸附的表面）
            theta_center = blade['theta']
            width = blade['width']
            height = blade['height']
            
            # 叶片从内表面向内（朝向中心）延伸
            r_inner = max(0.1, r_surface - height)  # 叶片内端半径，确保不为负
            
            theta_blade = np.linspace(theta_center - width/2, theta_center + width/2, 10)
            for i, theta in enumerate(theta_blade):
                # 内表面上的点（机器人吸附的面）
                x1_orig = r_surface * np.cos(theta)
                y1_orig = r_surface * np.sin(theta)
                # 叶片内端的点（朝向中心）
                x2_orig = r_inner * np.cos(theta)
                y2_orig = r_inner * np.sin(theta)
                # 坐标转换（水平放置）
                x1, y1, z1 = z, x1_orig, y1_orig
                x2, y2, z2 = z, x2_orig, y2_orig
                # 只给第一个叶片的第一条线添加label
                label = 'Blade' if not blade_label_added and i == 0 else None
                if label:
                    blade_label_added = True
                ax.plot([x1, x2], [y1, y2], [z1, z2], 'r-', linewidth=2, label=label)


class Robot:
    """双节机器人模型
    
    根据PDF文档（04_21376223_刘丹阳）中的运动学模型：
    - 机器人简化为两个可相对位移、相对旋转的三角形
    - h: 中心电机转轴到上表面（水平面）的高度
    - d1: 前足吸盘中心到中间电机转轴的水平距离（当a=0时）
    - d2: 后足吸盘中心到中间电机转轴的水平距离（当a=0时）
    - a: 直线电机行程，范围[0, s_max]
    - φ: 机器人前进方向角（机器人所在平面与环境轴线的角度）
    - γ: 翻折角（两节间相对转动角度）
    """
    
    def __init__(self, L1: float = 40, W1: float = 30, 
                 L2: float = 40, W2: float = 30, 
                 h: float = 30, s_max: float = 8,
                 d1: float = 20, d2: float = 20,
                 magnet_radius: float = 2.5, theta3_min: float = -90, theta3_max: float = 90):
        """
        初始化机器人参数（单位：mm，角度：度）
        L1, W1: 前节长宽 (40mm, 30mm)
        L2, W2: 后节长宽 (40mm, 30mm)
        h: 中间电机转轴到上表面的高度 (30mm)
        s_max: 直线电机（丝杠）最大行程 (8mm)
        d1: 前足吸盘中心到中间电机转轴的水平距离（当a=0时），默认L1/2=20mm
        d2: 后足吸盘中心到中间电机转轴的水平距离（当a=0时），默认L2/2=20mm
        magnet_radius: 电磁铁半径 (mm)，默认2.5mm（直径5mm）
        theta3_min, theta3_max: 两节相对旋转角度范围 (-90° 到 90°，0°为共面)
        """
        self.L1 = L1
        self.W1 = W1
        self.L2 = L2
        self.W2 = W2
        self.h = h
        self.s_max = s_max
        self.d1 = d1  # 前足到中间电机的水平距离
        self.d2 = d2  # 后足到中间电机的水平距离
        self.magnet_radius = magnet_radius
        self.theta3_min = np.radians(theta3_min)  # 转换为弧度
        self.theta3_max = np.radians(theta3_max)  # 转换为弧度
        
        # 机器人状态（电机位置）
        self.theta1 = 0.0  # 前节电磁铁角度（度，范围-360到360）
        self.theta2 = 0.0  # 后节电磁铁角度（度，范围-360到360）
        self.theta3 = 0.0  # 两节间相对角度（弧度，范围-90°到90°）
        self.s = 0.0  # 丝杠位移（mm，范围0到s_max）
        self.magnet1_on = True  # 前节电磁铁状态
        self.magnet2_on = True  # 后节电磁铁状态
        
        # 当前位置（表面坐标）
        self.pos_theta = 0.0
        self.pos_z = 50.0
        
        # 前后足的3D坐标位置（用于运动规划）
        # 坐标系：X水平，Y水平（指向圆柱内壁，负值表示在底部），Z轴向
        self.front_foot_pos = np.array([0.0, -900.0, 40.0])  # 前足位置
        self.rear_foot_pos = np.array([0.0, -900.0, 0.0])    # 后足位置
        
        # 机器人朝向：后足指向前足的方向
        # 初始状态：Z轴减小方向（因为前足Z=40，后足Z=0，前足在后足前方，
        # 但"后足指向前足"是从后足到前足的方向，即Z增大方向）
        # 根据用户描述："后足指向前足的方向为Z减小的方向"
        # 这意味着前足在Z轴负方向（即前足Z < 后足Z），但初始条件是前足Z=40 > 后足Z=0
        # 需要澄清：初始时前足在Z=40，后足在Z=0，机器人朝向Z减小方向意味着
        # 机器人是"倒着"走的，即从Z=40走向Z=0（减小），然后继续到Z=1000
        # 或者理解为：机器人整体会从Z=0走向Z=1000，但行进方向的"前"是Z减小方向
        # 根据目标（前足终点Z=1000），机器人需要向Z增大方向移动
        self.heading_direction = np.array([0.0, 0.0, 1.0])  # 朝向Z增大方向（移动方向）
    
    def calc_other_foot_position(self, fixed_foot_X: float, fixed_foot_theta: float,
                                  phi: float, a: float, R: float,
                                  find_front: bool = True) -> Tuple[float, float]:
        """
        根据PDF文档中的公式计算另一足的落点位置。
        机器人从壁面底部出发，Y 恒为负；落点须与固定足同半圆（同 Y 符号）。
        
        PDF: 椭圆 y²/R² + x²/(R/sin(φ))² = 1；小圆 (2.4)(2.5)。
        增量 ΔX = x0·cos(φ)，Δθ = arctan(x0·sin(φ)/y0)。
        """
        # ---------- φ≈0° 纯轴向运动：Δθ=0，ΔX=±(d1+a+d2) ----------
        if abs(np.sin(phi)) < 0.01:  # 修正：φ≈0° 为纯轴向运动
            d = self.d1 + a + self.d2
            if find_front:
                return (d, 0.0)   # 后足→前足：前足在轴向后方，ΔX>0
            else:
                return (-d, 0.0)  # 前足→后足：后足在轴向后方，ΔX<0
        
        # ---------- 一般情况：小圆与椭圆求交点 ----------
        sin_phi = np.sin(phi) if abs(np.sin(phi)) >= 0.01 else (0.01 if phi >= 0 else -0.01)
        a_ellipse = abs(R / sin_phi)
        b_ellipse = R
        
        if find_front:
            cx = -(self.d2 + a)
            cy = -R + self.h
            r_circle = np.sqrt(self.d1**2 + self.h**2)  # 修正：使用勾股定理计算实际距离
        else:
            cx = self.d1
            cy = -R + self.h
            r_circle = np.sqrt((a + self.d2)**2 + self.h**2)  # 修正：使用勾股定理计算实际距离
        
        # 使用解析方法求解椭圆与圆的交点
        x0, y0 = self._solve_ellipse_circle_intersection(cx, cy, r_circle, a_ellipse, b_ellipse, 
                                                         fixed_foot_theta, find_front)
        
        delta_X = x0 * np.cos(phi)
        delta_theta = np.arctan2(x0 * np.sin(phi), y0) if abs(y0) > 0.001 else 0.0
        return (delta_X, delta_theta)
    
    def _solve_ellipse_circle_intersection(self, cx, cy, r_circle, a_ellipse, b_ellipse, 
                                           fixed_foot_theta, find_front):
        """
        解析求解椭圆与圆的交点
        椭圆: x^2/a_ellipse^2 + y^2/b_ellipse^2 = 1
        圆: (x-cx)^2 + (y-cy)^2 = r_circle^2
        """
        # 使用numpy求解四次方程的方法
        # 从椭圆方程: x^2 = a_ellipse^2 * (1 - y^2/b_ellipse^2)
        # 从圆方程: (x-cx)^2 = r_circle^2 - (y-cy)^2
        # 所以 x = cx ± sqrt(r_circle^2 - (y-cy)^2)
        
        # 将x代入椭圆方程: [cx ± sqrt(r_circle^2-(y-cy)^2)]^2 = a_ellipse^2 * (1 - y^2/b_ellipse^2)
        # 展开: cx^2 ± 2*cx*sqrt(r_circle^2-(y-cy)^2) + (r_circle^2-(y-cy)^2) = a_ellipse^2 - a_ellipse^2*y^2/b_ellipse^2
        # 移项: ± 2*cx*sqrt(r_circle^2-(y-cy)^2) = a_ellipse^2 - a_ellipse^2*y^2/b_ellipse^2 - cx^2 - (r_circle^2-(y-cy)^2)
        
        # RHS = a_ellipse^2 - a_ellipse^2*y^2/b_ellipse^2 - cx^2 - r_circle^2 + (y-cy)^2
        # RHS = a_ellipse^2 - cx^2 - r_circle^2 + cy^2 - a_ellipse^2*y^2/b_ellipse^2 + y^2 - 2*cy*y
        # RHS = (a_ellipse^2 - cx^2 - r_circle^2 + cy^2) + y^2*(1 - a_ellipse^2/b_ellipse^2) - 2*cy*y
        # RHS = C1 + C2*y^2 + C3*y
        
        C1 = a_ellipse**2 - cx**2 - r_circle**2 + cy**2
        C2 = 1 - a_ellipse**2 / b_ellipse**2
        C3 = -2*cy
        
        # LHS = ± 2*cx*sqrt(r_circle^2-(y-cy)^2)
        # 两边平方: 4*cx^2*(r_circle^2-(y-cy)^2) = (C1 + C2*y^2 + C3*y)^2
        # 展开右边: C1^2 + C2^2*y^4 + C3^2*y^2 + 2*C1*C2*y^2 + 2*C1*C3*y + 2*C2*C3*y^3
        # 展开左边: 4*cx^2*(r_circle^2 - y^2 + 2*cy*y - cy^2)
        #         = 4*cx^2*(r_circle^2 - cy^2) + 8*cx^2*cy*y - 4*cx^2*y^2
        
        # 方程: 4*cx^2*(r_circle^2 - cy^2) + 8*cx^2*cy*y - 4*cx^2*y^2 
        #     = C1^2 + C2^2*y^4 + C3^2*y^2 + 2*C1*C2*y^2 + 2*C1*C3*y + 2*C2*C3*y^3
        # 
        # 重新整理为标准四次方程: coeff_4*y^4 + coeff_3*y^3 + coeff_2*y^2 + coeff_1*y + coeff_0 = 0
        
        coeff_4 = C2**2
        coeff_3 = 2*C2*C3
        coeff_2 = C3**2 + 2*C1*C2 - 4*cx**2  # C3^2*y^2 + 2*C1*C2*y^2 - 4*cx^2*y^2
        coeff_1 = 2*C1*C3 - 8*cx**2*cy  # 2*C1*C3*y - 8*cx^2*cy*y
        coeff_0 = C1**2 - 4*cx**2*(r_circle**2 - cy**2)  # C1^2 - 4*cx^2*(r^2-cy^2)
        
        # 使用numpy求解四次方程
        coeffs = [coeff_4, coeff_3, coeff_2, coeff_1, coeff_0]
        roots = np.roots(coeffs)
        
        # 筛选实根
        valid_solutions = []
        for root in roots:
            if np.isreal(root):
                y_val = np.real(root)
                
                # 检查是否满足圆方程的约束条件 (r_circle^2 - (y_val-cy)^2 >= 0)
                under_sqrt = r_circle**2 - (y_val - cy)**2
                if under_sqrt < 0:
                    continue  # 不是有效解
                    
                sqrt_val = np.sqrt(under_sqrt)
                
                # 有两个x值的候选：cx ± sqrt_val
                for sign in [1, -1]:
                    x_candidate = cx + sign * sqrt_val
                    
                    # 验证是否同时满足椭圆方程
                    ellipse_check = (x_candidate**2 / a_ellipse**2) + (y_val**2 / b_ellipse**2)
                    ellipse_error = abs(ellipse_check - 1)
                    
                    if ellipse_error < 1e-6:  # 验证误差阈值
                        # 验证圆方程
                        circle_check = (x_candidate - cx)**2 + (y_val - cy)**2
                        circle_error = abs(circle_check - r_circle**2)
                        
                        # 检查是否与固定足在同半圆（Y符号相同）
                        fixed_sign = np.sign(np.sin(fixed_foot_theta))
                        if fixed_sign == 0:
                            fixed_sign = -1.0
                        
                        # 计算新点的角度
                        other_theta = fixed_foot_theta  # 初始角度
                        if abs(y_val) > 0.001:  # 避免除零
                            # 正确计算角度变化：Δθ = arctan(x0·sin(φ)/y0)
                            # 但我们这里先只关心y值符号
                            pass
                        
                        # 检查是否在底部半圆（y < 0）
                        if y_val < 0:  # 确保在底部半圆（Y为负）
                            total_error = ellipse_error + circle_error
                            valid_solutions.append((x_candidate, y_val, total_error))
        
        # 如果没有找到有效解，使用近似解
        if not valid_solutions:
            # 使用原始的迭代方法作为后备
            return self._solve_ellipse_circle_intersection_iterative_backup(cx, cy, r_circle, a_ellipse, b_ellipse, 
                                                                           fixed_foot_theta, find_front)
        
        # 选择误差最小的解
        best_solution = min(valid_solutions, key=lambda x: x[2])
        return best_solution[0], best_solution[1]

    def _solve_ellipse_circle_intersection_iterative_backup(self, cx, cy, r_circle, a_ellipse, b_ellipse, 
                                                           fixed_foot_theta, find_front):
        """
        备用的迭代方法，当解析方法失败时使用
        """
        fixed_sign = np.sign(np.sin(fixed_foot_theta))
        if fixed_sign == 0:
            fixed_sign = -1.0  # 底部 Y<0，取负
        
        best_x, best_y = 0.0, -a_ellipse
        min_error = float('inf')
        
        for angle in np.linspace(0, 2 * np.pi, 720):  # 增加采样点提高精度
            x = cx + r_circle * np.cos(angle)
            y = cy + r_circle * np.sin(angle)
            if not (y < 0 and y > -a_ellipse - 10):
                continue
            err = abs((x**2 / a_ellipse**2) + (y**2 / b_ellipse**2) - 1)
            if err < min_error:
                # 检查角度约束
                if abs(y) < 0.001:
                    dt = 0.0
                else:
                    dt = np.arctan2(x * np.sin(0.785), y)  # 使用一个示例phi值
                other_theta = fixed_foot_theta + dt
                other_sign = np.sign(np.sin(other_theta))
                if other_sign == 0:
                    other_sign = -1.0
                if fixed_sign != other_sign:
                    continue
                min_error = err
                best_x, best_y = x, y
        
        return best_x, best_y
    
    def calc_rear_foot_from_front(self, front_X: float, front_theta: float,
                                   phi: float, a: float, R: float) -> Tuple[float, float]:
        """
        已知前足位置，计算后足落点位置
        
        参数:
            front_X: 前足X坐标（轴向）
            front_theta: 前足θ坐标（周向，弧度）
            phi: 机器人前进方向角（弧度）
            a: 直线电机行程
            R: 圆柱半径
        
        返回:
            (rear_X, rear_theta): 后足的绝对位置
        """
        delta_X, delta_theta = self.calc_other_foot_position(
            front_X, front_theta, phi, a, R, find_front=False
        )
        rear_X = front_X + delta_X
        rear_theta = front_theta + delta_theta
        return rear_X, rear_theta
    
    def calc_front_foot_from_rear(self, rear_X: float, rear_theta: float,
                                   phi: float, a: float, R: float) -> Tuple[float, float]:
        """
        已知后足位置，计算前足落点位置
        
        参数:
            rear_X: 后足X坐标（轴向）
            rear_theta: 后足θ坐标（周向，弧度）
            phi: 机器人前进方向角（弧度）
            a: 直线电机行程
            R: 圆柱半径
        
        返回:
            (front_X, front_theta): 前足的绝对位置
        """
        delta_X, delta_theta = self.calc_other_foot_position(
            rear_X, rear_theta, phi, a, R, find_front=True
        )
        front_X = rear_X + delta_X
        front_theta = rear_theta + delta_theta
        return front_X, front_theta
    
    def get_state_code(self) -> str:
        """
        获取六位状态码
        格式: ω1 ω2 ω3 v m1 m2
        ω1, ω2, ω3, v: 0(停止), 1(正转), 2(反转)
        m1, m2: 0(脱附), 1(吸附)
        """
        # 这里需要根据实际运动状态返回，暂时返回当前静态状态
        return "0000" + ("1" if self.magnet1_on else "0") + ("1" if self.magnet2_on else "0")
    
    def update_state(self, state_code: str, duration: float):
        """
        根据状态码更新机器人状态
        state_code: 六位状态码
        duration: 持续时间
        """
        # 解析状态码
        w1 = int(state_code[0])
        w2 = int(state_code[1])
        w3 = int(state_code[2])
        v = int(state_code[3])
        m1 = int(state_code[4])
        m2 = int(state_code[5])
        
        # 更新电磁铁状态
        self.magnet1_on = (m1 == 1)
        self.magnet2_on = (m2 == 1)
        
        # 更新角度和位移（简化模型，假设恒定角速度/线速度）
        omega = 1.0  # 角速度（弧度/秒）
        v_linear = 1.0  # 线速度（单位/秒）
        
        if w1 == 1:
            self.theta1 += omega * duration
        elif w1 == 2:
            self.theta1 -= omega * duration
        
        if w2 == 1:
            self.theta2 += omega * duration
        elif w2 == 2:
            self.theta2 -= omega * duration
        
        if w3 == 1:
            self.theta3 += omega * duration
        elif w3 == 2:
            self.theta3 -= omega * duration
        
        if v == 1:
            self.s += v_linear * duration
            self.s = min(self.s, self.s_max)
        elif v == 2:
            self.s -= v_linear * duration
            self.s = max(self.s, -self.s_max)
        
        # 规范化角度
        self.theta1 = self.theta1 % (2 * np.pi)
        self.theta2 = self.theta2 % (2 * np.pi)
        # theta3限制在-90°到90°之间
        self.theta3 = max(self.theta3_min, min(self.theta3_max, self.theta3))


class MotionPlanner:
    """运动规划器"""
    
    def __init__(self, robot: Robot, surface: EngineSurface):
        self.robot = robot
        self.surface = surface
        self.plan = []  # 规划结果 [(state_code, duration), ...]
        self.path_points = []  # 实际路径点序列 [(theta, z), ...]
        # 跟踪机器人当前状态（用于运动规划）
        self.current_theta1 = 0.0  # 当前ω1角度（度，范围-360°到360°）
        self.current_theta2 = 0.0  # 当前ω2角度（度，范围-360°到360°）
        self.current_theta3 = 0.0  # 当前两节相对角度（弧度，范围-90°到90°）
        self.current_s = 0.0  # 当前丝杠位移（mm，范围0到s_max）
        
        # 3D坐标轨迹记录
        self.front_foot_trajectory = []  # 前足轨迹 [(x, y, z), ...]
        self.rear_foot_trajectory = []   # 后足轨迹 [(x, y, z), ...]
    
    def xyz_to_surface_coords(self, x: float, y: float, z: float) -> Tuple[float, float]:
        """
        将3D笛卡尔坐标转换为圆柱面坐标 (theta, z_axial)
        x, y: 径向坐标
        z: 轴向坐标
        返回: (theta, z_axial) 其中theta是角度(弧度)，z_axial是轴向位置
        """
        theta = np.arctan2(y, x)
        if theta < 0:
            theta += 2 * np.pi
        return theta, z
    
    def surface_coords_to_xyz(self, theta: float, z: float, radius: float = None) -> Tuple[float, float, float]:
        """
        将圆柱面坐标转换为3D笛卡尔坐标
        theta: 角度(弧度)
        z: 轴向位置
        radius: 圆柱半径，默认使用surface的恒定半径
        返回: (x, y, z)
        """
        if radius is None:
            radius = self.surface.r_constant
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        return x, y, z
    
    def plan_path_3d(self, front_start: Tuple[float, float, float],
                     rear_start: Tuple[float, float, float],
                     front_goal: Tuple[float, float, float],
                     step_size: float = 20.0) -> List[Tuple[str, float]]:
        """
        基于3D坐标进行路径规划
        
        参数:
        front_start: 前足起始点 (x, y, z)
        rear_start: 后足起始点 (x, y, z)  
        front_goal: 前足终点 (x, y, z)
        step_size: 步长 (mm)
        
        返回: 运动序列 [(state_code, duration), ...]
        """
        # 转换为表面坐标进行规划
        start_theta, start_z = self.xyz_to_surface_coords(*front_start)
        goal_theta, goal_z = self.xyz_to_surface_coords(*front_goal)
        
        # 记录初始3D坐标
        self.front_foot_trajectory = [front_start]
        self.rear_foot_trajectory = [rear_start]
        
        # 使用原有的规划方法
        return self.plan_path((start_theta, start_z), (goal_theta, goal_z), step_size)
    
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float],
                  step_size: float = 1.0) -> List[Tuple[str, float]]:
        """
        规划从起点到终点的路径（使用交替步进策略）
        start: (theta, z) 起点
        goal: (theta, z) 终点
        step_size: 步长
        """
        self.plan = []
        
        current_theta, current_z = start
        goal_theta, goal_z = goal
        
        # 保存可视化用起点/终点（仅当非 plan_path_3d 调用时）
        if not hasattr(self, '_viz_front_start_z'):
            self._viz_start = start
            self._viz_goal = goal
        
        # 处理角度周期性
        theta_diff = goal_theta - current_theta
        if theta_diff > np.pi:
            theta_diff -= 2 * np.pi
        elif theta_diff < -np.pi:
            theta_diff += 2 * np.pi
        
        # 使用交替步进策略进行规划
        # 首先分解运动：先处理周向运动，再处理轴向运动
        steps = self._generate_step_sequence(current_theta, current_z, 
                                            goal_theta, goal_z, step_size)
        
        # 保存路径点序列（包括起点）
        self.path_points = [(start[0], start[1])]
        
        # 初始化机器人状态跟踪
        self.current_theta1 = 0.0  # 初始时ω1角度为0°
        self.current_theta2 = 0.0  # 初始时ω2角度为0°
        self.current_theta3 = 0.0  # 初始时两节共面（θ3=0°）
        self.current_s = 0.0  # 初始时丝杠位置为0mm
        
        # 当前吸附状态：True表示前节吸附，False表示后节吸附
        front_attached = True
        
        for step_theta, step_z in steps:
            # 生成交替步进的运动序列
            step_plan = self._generate_step_motion(
                current_theta, current_z, step_theta, step_z, front_attached
            )
            self.plan.extend(step_plan)
            # 保存路径点
            self.path_points.append((step_theta, step_z))
            current_theta, current_z = step_theta, step_z
            front_attached = not front_attached  # 交替吸附
        
        # 确保终点被包含（如果不在序列中，添加到末尾）
        if len(self.path_points) == 0 or self.path_points[-1] != (goal_theta, goal_z):
            self.path_points.append((goal_theta, goal_z))
        
        # 在规划结束时，确保机器人处于安全状态（两磁铁都吸附）
        # 如果最后一个状态不是两磁铁都吸附，添加一个过渡状态
        if len(self.plan) > 0:
            last_state = self.plan[-1][0]
            if len(last_state) >= 6:
                last_m1 = last_state[4]
                last_m2 = last_state[5]
                # 如果最后一个状态不是两磁铁都吸附，添加一个安全状态
                if not (last_m1 == '1' and last_m2 == '1'):
                    # 添加一个短暂的两磁铁都吸附状态（0.1秒）
                    self.plan.append(("000011", 0.1))
        
        return self.plan
    
    def _find_passage_theta(self, z_pos: float, current_theta: float) -> float:
        """
        找到在给定z位置最近的通道角度
        叶片在偶数位置（0, 2, 4, 6），通道在奇数位置（1, 3, 5, 7）
        """
        num_positions = 8
        theta_step = 2 * np.pi / num_positions
        
        # 通道位置（奇数）
        passage_thetas = [(2*i + 1) * theta_step for i in range(4)]  # 1, 3, 5, 7位置
        
        # 找到最近的通道
        min_dist = float('inf')
        best_theta = passage_thetas[0]
        for pt in passage_thetas:
            # 计算角度距离（考虑周期性）
            diff = abs(current_theta - pt)
            diff = min(diff, 2 * np.pi - diff)
            if diff < min_dist:
                min_dist = diff
                best_theta = pt
        
        return best_theta
    
    def _get_blade_positions_between(self, z_start: float, z_end: float) -> List[float]:
        """获取起点和终点之间的所有叶片z位置"""
        z_min, z_max = min(z_start, z_end), max(z_start, z_end)
        blade_z_positions = []
        for blade in self.surface.blades:
            if z_min < blade['z'] < z_max:
                if blade['z'] not in blade_z_positions:
                    blade_z_positions.append(blade['z'])
        return sorted(blade_z_positions)
    
    def _is_blocked_by_blade(self, theta: float, z_pos: float, tolerance: float = 5.0) -> bool:
        """检查在给定theta角度、接近z_pos的位置是否被叶片阻挡"""
        for blade in self.surface.blades:
            if abs(z_pos - blade['z']) < tolerance:
                theta_diff = abs(theta - blade['theta'])
                theta_diff = min(theta_diff, 2 * np.pi - theta_diff)
                if theta_diff < blade['width'] / 2 + 0.1:  # 加一些余量
                    return True
        return False
    
    def _generate_step_sequence(self, start_theta: float, start_z: float,
                               goal_theta: float, goal_z: float,
                               step_size: float) -> List[Tuple[float, float]]:
        """
        生成路径点序列（不包括起点，但包括终点）
        
        避障策略：
        1. 叶片高度远大于机器人高度，机器人不能跨过叶片
        2. 机器人必须在周向上绕到通道位置才能通过叶片所在的z位置
        3. 通道位置在theta=45°, 135°, 225°, 315°（即(2i+1)*45°）
        4. 叶片位置在theta=0°, 90°, 180°, 270°（即2i*45°）
        5. 路径规划：移动到通道 -> 沿通道前进穿过所有叶片 -> 到达目标Z后回到目标theta
        """
        steps = []
        
        current_theta = start_theta
        current_z = start_z
        
        # 获取路径上需要穿过的所有叶片位置
        blade_z_list = self._get_blade_positions_between(start_z, goal_z)
        
        # 确定移动方向
        moving_forward = (goal_z > start_z)
        
        # 如果有叶片需要穿过，先找到最近的通道位置
        if len(blade_z_list) > 0:
            # 找到距离起始位置最近的通道
            passage_theta = self._find_passage_theta(blade_z_list[0], current_theta)
            
            # 如果当前位置不在通道上，先移动到通道
            if self._is_blocked_by_blade(current_theta, blade_z_list[0]):
                # 生成周向移动路径到通道
                theta_diff = passage_theta - current_theta
                if theta_diff > np.pi:
                    theta_diff -= 2 * np.pi
                elif theta_diff < -np.pi:
                    theta_diff += 2 * np.pi
                
                num_theta_steps = max(1, int(abs(theta_diff) / 0.2))
                for i in range(1, num_theta_steps + 1):
                    t = i / num_theta_steps
                    next_theta = (current_theta + theta_diff * t) % (2 * np.pi)
                    steps.append((next_theta, current_z))
                current_theta = passage_theta
            
            # 现在在通道位置，沿Z方向一直前进，穿过所有叶片
            # 找到最后一个叶片的位置
            last_blade_z = blade_z_list[-1]
            target_z_after_blades = last_blade_z + 20 if moving_forward else last_blade_z - 20
            
            # 沿通道前进到最后一个叶片之后
            z_distance = abs(target_z_after_blades - current_z)
            num_steps = max(1, int(z_distance / step_size))
            for i in range(1, num_steps + 1):
                t = i / num_steps
                next_z = current_z + (target_z_after_blades - current_z) * t
                steps.append((current_theta, next_z))
            current_z = target_z_after_blades
        
        # 现在已经穿过所有叶片，可以安全地移动到目标theta
        # 检查是否需要周向移动（纯直线运动时，theta应该严格不变）
        theta_diff = goal_theta - current_theta
        if theta_diff > np.pi:
            theta_diff -= 2 * np.pi
        elif theta_diff < -np.pi:
            theta_diff += 2 * np.pi
        
        # 对于纯直线运动，如果theta差异很小（<0.001弧度≈0.057°），直接使用goal_theta，不生成中间步骤
        if abs(theta_diff) > 0.001:  # 约0.057°
            num_theta_steps = max(1, int(abs(theta_diff) / 0.2))
            for i in range(1, num_theta_steps + 1):
                t = i / num_theta_steps
                next_theta = (current_theta + theta_diff * t) % (2 * np.pi)
                steps.append((next_theta, current_z))
            current_theta = goal_theta
        else:
            # theta差异很小，直接使用goal_theta，确保纯直线运动
            current_theta = goal_theta
        
        # 最后轴向移动到目标Z
        z_distance = abs(goal_z - current_z)
        if z_distance > 0.1:
            num_steps_final = max(1, int(z_distance / step_size))
            for i in range(1, num_steps_final + 1):
                t = i / num_steps_final
                next_z = current_z + (goal_z - current_z) * t
                steps.append((current_theta, next_z))
        
        # 确保终点被包含
        if len(steps) == 0 or steps[-1] != (goal_theta, goal_z):
            steps.append((goal_theta, goal_z))
        
        return steps
    
    def _generate_step_motion(self, from_theta: float, from_z: float, 
                               to_theta: float, to_z: float, 
                               front_attached: bool) -> List[Tuple[str, float]]:
        """
        生成从一点到另一点的交替步进运动序列
        
        根据论文（04_21376223_刘丹阳）中描述的运动逻辑：
        
        蠕虫式爬行（轴向移动）完整周期：
        阶段1: 后足脱附 + 内折抬起
        阶段2: 丝杠伸出（后足前移）
        阶段3: 放下后足 + 双足吸附
        阶段4: 前足脱附 + 内折抬起
        阶段5: 丝杠收缩（前足前移）
        阶段6: 放下前足 + 双足吸附
        
        一个完整周期的轴向位移 ≈ 丝杠行程（最大8mm）
        
        返回: [(state_code, duration), ...]
        """
        motions = []
        dtheta = to_theta - from_theta
        dz = to_z - from_z
        
        # 处理角度差
        if dtheta > np.pi:
            dtheta -= 2 * np.pi
        elif dtheta < -np.pi:
            dtheta += 2 * np.pi
        
        # 运动参数
        omega_w3 = 0.5  # ω3角速度 (rad/s)，用于翻折动作
        omega_w1w2 = 0.5  # ω1/ω2角速度 (rad/s)，用于周向旋转
        v_linear = 0.5  # 线速度 (mm/s)
        
        # 翻折角度参数（单位：弧度）
        theta3_fold_angle = np.radians(35)  # 翻折角度：35°
        
        # 短暂停顿时间（用于状态切换）
        pause_time = 0.5  # 秒
        
        # 单步丝杠行程（充分利用8mm行程，留5%裕量）
        step_stroke = self.robot.s_max * 0.95  # ≈7.6mm
        
        # =====================================================
        # 轴向运动：蠕虫式爬行（需要完整周期）
        # 需要循环直到完成目标位移
        # =====================================================
        remaining_dz = abs(dz)
        move_forward = (dz > 0)  # 是否向Z正方向移动
        
        while remaining_dz > 0.1:  # 当剩余位移大于0.1mm时继续
            # 本次蠕虫周期的位移
            cycle_dz = min(remaining_dz, step_stroke)
            
            # === 阶段1: 后足脱附 + 内折抬起 ===
            # 脱附后足
            motions.append(("000010", pause_time))  # M1=ON, M2=OFF
            
            # 内折（θ3增大，抬起后足，θ3始终为正）
            fold_delta_rad = theta3_fold_angle - max(0.0, self.current_theta3)
            if fold_delta_rad > 0.01:
                duration = fold_delta_rad / omega_w3
                motions.append(("001010", duration))  # ω3=CW, M1=ON, M2=OFF
                self.current_theta3 = theta3_fold_angle
            
            # === 阶段2: 丝杠伸出（后足前移）===
            # 前足吸附时，伸出丝杠使后足向前移动
            if move_forward:
                # 向Z正方向移动：伸出丝杠
                # 尽量使用最大行程，但不超过剩余位移和可用行程
                available_stroke = self.robot.s_max - self.current_s
                # 优先使用step_stroke（7.6mm），但受限于available_stroke和cycle_dz
                actual_stroke = min(step_stroke, available_stroke, cycle_dz)
                if actual_stroke > 0.01:
                    duration = actual_stroke / v_linear
                    motions.append(("001110", duration))  # ω3=CW(保持), v=Fwd, M1=ON, M2=OFF
                    self.current_s += actual_stroke
            else:
                # 向Z负方向移动：收缩丝杠
                available_stroke = self.current_s
                actual_stroke = min(cycle_dz, available_stroke, step_stroke)
                if actual_stroke > 0.01:
                    duration = actual_stroke / v_linear
                    motions.append(("001210", duration))  # ω3=CW(保持), v=Rev, M1=ON, M2=OFF
                    self.current_s -= actual_stroke
            
            # === 阶段3: 放下后足 + 双足吸附 ===
            # 放下后足（θ3恢复到0°，θ3始终为正，内折方向）
            unfold_delta_rad = max(0.0, self.current_theta3)  # 确保非负
            if unfold_delta_rad > 0.01:
                duration = unfold_delta_rad / omega_w3
                motions.append(("002010", duration))  # ω3=CCW, M1=ON, M2=OFF
                self.current_theta3 = 0.0
            
            # 双足吸附
            motions.append(("000011", pause_time))  # M1=ON, M2=ON
            
            # === 阶段4: 前足脱附 + 内折抬起 ===
            # 脱附前足
            motions.append(("000001", pause_time))  # M1=OFF, M2=ON
            
            # 内折（θ3增大，抬起前足，θ3始终为正）
            fold_delta_rad = theta3_fold_angle - max(0.0, self.current_theta3)
            if fold_delta_rad > 0.01:
                duration = fold_delta_rad / omega_w3
                motions.append(("001001", duration))  # ω3=CW, M1=OFF, M2=ON
                self.current_theta3 = theta3_fold_angle
            
            # === 阶段5: 丝杠收缩（前足前移）===
            # 后足吸附时，收缩丝杠使前足向前移动
            actual_stroke_phase5 = 0.0
            if move_forward:
                # 向Z正方向移动：收缩丝杠，前足被"拉"向后足
                # 尽量使用最大行程，但不超过剩余位移和可用行程
                available_stroke = self.current_s
                # 优先使用step_stroke（7.6mm），但受限于available_stroke和cycle_dz
                actual_stroke_phase5 = min(step_stroke, available_stroke, cycle_dz)
                if actual_stroke_phase5 > 0.01:
                    duration = actual_stroke_phase5 / v_linear
                    motions.append(("002201", duration))  # ω3=CCW(放下), v=Rev, M1=OFF, M2=ON
                    self.current_s -= actual_stroke_phase5
            else:
                # 向Z负方向移动：伸出丝杠
                available_stroke = self.robot.s_max - self.current_s
                actual_stroke_phase5 = min(cycle_dz, available_stroke, step_stroke)
                if actual_stroke_phase5 > 0.01:
                    duration = actual_stroke_phase5 / v_linear
                    motions.append(("002101", duration))  # ω3=CCW(放下), v=Fwd, M1=OFF, M2=ON
                    self.current_s += actual_stroke_phase5
            
            # === 阶段6: 放下前足 + 双足吸附 ===
            # 放下前足（θ3恢复到0°，θ3始终为正，内折方向）
            unfold_delta_rad = max(0.0, self.current_theta3)  # 确保非负
            if unfold_delta_rad > 0.01:
                duration = unfold_delta_rad / omega_w3
                motions.append(("002001", duration))  # ω3=CCW, M1=OFF, M2=ON
                self.current_theta3 = 0.0
            
            # 双足吸附
            motions.append(("000011", pause_time))  # M1=ON, M2=ON
            
            # 更新剩余位移（每个完整周期移动 actual_stroke_phase5）
            # 一个完整的蠕虫周期：后足前移 + 前足前移 ≈ 2 * stroke
            # 但由于我们交替吸附，实际前进距离约等于一个stroke
            remaining_dz -= actual_stroke_phase5 if actual_stroke_phase5 > 0 else cycle_dz
            
            # 安全保护：防止无限循环
            if len(motions) > 2000:
                break
        
        # =====================================================
        # 周向运动：通过ω1或ω2旋转实现
        # 注意：纯直线蠕动（dtheta≈0）时跳过此部分
        # =====================================================
        if abs(dtheta) > 0.01:  # 增大阈值，避免微小角度误差触发旋转
            if front_attached:
                # 前节吸附，后节移动
                # 先脱附后足
                motions.append(("000010", pause_time))
                
                # 内折抬起后足（θ3增大，始终为正）
                fold_delta_rad = theta3_fold_angle - max(0.0, self.current_theta3)
                if fold_delta_rad > 0.01:
                    duration = fold_delta_rad / omega_w3
                    motions.append(("001010", duration))
                    self.current_theta3 = theta3_fold_angle
                
                # ω1旋转（周向移动）
                rotate_dir = '1' if dtheta > 0 else '2'
                duration = abs(dtheta) / omega_w1w2
                motions.append((f"{rotate_dir}01010", duration))  # ω1旋转, ω3保持, M1=ON, M2=OFF
                if dtheta > 0:
                    self.current_theta1 += np.degrees(dtheta)
                else:
                    self.current_theta1 -= np.degrees(abs(dtheta))
                
                # 放下后足（θ3恢复到0°，始终为正）
                unfold_delta_rad = max(0.0, self.current_theta3)
                if unfold_delta_rad > 0.01:
                    duration = unfold_delta_rad / omega_w3
                    motions.append(("002010", duration))
                    self.current_theta3 = 0.0
                
                # 双足吸附
                motions.append(("000011", pause_time))
            else:
                # 后节吸附，前节移动
                # 先脱附前足
                motions.append(("000001", pause_time))
                
                # 内折抬起前足（θ3增大，始终为正）
                fold_delta_rad = theta3_fold_angle - max(0.0, self.current_theta3)
                if fold_delta_rad > 0.01:
                    duration = fold_delta_rad / omega_w3
                    motions.append(("001001", duration))
                    self.current_theta3 = theta3_fold_angle
                
                # ω2旋转（周向移动）
                rotate_dir = '1' if dtheta > 0 else '2'
                duration = abs(dtheta) / omega_w1w2
                motions.append((f"0{rotate_dir}1001", duration))  # ω2旋转, ω3保持, M1=OFF, M2=ON
                if dtheta > 0:
                    self.current_theta2 += np.degrees(dtheta)
                else:
                    self.current_theta2 -= np.degrees(abs(dtheta))
                
                # 放下前足（θ3恢复到0°，始终为正）
                unfold_delta_rad = max(0.0, self.current_theta3)
                if unfold_delta_rad > 0.01:
                    duration = unfold_delta_rad / omega_w3
                    motions.append(("002001", duration))
                    self.current_theta3 = 0.0
                
                # 双足吸附
                motions.append(("000011", pause_time))
        
        # 如果没有运动（dz和dtheta都很小），保持当前状态
        if not motions:
            motions.append(("000011", pause_time))
        
        # 限制s在有效范围内
        self.current_s = max(0, min(self.robot.s_max, self.current_s))
        
        # 验证状态码符合约束
        validated_motions = []
        for state_code, duration in motions:
            validated_state = self._validate_state_code(state_code)
            validated_motions.append((validated_state, duration))
        
        return validated_motions
    
    def _validate_state_code(self, state_code: str) -> str:
        """
        验证并修正状态码，确保符合安全约束：
        1. 两磁铁都ON时，所有电机必须停止
        2. 不能两个磁铁都OFF
        
        注意：OFF节的ω电机（ω1或ω2）是可以旋转的！
        虽然OFF节的电磁铁未吸附在壁面上，ω电机旋转看似"空转"，
        但这个旋转可以用来调整电磁铁的角度，避免旋转角度累积超出±360°范围。
        因此不再强制OFF节的ω电机为0。
        """
        if len(state_code) != 6:
            return state_code
        
        w1 = state_code[0]
        w2 = state_code[1]
        w3 = state_code[2]
        v = state_code[3]
        m1 = state_code[4]
        m2 = state_code[5]
        
        # 约束1：两磁铁都ON时，所有电机必须停止
        if m1 == '1' and m2 == '1':
            return "000011"  # 所有电机停止，两磁铁都吸附
        
        # 约束2：不能两个磁铁都OFF
        if m1 == '0' and m2 == '0':
            # 如果检测到两个都OFF，修正为前节吸附（安全默认）
            return w1 + w2 + w3 + v + "10"
        
        # 注意：不再强制OFF节的ω电机为0
        # OFF节的ω电机可以旋转，用于避免角度累积超出±360°范围
        
        return w1 + w2 + w3 + v + m1 + m2
    
    def get_plan_output(self) -> List[Dict]:
        """获取规划输出格式"""
        output = []
        for state_code, duration in self.plan:
            output.append({
                'state': state_code,
                'duration': duration
            })
        return output
    
    def save_plan(self, filename: str):
        """保存规划结果（JSON格式）"""
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(self.get_plan_output(), f, indent=2, ensure_ascii=False)
    
    def save_plan_table(self, filename: str, viz_path: Optional[str] = None):
        """
        保存规划结果为制表符分隔的文本文件，并据此更新运动轨迹可视化图。
        
        包含每一步运动完成后各电机的位置状态
        以及吸附电磁铁在三维圆柱内壁的位置坐标。
        若提供 viz_path（或使用默认 motion_plan_visualization.png），
        会在保存表格后根据 motion_plan_table 重绘并保存该图像。
        
        电机位置范围：
        - θ1, θ2: -360° 到 360°（ω1, ω2旋转电机）
        - θ3: -90° 到 90°（中间旋转电机）
        - s: 0 到 8mm（直线电机）
        
        filename: 表格输出文件名
        viz_path: 可视化图保存路径；默认 'motion_plan_visualization.png'，传 '' 可跳过更新
        """
        with open(filename, 'w', encoding='utf-8') as f:
            # 写入文件头部说明
            f.write("=" * 200 + "\n")
            f.write("Motion Planning Output - Robot Motor States and Attached Magnet Position\n")
            f.write("=" * 200 + "\n")
            f.write("\n")
            f.write("Motor Position Ranges:\n")
            f.write("  - θ1 (ω1 Motor): -360° to +360° (Front section rotation)\n")
            f.write("  - θ2 (ω2 Motor): -360° to +360° (Rear section rotation)\n")
            f.write("  - θ3 (ω3 Motor): -90° to +90° (Inter-section rotation)\n")
            f.write("  - s  (v Motor):  0mm to 8mm (Linear displacement)\n")
            f.write("\n")
            f.write("State Code Format: ω1ω2ω3vm1m2\n")
            f.write("  - ω1,ω2,ω3,v: 0=Stop, 1=Forward/CW, 2=Reverse/CCW\n")
            f.write("  - m1,m2: 0=OFF(detached), 1=ON(attached)\n")
            f.write("\n")
            f.write("Attached Magnet Position:\n")
            f.write("  - Shows the 3D coordinate (X, Y, Z) of the magnet that is attached (ON) to the cylinder inner wall\n")
            f.write("  - Coordinate system: X and Y are radial coordinates, Z is axial coordinate (0-1000mm)\n")
            f.write("  - Cylinder inner wall radius: 900mm\n")
            f.write("  - When both magnets are ON, both positions are shown\n")
            f.write("\n")
            f.write("=" * 200 + "\n\n")
            
            # 写入表头
            header = "Step\tState\tω1\tω2\tω3\tv\tM1\tM2\tDur(s)\tTime(s)\tθ1(°)\tθ2(°)\tθ3(°)\ts(mm)\tAttached Magnet Position (X,Y,Z)\tDescription"
            f.write(header + "\n")
            
            # 写入分隔线
            f.write("-" * 200 + "\n")
            
            # 初始化电机位置状态
            theta1 = 0.0  # 前节旋转电机角度 (度)
            theta2 = 0.0  # 后节旋转电机角度 (度)
            theta3 = 0.0  # 中间旋转电机角度 (度)
            s_pos = 0.0   # 直线电机行程 (mm)
            
            # 初始化机器人在圆柱内壁上的位置
            # 使用plan_path_3d中记录的初始轨迹位置（如果有的话）
            # 否则使用默认值: 前足(0, -900, 40), 后足(0, -900, 0)
            if hasattr(self, 'front_foot_trajectory') and len(self.front_foot_trajectory) > 0:
                front_foot_x = self.front_foot_trajectory[0][0]
                front_foot_y = self.front_foot_trajectory[0][1]
                front_foot_z = self.front_foot_trajectory[0][2]
            else:
                front_foot_x = 0.0
                front_foot_y = -900.0
                front_foot_z = 40.0
            
            if hasattr(self, 'rear_foot_trajectory') and len(self.rear_foot_trajectory) > 0:
                rear_foot_x = self.rear_foot_trajectory[0][0]
                rear_foot_y = self.rear_foot_trajectory[0][1]
                rear_foot_z = self.rear_foot_trajectory[0][2]
            else:
                rear_foot_x = 0.0
                rear_foot_y = -900.0
                rear_foot_z = 0.0
            
            # 圆柱内壁半径
            R = 900.0
            
            # 角速度和线速度（用于计算位置变化）
            # 与_generate_step_motion中的参数保持一致
            # _generate_step_motion使用:
            #   omega_w3 = 0.5 rad/s
            #   omega_w1w2 = 0.5 rad/s  
            #   v_linear = 0.5 单位/s
            # 这里转换为度/秒和mm/秒
            omega_w1w2 = np.degrees(0.5)  # ω1/ω2 角速度 (度/秒)，约28.6°/s
            omega_w3 = np.degrees(0.5)    # ω3 角速度 (度/秒)，约28.6°/s
            v_linear = 0.5                # 丝杠线速度 (mm/秒)
            
            # s_max用于限制范围
            s_max = 8.0  # 直线电机最大行程 (mm)
            
            cumulative_time = 0.0
            
            # 初始化前进方向角φ和上一步吸附的脚
            phi_deg = 90.0  # 初始前进方向角（沿Z轴正方向）
            last_attached_foot = 'rear'  # 初始假设后足吸附，前足在前方
            last_v_stroke = 0.0        # 上一段v运动产生的轴向位移 (mm)
            last_v_moving_foot = None  # 上一段v运动时移动的足 'front'|'rear'
            
            # 写入初始状态（第0步）- 初始时两个电磁铁都吸附
            init_magnet_pos = f"M1:({front_foot_x:.1f},{front_foot_y:.1f},{front_foot_z:.1f}); M2:({rear_foot_x:.1f},{rear_foot_y:.1f},{rear_foot_z:.1f})"
            f.write(f"0\t------\t-\t-\t-\t-\t1\t1\t0.000\t0.000\t{theta1:.1f}\t{theta2:.1f}\t{theta3:.1f}\t{s_pos:.2f}\t{init_magnet_pos}\tInitial State (Both Attached)\n")
            
            for i, (state_code, duration) in enumerate(self.plan, 1):
                cumulative_time += duration
                
                # 解析状态码
                w1 = state_code[0]
                w2 = state_code[1]
                w3 = state_code[2]
                v = state_code[3]
                m1 = state_code[4]
                m2 = state_code[5]
                
                # 计算电机位置变化
                # ω1旋转
                if w1 == '1':
                    theta1 += omega_w1w2 * duration
                elif w1 == '2':
                    theta1 -= omega_w1w2 * duration
                
                # ω2旋转
                if w2 == '1':
                    theta2 += omega_w1w2 * duration
                elif w2 == '2':
                    theta2 -= omega_w1w2 * duration
                
                # ω3旋转（θ3始终为正，内折方向）
                if w3 == '1':
                    theta3 += omega_w3 * duration
                elif w3 == '2':
                    theta3 -= omega_w3 * duration
                # 确保θ3始终在[0, 90]范围内
                theta3 = max(0.0, min(90.0, theta3))
                
                # v直线运动
                # s表示两节之间的相对位移，范围0到s_max
                # v=1表示丝杠正转，v=2表示丝杠反转
                # s增大表示两节拉开，s减小表示两节靠拢
                # 当前节吸附、后节脱附时(m1=1,m2=0)：v=1使后节远离前节(s增大)，v=2使后节靠近前节(s减小)
                # 当后节吸附、前节脱附时(m1=0,m2=1)：v=1使前节远离后节(s增大)，v=2使前节靠近后节(s减小)
                if v == '1':
                    s_pos += v_linear * duration
                    last_v_stroke = v_linear * duration
                    last_v_moving_foot = 'rear' if m1 == '1' else 'front'
                elif v == '2':
                    s_pos -= v_linear * duration
                    last_v_stroke = v_linear * duration
                    last_v_moving_foot = 'rear' if m1 == '1' else 'front'
                
                # 限制电机位置在有效范围内
                # θ1, θ2: -360° 到 360°
                theta1 = max(-360.0, min(360.0, theta1))
                theta2 = max(-360.0, min(360.0, theta2))
                # θ3: 0° 到 90°（始终为正，内折方向）
                theta3 = max(0.0, min(90.0, theta3))
                # s: 0 到 s_max(8mm)
                s_pos = max(0.0, min(s_max, s_pos))
                
                # 关键：当双足都吸附时（状态000011），θ3应该接近0°
                # 这是因为只有θ3≈0°（两节共面）时，两个电磁铁才能同时贴附在壁面上
                # 根据论文中的描述，每一步运动结束时，θ3会被"放下"恢复到接近0°
                if m1 == '1' and m2 == '1' and w3 == '0':
                    # 双足吸附且ω3停止，意味着机器人已经完成一步，θ3应该接近0°
                    theta3 = 0.0
                
                # ======================================================================
                # 根据PDF文档中的运动学公式计算足部位置
                # 
                # 关键公式（PDF 04_21376223_刘丹阳）：
                # - 椭圆方程(2.3): y²/R² + x²/(R/sin(φ))² = 1
                # - 小圆方程(2.4): 已知后足求前足 (x+d2+a)²+(y+R-h)²=(d1+h)²
                # - 小圆方程(2.5): 已知前足求后足 (x-d1)²+(y+R-h)²=(a+d2+h)²
                # - 位置增量(2.6): ΔX = x₀·cos(φ)
                # - 位置增量(2.7): Δθ = arctan(x₀·sin(φ)/y₀)
                #
                # 姿态参数：
                # - φ: 前进方向角（机器人所在平面与轴线的角度），由θ1/θ2旋转改变
                # - a: 直线电机行程，由v运动改变
                # - γ: 翻折角，由θ3运动改变
                #
                # 当一足吸附、另一足脱附时，整个机器人绕吸附足旋转
                # 当θ3使未吸附足"放下"时，该足落点由当前的φ和a决定
                # ======================================================================
                
                # 获取机器人参数
                d1 = self.robot.d1  # 前足到中间电机的距离 (mm)
                d2 = self.robot.d2  # 后足到中间电机的距离 (mm)
                h = self.robot.h    # 中间电机到上表面的高度 (mm)
                
                # 跟踪前进方向角φ的变化
                # 初始φ = 90°（沿Z轴正方向前进）
                # θ1旋转时（前足吸附），φ += Δθ1
                # θ2旋转时（后足吸附），φ += Δθ2
                
                # 跟踪前进方向角φ的变化
                # 初始φ = 90°（沿Z轴正方向前进）
                # θ1旋转时（前足吸附），φ += Δθ1
                # θ2旋转时（后足吸附），φ += Δθ2
                
                # 根据电机旋转更新前进方向角φ
                if m1 == '1' and m2 == '0':
                    # 前足吸附
                    if w1 == '1':
                        phi_deg += omega_w1w2 * duration
                    elif w1 == '2':
                        phi_deg -= omega_w1w2 * duration
                    last_attached_foot = 'front'
                    
                elif m1 == '0' and m2 == '1':
                    # 后足吸附
                    if w2 == '1':
                        phi_deg += omega_w1w2 * duration
                    elif w2 == '2':
                        phi_deg -= omega_w1w2 * duration
                    last_attached_foot = 'rear'
                
                # 关键：位置更新逻辑
                phi_rad = np.radians(phi_deg)
                a = s_pos  # 当前直线电机行程
                
                # 1. 丝杠运动阶段：更新移动足的位置
                if m1 == '1' and m2 == '0' and v != '0':  # 前足吸附，后足移动
                    delta_X, delta_theta = self.robot.calc_other_foot_position(
                        fixed_foot_X=front_foot_z, fixed_foot_theta=np.arctan2(front_foot_y, front_foot_x),
                        phi=phi_rad, a=a, R=R, find_front=False
                    )
                    rear_foot_x = R * np.cos(np.arctan2(front_foot_y, front_foot_x) + delta_theta)
                    rear_foot_y = R * np.sin(np.arctan2(front_foot_y, front_foot_x) + delta_theta)
                    rear_foot_z = front_foot_z + delta_X
                    
                elif m1 == '0' and m2 == '1' and v != '0':  # 后足吸附，前足移动
                    delta_X, delta_theta = self.robot.calc_other_foot_position(
                        fixed_foot_X=rear_foot_z, fixed_foot_theta=np.arctan2(rear_foot_y, rear_foot_x),
                        phi=phi_rad, a=a, R=R, find_front=True
                    )
                    front_foot_x = R * np.cos(np.arctan2(rear_foot_y, rear_foot_x) + delta_theta)
                    front_foot_y = R * np.sin(np.arctan2(rear_foot_y, rear_foot_x) + delta_theta)
                    front_foot_z = rear_foot_z + delta_X
                
                # 2. 双足吸附阶段：精确校准刚放下的足的位置
                if m1 == '1' and m2 == '1' and w3 == '0':
                    if last_attached_foot == 'front':
                        delta_X, delta_theta = self.robot.calc_other_foot_position(
                            fixed_foot_X=front_foot_z, fixed_foot_theta=np.arctan2(front_foot_y, front_foot_x),
                            phi=phi_rad, a=a, R=R, find_front=False
                        )
                        rear_foot_z = front_foot_z + delta_X
                        rear_theta_rad = np.arctan2(front_foot_y, front_foot_x) + delta_theta
                        rear_foot_x = R * np.cos(rear_theta_rad)
                        rear_foot_y = R * np.sin(rear_theta_rad)
                    elif last_attached_foot == 'rear':
                        delta_X, delta_theta = self.robot.calc_other_foot_position(
                            fixed_foot_X=rear_foot_z, fixed_foot_theta=np.arctan2(rear_foot_y, rear_foot_x),
                            phi=phi_rad, a=a, R=R, find_front=True
                        )
                        front_foot_z = rear_foot_z + delta_X
                        front_theta_rad = np.arctan2(rear_foot_y, rear_foot_x) + delta_theta
                        front_foot_x = R * np.cos(front_theta_rad)
                        front_foot_y = R * np.sin(front_theta_rad)
                
                # 限制Z坐标在有效范围内
                front_foot_z = max(0.0, min(1000.0, front_foot_z))
                rear_foot_z = max(0.0, min(1000.0, rear_foot_z))
                
                # 生成吸附电磁铁位置字符串
                magnet_pos_parts = []
                if m1 == '1':
                    magnet_pos_parts.append(f"M1:({front_foot_x:.1f},{front_foot_y:.1f},{front_foot_z:.1f})")
                if m2 == '1':
                    magnet_pos_parts.append(f"M2:({rear_foot_x:.1f},{rear_foot_y:.1f},{rear_foot_z:.1f})")
                magnet_pos = "; ".join(magnet_pos_parts) if magnet_pos_parts else "None (Both OFF)"
                
                # 生成描述
                desc_parts = []
                if w1 != '0':
                    desc_parts.append(f"ω1={'CW' if w1 == '1' else 'CCW'}")
                if w2 != '0':
                    desc_parts.append(f"ω2={'CW' if w2 == '1' else 'CCW'}")
                if w3 != '0':
                    desc_parts.append(f"ω3={'CW' if w3 == '1' else 'CCW'}")
                if v != '0':
                    desc_parts.append(f"v={'Fwd' if v == '1' else 'Rev'}")
                desc_parts.append(f"M1={'ON' if m1 == '1' else 'OFF'}")
                desc_parts.append(f"M2={'ON' if m2 == '1' else 'OFF'}")
                
                description = ", ".join(desc_parts) if desc_parts else "All Stop"
                
                # 写入数据行（使用制表符分隔）
                line = f"{i}\t{state_code}\t{w1}\t{w2}\t{w3}\t{v}\t{m1}\t{m2}\t{duration:.3f}\t{cumulative_time:.3f}\t{theta1:.1f}\t{theta2:.1f}\t{theta3:.1f}\t{s_pos:.2f}\t{magnet_pos}\t{description}"
                f.write(line + "\n")
            
            # 写入分隔线
            f.write("-" * 200 + "\n")
            
            # 写入汇总信息
            f.write("\n")
            f.write("=" * 200 + "\n")
            f.write("Summary:\n")
            f.write("=" * 200 + "\n")
            f.write(f"Total Steps: {len(self.plan)}\n")
            f.write(f"Total Time: {cumulative_time:.3f} s\n")
            f.write("\n")
            f.write("Final Motor Positions:\n")
            f.write(f"  - θ1 (Front Section): {theta1:.1f}°\n")
            f.write(f"  - θ2 (Rear Section):  {theta2:.1f}°\n")
            f.write(f"  - θ3 (Inter-section): {theta3:.1f}°\n")
            f.write(f"  - s  (Linear Motor):  {s_pos:.2f} mm\n")
            f.write("\n")
            f.write("Final Robot Foot Positions on Cylinder Inner Wall:\n")
            f.write(f"  - Front Foot (M1): X={front_foot_x:.1f}mm, Y={front_foot_y:.1f}mm, Z={front_foot_z:.1f}mm\n")
            f.write(f"  - Rear Foot  (M2): X={rear_foot_x:.1f}mm, Y={rear_foot_y:.1f}mm, Z={rear_foot_z:.1f}mm\n")
            f.write("\n")
            
            # 检查并报告位置是否接近极限
            warnings = []
            if abs(theta1) > 330:
                warnings.append(f"Warning: θ1 ({theta1:.1f}°) is close to ±360° limit!")
            if abs(theta2) > 330:
                warnings.append(f"Warning: θ2 ({theta2:.1f}°) is close to ±360° limit!")
            if abs(theta3) > 80:
                warnings.append(f"Warning: θ3 ({theta3:.1f}°) is close to ±90° limit!")
            if s_pos < 0.5 or s_pos > 7.5:
                warnings.append(f"Warning: s ({s_pos:.2f}mm) is close to 0-8mm limit!")
            
            if warnings:
                f.write("Position Warnings:\n")
                for w in warnings:
                    f.write(f"  {w}\n")
            else:
                f.write("All motor positions are within safe range.\n")
            
            f.write("=" * 200 + "\n")
        
        # 每次保存运动逻辑后自动更新 motion_plan_visualization 图像
        out_viz = viz_path if viz_path is not None else 'motion_plan_visualization.png'
        if out_viz and hasattr(self, '_viz_start') and hasattr(self, '_viz_goal'):
            try:
                front_z = getattr(self, '_viz_front_start_z', None)
                rear_z = getattr(self, '_viz_rear_start_z', None)
                goal_z = getattr(self, '_viz_front_goal_z', None)
                fig = visualize_plan(
                    self,
                    self.surface,
                    self._viz_start,
                    self._viz_goal,
                    front_start_z=front_z,
                    rear_start_z=rear_z,
                    front_goal_z=goal_z,
                    table_path=filename,
                )
                fig.savefig(out_viz, dpi=150, bbox_inches='tight')
                plt.close(fig)
                print(f"已更新可视化: {out_viz}")
            except Exception as e:
                print(f"更新可视化失败: {e}")
    
    def load_plan(self, filename: str):
        """加载规划结果"""
        with open(filename, 'r', encoding='utf-8') as f:
            data = json.load(f)
            self.plan = [(item['state'], item['duration']) for item in data]


def visualize_robot_structure(robot: Robot):
    """
    可视化机器人结构
    绘制双节机器人的结构示意图
    """
    fig = plt.figure(figsize=(16, 10))
    
    # 1. 侧视图（YZ平面）
    ax1 = fig.add_subplot(231)
    # 绘制前节（矩形）
    front_rect = plt.Rectangle((0, 0), robot.L1, robot.W1, 
                               fill=True, facecolor='lightblue', 
                               edgecolor='blue', linewidth=2)
    ax1.add_patch(front_rect)
    # 前节电磁铁（圆柱形，从下往上）
    magnet1 = plt.Circle((robot.L1/2, -robot.h), robot.magnet_radius, 
                        fill=True, color='darkblue')
    ax1.add_patch(magnet1)
    ax1.plot([robot.L1/2, robot.L1/2], [-robot.h, 0], 'b-', linewidth=2, label='Magnet 1')
    
    # 绘制后节（矩形，初始位置与前节共面）
    rear_rect = plt.Rectangle((robot.L1, 0), robot.L2, robot.W2, 
                              fill=True, facecolor='lightcoral', 
                              edgecolor='red', linewidth=2)
    ax1.add_patch(rear_rect)
    # 后节电磁铁
    magnet2 = plt.Circle((robot.L1 + robot.L2/2, -robot.h), robot.magnet_radius, 
                        fill=True, color='darkred')
    ax1.add_patch(magnet2)
    ax1.plot([robot.L1 + robot.L2/2, robot.L1 + robot.L2/2], [-robot.h, 0], 'r-', linewidth=2, label='Magnet 2')
    
    # 绘制旋转轴（ω3，在上表面共面）
    ax1.plot([robot.L1, robot.L1], [0, robot.W1], 'g--', linewidth=2, alpha=0.7, label='ω3 Axis (on top surface)')
    # 绘制直线电机（丝杠）位移范围
    ax1.arrow(robot.L1, robot.W1/2, robot.s_max, 0, head_width=2, head_length=2, 
             fc='orange', ec='orange', linewidth=1, alpha=0.5, label=f'Linear Motor (v, max {robot.s_max}mm)')
    
    ax1.set_xlabel('Length (mm)')
    ax1.set_ylabel('Width/Height (mm)')
    ax1.set_title('Side View (Initial Position)')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_xlim(-10, robot.L1 + robot.L2 + 10)
    ax1.set_ylim(-robot.h - 10, max(robot.W1, robot.W2) + 10)
    
    # 2. 俯视图（XY平面）- 前节
    ax2 = fig.add_subplot(232)
    front_top = plt.Rectangle((0, 0), robot.L1, robot.W1, 
                             fill=True, facecolor='lightblue', 
                             edgecolor='blue', linewidth=2)
    ax2.add_patch(front_top)
    # 前节旋转电机中心（ω1）
    ax2.plot(robot.L1/2, robot.W1/2, 'bo', markersize=10, label='ω1 Motor')
    ax2.plot([robot.L1/2, robot.L1/2 + 10], [robot.W1/2, robot.W1/2], 
            'b-', linewidth=2, label='Rotation Axis')
    ax2.set_xlabel('Length L1 (mm)')
    ax2.set_ylabel('Width W1 (mm)')
    ax2.set_title('Top View - Front Section')
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_xlim(-5, robot.L1 + 5)
    ax2.set_ylim(-5, robot.W1 + 5)
    
    # 3. 俯视图 - 后节
    ax3 = fig.add_subplot(233)
    rear_top = plt.Rectangle((0, 0), robot.L2, robot.W2, 
                            fill=True, facecolor='lightcoral', 
                            edgecolor='red', linewidth=2)
    ax3.add_patch(rear_top)
    # 后节旋转电机中心（ω2）
    ax3.plot(robot.L2/2, robot.W2/2, 'ro', markersize=10, label='ω2 Motor')
    ax3.plot([robot.L2/2, robot.L2/2 + 10], [robot.W2/2, robot.W2/2], 
            'r-', linewidth=2, label='Rotation Axis')
    ax3.set_xlabel('Length L2 (mm)')
    ax3.set_ylabel('Width W2 (mm)')
    ax3.set_title('Top View - Rear Section')
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    ax3.set_xlim(-5, robot.L2 + 5)
    ax3.set_ylim(-5, robot.W2 + 5)
    
    # 4. 3D结构图
    ax4 = fig.add_subplot(234, projection='3d')
    
    # 前节（蓝色长方体）
    front_x = [0, robot.L1, robot.L1, 0, 0, robot.L1, robot.L1, 0]
    front_y = [0, 0, robot.W1, robot.W1, 0, 0, robot.W1, robot.W1]
    front_z = [0, 0, 0, 0, robot.h, robot.h, robot.h, robot.h]
    
    # 绘制前节框架
    for i in range(4):
        ax4.plot3D([front_x[i], front_x[i+4]], [front_y[i], front_y[i+4]], 
                  [front_z[i], front_z[i+4]], 'b-', linewidth=2)
    
    # 前节电磁铁（圆柱形，从底面向上）
    theta_magnet = np.linspace(0, 2*np.pi, 20)
    magnet1_x = robot.L1/2 + robot.magnet_radius * np.cos(theta_magnet)
    magnet1_y = robot.W1/2 + robot.magnet_radius * np.sin(theta_magnet)
    ax4.plot3D(magnet1_x, magnet1_y, [0]*len(magnet1_x), 'b-', linewidth=2)
    ax4.plot3D(magnet1_x, magnet1_y, [robot.h]*len(magnet1_x), 'b-', linewidth=2)
    
    # 后节（红色长方体，初始与前节共面）
    rear_x = [robot.L1, robot.L1+robot.L2, robot.L1+robot.L2, robot.L1, 
             robot.L1, robot.L1+robot.L2, robot.L1+robot.L2, robot.L1]
    rear_y = [0, 0, robot.W2, robot.W2, 0, 0, robot.W2, robot.W2]
    rear_z = [0, 0, 0, 0, robot.h, robot.h, robot.h, robot.h]
    
    # 绘制后节框架
    for i in range(4):
        ax4.plot3D([rear_x[i], rear_x[i+4]], [rear_y[i], rear_y[i+4]], 
                  [rear_z[i], rear_z[i+4]], 'r-', linewidth=2)
    
    # 后节电磁铁
    magnet2_x = robot.L1 + robot.L2/2 + robot.magnet_radius * np.cos(theta_magnet)
    magnet2_y = robot.W2/2 + robot.magnet_radius * np.sin(theta_magnet)
    ax4.plot3D(magnet2_x, magnet2_y, [0]*len(magnet2_x), 'r-', linewidth=2)
    ax4.plot3D(magnet2_x, magnet2_y, [robot.h]*len(magnet2_x), 'r-', linewidth=2)
    
    # 连接轴（ω3旋转轴，在上表面共面）
    ax4.plot3D([robot.L1, robot.L1], [robot.W1/2, robot.W2/2], 
              [robot.h, robot.h], 'g-', linewidth=3, label='ω3 Axis (on top surface)')
    
    ax4.set_xlabel('Length (mm)')
    ax4.set_ylabel('Width (mm)')
    ax4.set_zlabel('Height (mm)')
    ax4.set_title('3D Structure View')
    ax4.legend()
    
    # 5. 参数说明表格
    ax5 = fig.add_subplot(235)
    ax5.axis('off')
    params_text = f"""
    Robot Structure Parameters:
    
    Front Section:
      Length (L1): {robot.L1} mm
      Width (W1):  {robot.W1} mm
      Rotation Motor: ω1 (center)
      Magnet: Cylindrical, radius {robot.magnet_radius} mm
    
    Rear Section:
      Length (L2): {robot.L2} mm
      Width (W2):  {robot.W2} mm
      Rotation Motor: ω2 (center)
      Magnet: Cylindrical, radius {robot.magnet_radius} mm
    
    Common:
      Height (h): {robot.h} mm (magnet bottom to top surface)
      Linear Motor (v): Max travel {robot.s_max} mm
      Relative Rotation (θ3): {np.degrees(robot.theta3_min):.0f}° to {np.degrees(robot.theta3_max):.0f}°
                            (0° = coplanar position)
    
    Motors:
      ω1: Front section magnet rotation (±360°)
      ω2: Rear section magnet rotation (±360°)
      ω3: Relative rotation between sections (-90° to 90°)
      v:  Linear displacement (max {robot.s_max} mm)
    """
    ax5.text(0.1, 0.5, params_text, fontsize=10, verticalalignment='center',
            family='monospace', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 6. 运动范围示意图
    ax6 = fig.add_subplot(236)
    
    # 绘制丝杠行程范围
    center_x = robot.L1 + robot.L2/2
    center_y = robot.W1/2
    ax6.plot([center_x - robot.s_max, center_x + robot.s_max], 
            [center_y, center_y], 'orange', linewidth=4, label=f'Linear Motor Range (±{robot.s_max}mm)')
    
    # 绘制角度范围示意（θ3）
    angles = np.linspace(robot.theta3_min, robot.theta3_max, 20)
    for angle in [robot.theta3_min, 0, robot.theta3_max]:
        if angle == 0:
            style = 'g-'
            label = 'θ3 = 0° (Coplanar)'
        else:
            style = 'g--'
            label = None
        end_x = center_x + 20 * np.cos(angle)
        end_y = center_y + 20 * np.sin(angle)
        ax6.plot([center_x, end_x], [center_y, end_y], style, linewidth=2, label=label)
    
    # 绘制角度范围弧
    theta_arc = np.linspace(robot.theta3_min, robot.theta3_max, 30)
    arc_x = center_x + 15 * np.cos(theta_arc)
    arc_y = center_y + 15 * np.sin(theta_arc)
    ax6.plot(arc_x, arc_y, 'g:', linewidth=1, alpha=0.5, label='θ3 Range (-90° to 90°)')
    
    ax6.plot(center_x, center_y, 'ko', markersize=8, label='Rotation Center (ω3)')
    ax6.set_xlabel('X (mm)')
    ax6.set_ylabel('Y (mm)')
    ax6.set_title('Motion Range')
    ax6.set_aspect('equal')
    ax6.grid(True, alpha=0.3)
    ax6.legend(fontsize=8)
    ax6.set_xlim(center_x - 30, center_x + 30)
    ax6.set_ylim(center_y - 30, center_y + 30)
    
    plt.tight_layout()
    return fig


def visualize_robot_state(robot: Robot, theta3: float, s: float, title: str = ""):
    """
    可视化机器人在特定状态下的结构
    theta3: 两节相对旋转角度（弧度）
    s: 直线电机位移（mm，相对于初始位置）
    title: 图像标题
    """
    fig = plt.figure(figsize=(14, 8))
    
    # 1. 侧视图（考虑θ3和s的影响）
    ax1 = fig.add_subplot(121)
    
    # 前节位置（固定）
    front_center_x = robot.L1 / 2
    front_center_y = robot.W1 / 2
    
    # 计算后节位置（考虑s和θ3）
    # 初始时，后节中心在前节右端
    rear_center_x_init = robot.L1 + robot.L2 / 2
    rear_center_y_init = robot.W2 / 2
    
    # 考虑直线电机位移s（沿前节长度方向）
    rear_center_x = rear_center_x_init + s
    rear_center_y = rear_center_y_init
    
    # ω3旋转轴位置：在两节上表面的交线上
    # 初始时，交线在前节右端（x = L1），y方向在W1/2位置
    # 旋转后，交线位置会变化，但始终在两节上表面的交线上
    rotation_center_x = robot.L1  # 旋转中心在前节右端（上表面边缘）
    rotation_center_y = robot.W1 / 2  # 旋转中心在宽度中心
    
    # 计算旋转后的后节中心位置
    # 后节相对于前节的旋转：绕旋转中心（两节上表面交线）旋转
    dx = rear_center_x - rotation_center_x
    dy = rear_center_y - rotation_center_y
    rear_center_x_rot = rotation_center_x + dx * np.cos(theta3) - dy * np.sin(theta3)
    rear_center_y_rot = rotation_center_y + dx * np.sin(theta3) + dy * np.cos(theta3)
    
    # 计算旋转后两节上表面的交线位置
    # 交线在前节右边缘和后节左边缘的交点
    # 前节右边缘：x = L1, y ∈ [0, W1]
    # 后节左边缘：经过旋转后的后节左边缘
    # 交线点：前节右边缘与后节左边缘的交点
    # 简化：交线始终在旋转中心（前节右端中心）
    intersection_x = rotation_center_x
    intersection_y = rotation_center_y
    
    # 绘制前节
    front_rect = plt.Rectangle((0, 0), robot.L1, robot.W1,
                              fill=True, facecolor='lightblue',
                              edgecolor='blue', linewidth=2, label='Front Section')
    ax1.add_patch(front_rect)
    
    # 前节电磁铁（圆柱形，中心在上表面中心，垂直向下）
    # 电磁铁与ω1旋转电机共轴，轴线垂直于上表面
    # 电磁铁底面与上表面的距离为h，所以底面y坐标为 -h
    magnet1_bottom_y = -robot.h  # 电磁铁底面（y坐标）
    magnet1_top_y = 0  # 上表面（y坐标）
    magnet1_center_x = front_center_x
    magnet1_center_y = (magnet1_bottom_y + magnet1_top_y) / 2  # 圆柱中心
    magnet1 = plt.Circle((magnet1_center_x, magnet1_center_y), robot.magnet_radius,
                        fill=True, color='darkblue', label='Magnet 1 (cylindrical)')
    ax1.add_patch(magnet1)
    # 绘制电磁铁轴线（与ω1旋转轴共线，垂直）
    ax1.plot([magnet1_center_x, magnet1_center_x], [magnet1_bottom_y, magnet1_top_y],
            'b-', linewidth=2, label='ω1 Axis / Magnet 1 Axis (vertical)')
    
    # 绘制后节（旋转和平移后）
    # 计算后节四个角点（相对于后节中心）
    half_L2 = robot.L2 / 2
    half_W2 = robot.W2 / 2
    rear_corners_local = [
        [-half_L2, -half_W2],
        [half_L2, -half_W2],
        [half_L2, half_W2],
        [-half_L2, half_W2]
    ]
    
    # 旋转和平移后节角点（侧视图投影）
    # 注意：侧视图是XY平面投影，后节上表面会随θ3旋转而倾斜
    # 在侧视图中，我们需要显示后节上表面在XY平面的投影
    # 由于后节绕交线旋转，在侧视图中的投影会显示为倾斜的矩形
    rear_corners_rotated = []
    for corner in rear_corners_local:
        # 先旋转（在XY平面内）
        x_rot = corner[0] * np.cos(theta3) - corner[1] * np.sin(theta3)
        y_rot = corner[0] * np.sin(theta3) + corner[1] * np.cos(theta3)
        # 再平移到后节中心位置
        rear_corners_rotated.append([
            rear_center_x_rot + x_rot,
            rear_center_y_rot + y_rot
        ])
    
    # 绘制后节（多边形，在侧视图中显示为XY平面的投影）
    rear_polygon = plt.Polygon(rear_corners_rotated, fill=True, facecolor='lightcoral',
                              edgecolor='red', linewidth=2, label='Rear Section (top view projection)')
    ax1.add_patch(rear_polygon)
    
    # 注意：侧视图中，后节上表面显示为XY平面的投影，但实际上后节上表面是倾斜的
    # 后节电磁铁的位置应该反映上表面的倾斜
    
    # 后节电磁铁（圆柱形，在后节中心，垂直向下）
    # 电磁铁与ω2旋转电机共轴，轴线垂直于后节上表面
    # 注意：后节上表面始终是水平的，所以电磁铁轴线始终是垂直的（y方向向下）
    # 在侧视图中，y轴是垂直方向
    # 重要：后节电磁铁底面相对于后节上表面计算，当θ3≠0时，与前节电磁铁底面不共面
    magnet2_top_y = rear_center_y_rot  # 后节上表面中心（y坐标）
    magnet2_bottom_y = rear_center_y_rot - robot.h  # 电磁铁底面（y坐标，相对于后节上表面）
    magnet2_center_x = rear_center_x_rot
    magnet2_center_y = (magnet2_bottom_y + magnet2_top_y) / 2  # 圆柱中心
    magnet2 = plt.Circle((magnet2_center_x, magnet2_center_y), robot.magnet_radius,
                        fill=True, color='darkred', label='Magnet 2 (cylindrical)')
    ax1.add_patch(magnet2)
    # 绘制电磁铁轴线（与ω2旋转轴共线，垂直向下，始终垂直于后节上表面）
    ax1.plot([magnet2_center_x, magnet2_center_x], 
            [magnet2_bottom_y, magnet2_top_y],
            'r-', linewidth=2, label='ω2 Axis / Magnet 2 Axis (vertical, perpendicular to rear top surface)')
    
    # 绘制旋转轴（ω3，在两节上表面的交线上）
    # 交线在前节右边缘（x = L1）上
    # 由于后节旋转，交线位置会变化，但始终通过旋转中心
    ax1.plot([rotation_center_x, rotation_center_x], [0, max(robot.W1, robot.W2) + 5],
            'g--', linewidth=2, alpha=0.7, label='ω3 Axis (on intersection line)')
    
    # 绘制旋转角度指示
    angle_arc = np.linspace(0, theta3, 20)
    arc_radius = 15
    arc_x = rotation_center_x + arc_radius * np.cos(angle_arc)
    arc_y = rotation_center_y + arc_radius * np.sin(angle_arc)
    ax1.plot(arc_x, arc_y, 'g-', linewidth=1, alpha=0.5)
    ax1.text(rotation_center_x + arc_radius * 1.2, rotation_center_y + arc_radius * 1.2,
            f'θ3 = {np.degrees(theta3):.1f}°', fontsize=10, color='green')
    
    # 绘制直线电机位移指示
    if abs(s) > 0.01:
        ax1.arrow(rotation_center_x, rotation_center_y, s, 0,
                 head_width=2, head_length=2, fc='orange', ec='orange',
                 linewidth=2, label=f'Linear Motor s = {s:.1f}mm')
    
    ax1.set_xlabel('Length (mm)')
    ax1.set_ylabel('Width (mm)')
    ax1.set_title(f'Side View - {title}' if title else 'Side View')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=8)
    
    # 设置合适的显示范围
    all_x = [0, robot.L1] + [p[0] for p in rear_corners_rotated]
    all_y = [0, robot.W1] + [p[1] for p in rear_corners_rotated]
    ax1.set_xlim(min(all_x) - 10, max(all_x) + 10)
    ax1.set_ylim(-robot.h - 10, max(all_y) + 10)
    
    # 2. 俯视图（从上往下看）
    ax2 = fig.add_subplot(122, projection='3d')
    
    # 前节（上表面，z = h）
    front_corners_3d = [
        [0, 0, robot.h],
        [robot.L1, 0, robot.h],
        [robot.L1, robot.W1, robot.h],
        [0, robot.W1, robot.h]
    ]
    front_corners_3d.append(front_corners_3d[0])  # 闭合
    
    front_x_3d = [p[0] for p in front_corners_3d]
    front_y_3d = [p[1] for p in front_corners_3d]
    front_z_3d = [p[2] for p in front_corners_3d]
    ax2.plot3D(front_x_3d, front_y_3d, front_z_3d, 'b-', linewidth=2, label='Front Section')
    
    # 前节电磁铁中心轴（垂直，与ω1旋转轴共线，垂直于前节上表面）
    # 前节上表面是水平的（z = h），所以法向量是垂直的
    ax2.plot3D([front_center_x, front_center_x], [front_center_y, front_center_y],
              [0, robot.h], 'b-', linewidth=3, label='ω1 Axis / Magnet 1 Axis (vertical, perpendicular to front top surface)')
    
    # 绘制前节电磁铁（圆柱形，直径5mm）
    # 生成圆柱体的网格
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    theta_cyl = np.linspace(0, 2*np.pi, 20)
    z_cyl = np.linspace(0, robot.h, 2)
    theta_grid, z_grid = np.meshgrid(theta_cyl, z_cyl)
    x_cyl = front_center_x + robot.magnet_radius * np.cos(theta_grid)
    y_cyl = front_center_y + robot.magnet_radius * np.sin(theta_grid)
    ax2.plot_surface(x_cyl, y_cyl, z_grid, alpha=0.6, color='darkblue', label='Magnet 1 (Cylinder, D=5mm)')
    
    # 绘制上下底面圆圈
    theta_circle = np.linspace(0, 2*np.pi, 50)
    x_top_circle = front_center_x + robot.magnet_radius * np.cos(theta_circle)
    y_top_circle = front_center_y + robot.magnet_radius * np.sin(theta_circle)
    z_top_circle = np.full_like(theta_circle, robot.h)
    ax2.plot3D(x_top_circle, y_top_circle, z_top_circle, 'b-', linewidth=2)
    z_bottom_circle = np.zeros_like(theta_circle)
    ax2.plot3D(x_top_circle, y_top_circle, z_bottom_circle, 'b-', linewidth=2)
    
    # 后节上表面计算（绕交线旋转，形成"折叠"状态）
    # 交线：前节右边缘，x = L1, y ∈ [0, W1], z = h（通过点 (L1, W1/2, h)，方向沿y轴）
    intersection_line_point = np.array([robot.L1, robot.W1/2, robot.h])
    intersection_line_dir = np.array([0, 1, 0])  # 沿y方向
    
    # 后节上表面初始时与前节共面（z = h），绕交线旋转θ3角度
    # 计算后节中心在初始位置的3D坐标（相对于后节初始中心）
    rear_center_init_3d = np.array([rear_center_x_init + s, rear_center_y_init, robot.h])
    
    # 计算后节中心相对于交线上某点的向量
    vec_to_rear_center = rear_center_init_3d - intersection_line_point
    
    # 绕交线旋转（使用Rodrigues旋转公式）
    k = intersection_line_dir / np.linalg.norm(intersection_line_dir)
    cos_theta = np.cos(theta3)
    sin_theta = np.sin(theta3)
    
    # Rodrigues旋转公式
    rotated_vec_center = (vec_to_rear_center * cos_theta + 
                         np.cross(k, vec_to_rear_center) * sin_theta + 
                         k * np.dot(k, vec_to_rear_center) * (1 - cos_theta))
    
    # 旋转后的后节中心
    rear_center_3d_rotated = intersection_line_point + rotated_vec_center
    
    # 计算后节四个角点的3D坐标（绕交线旋转）
    rear_corners_3d = []
    for corner in rear_corners_local:
        # 后节角点在初始位置的3D坐标（相对于后节中心）
        corner_init_3d = rear_center_init_3d + np.array([corner[0], corner[1], 0])
        
        # 计算相对于交线的向量
        vec_to_corner = corner_init_3d - intersection_line_point
        
        # 绕交线旋转
        rotated_vec_corner = (vec_to_corner * cos_theta + 
                             np.cross(k, vec_to_corner) * sin_theta + 
                             k * np.dot(k, vec_to_corner) * (1 - cos_theta))
        
        # 旋转后的角点
        corner_rotated_3d = intersection_line_point + rotated_vec_corner
        rear_corners_3d.append(corner_rotated_3d.tolist())
    
    rear_corners_3d.append(rear_corners_3d[0])  # 闭合
    
    rear_x_3d = [p[0] for p in rear_corners_3d]
    rear_y_3d = [p[1] for p in rear_corners_3d]
    rear_z_3d = [p[2] for p in rear_corners_3d]
    ax2.plot3D(rear_x_3d, rear_y_3d, rear_z_3d, 'r-', linewidth=2, label='Rear Section')
    
    # 计算后节上表面的法向量（垂直于后节上表面）
    # 使用后节上表面的两个边向量计算法向量
    if len(rear_corners_3d) >= 4:
        v1 = np.array(rear_corners_3d[1]) - np.array(rear_corners_3d[0])
        v2 = np.array(rear_corners_3d[3]) - np.array(rear_corners_3d[0])
        rear_surface_normal = np.cross(v1, v2)
        rear_surface_normal = rear_surface_normal / np.linalg.norm(rear_surface_normal)
    else:
        # 备用：如果计算失败，使用默认垂直方向
        rear_surface_normal = np.array([0, 0, 1])
    
    # 后节电磁铁中心轴（与ω2旋转轴共线，垂直于后节上表面）
    # 后节电磁铁轴线方向就是后节上表面的法向量方向
    magnet2_top_3d = rear_center_3d_rotated  # 后节上表面中心
    magnet2_bottom_3d = magnet2_top_3d - rear_surface_normal * robot.h  # 电磁铁底面中心（沿法向量向下）
    
    ax2.plot3D([magnet2_bottom_3d[0], magnet2_top_3d[0]], 
              [magnet2_bottom_3d[1], magnet2_top_3d[1]],
              [magnet2_bottom_3d[2], magnet2_top_3d[2]], 
              'r-', linewidth=3, label='ω2 Axis / Magnet 2 Axis (perpendicular to rear top surface)')
    
    # 绘制后节电磁铁（圆柱形，直径5mm，垂直于后节上表面）
    # 需要在垂直于后节上表面的方向上绘制圆柱体
    # 计算垂直于后节上表面的两个正交向量（用于生成圆柱体的圆周）
    # 使用后节上表面的两个边向量
    if len(rear_corners_3d) >= 4:
        v1 = np.array(rear_corners_3d[1]) - np.array(rear_corners_3d[0])
        v1 = v1 / np.linalg.norm(v1)
        # 第二个向量垂直于法向量和v1
        v2 = np.cross(rear_surface_normal, v1)
        v2 = v2 / np.linalg.norm(v2)
    else:
        v1 = np.array([1, 0, 0])
        v2 = np.array([0, 1, 0])
    
    # 生成圆柱体表面的点
    theta_cyl2 = np.linspace(0, 2*np.pi, 20)
    h_steps = np.linspace(0, robot.h, 5)
    
    for h_val in h_steps:
        # 圆柱体上每个高度层的圆周点
        circle_points = []
        for theta in theta_cyl2:
            # 在垂直于圆柱轴线的平面内生成点
            offset_vec = robot.magnet_radius * (np.cos(theta) * v1 + np.sin(theta) * v2)
            point_on_cylinder = magnet2_top_3d - rear_surface_normal * h_val + offset_vec
            circle_points.append(point_on_cylinder)
        
        # 绘制圆周
        circle_x = [p[0] for p in circle_points]
        circle_y = [p[1] for p in circle_points]
        circle_z = [p[2] for p in circle_points]
        if h_val == 0:
            ax2.plot3D(circle_x, circle_y, circle_z, 'r-', linewidth=2)  # 上表面
        elif abs(h_val - robot.h) < 0.01:
            ax2.plot3D(circle_x, circle_y, circle_z, 'r-', linewidth=2)  # 下表面
    
    # 绘制圆柱体的侧面线（沿高度方向）
    for i in range(0, len(theta_cyl2), 4):
        theta = theta_cyl2[i]
        offset_vec = robot.magnet_radius * (np.cos(theta) * v1 + np.sin(theta) * v2)
        top_point = magnet2_top_3d + offset_vec
        bottom_point = magnet2_bottom_3d + offset_vec
        ax2.plot3D([bottom_point[0], top_point[0]], 
                  [bottom_point[1], top_point[1]],
                  [bottom_point[2], top_point[2]], 'r-', linewidth=1, alpha=0.5)
    
    # ω3旋转轴：在两节上表面的交线上
    # 交线是前节右边缘（x = L1, y ∈ [0, W1], z = h）
    # 交线方向沿y轴：从 (L1, 0, h) 到 (L1, W1, h)
    intersection_line_start = np.array([robot.L1, 0, robot.h])
    intersection_line_end = np.array([robot.L1, robot.W1, robot.h])
    
    # 绘制ω3旋转轴（在交线上）
    ax2.scatter([intersection_line_point[0]], [intersection_line_point[1]], [intersection_line_point[2]], 
               c='green', s=100, marker='o', label='ω3 Axis (on intersection line of two top surfaces)')
    # 绘制交线指示（前节右边缘）
    ax2.plot3D([intersection_line_start[0], intersection_line_end[0]], 
              [intersection_line_start[1], intersection_line_end[1]],
              [intersection_line_start[2], intersection_line_end[2]], 
              'g-', linewidth=2, alpha=0.5, linestyle='--',
              label='Intersection line of top surfaces')
    
    # 绘制上表面填充
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    front_surface = [[front_corners_3d[i] for i in range(4)]]
    front_collection = Poly3DCollection(front_surface, alpha=0.3, facecolor='lightblue')
    ax2.add_collection3d(front_collection)
    
    rear_surface = [[rear_corners_3d[i] for i in range(4)]]
    rear_collection = Poly3DCollection(rear_surface, alpha=0.3, facecolor='lightcoral')
    ax2.add_collection3d(rear_collection)
    
    ax2.set_xlabel('Length (mm)')
    ax2.set_ylabel('Width (mm)')
    ax2.set_zlabel('Height (mm)')
    ax2.set_title(f'3D Top View - {title}' if title else '3D Top View')
    ax2.legend()
    
    # 设置3D轴等比例尺（确保X、Y、Z轴单位长度相同）
    set_axes_equal_3d(ax2)
    
    # 设置合适的视角
    ax2.view_init(elev=20, azim=45)
    
    plt.tight_layout()
    return fig


def parse_motion_plan_table(filename: str) -> Optional[List[Dict]]:
    """
    解析 motion_plan_table.txt，提取每步的吸附状态与磁铁位置。

    返回:
        若解析成功则返回 List[Dict]，每项含:
        - step: int
        - m1_on: bool
        - m2_on: bool
        - m1_xyz: (x,y,z) 或 None
        - m2_xyz: (x,y,z) 或 None
        否则返回 None。
    """
    if not os.path.isfile(filename):
        return None
    pat_m1 = re.compile(r'M1:\(([-\d.]+),([-\d.]+),([-\d.]+)\)')
    pat_m2 = re.compile(r'M2:\(([-\d.]+),([-\d.]+),([-\d.]+)\)')
    rows = []
    found_header = False
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.rstrip('\n')
                if not line.strip():
                    continue
                if 'Step\tState\t' in line and 'Attached Magnet Position' in line:
                    found_header = True
                    continue
                if found_header and 'Summary' in line:
                    break
                if found_header and line.strip().startswith('---'):
                    continue
                if found_header and re.match(r'^\d+\t', line):
                    parts = line.split('\t')
                    if len(parts) < 15:
                        continue
                    try:
                        step = int(parts[0])
                        m1_on = (parts[6].strip() == '1')
                        m2_on = (parts[7].strip() == '1')
                        pos_str = parts[14] if len(parts) > 14 else ''
                    except (IndexError, ValueError):
                        continue
                    m1_xyz = None
                    m2_xyz = None
                    if 'M1:(' in pos_str:
                        m = pat_m1.search(pos_str)
                        if m:
                            m1_xyz = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
                    if 'M2:(' in pos_str:
                        m = pat_m2.search(pos_str)
                        if m:
                            m2_xyz = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
                    rows.append({
                        'step': step, 'm1_on': m1_on, 'm2_on': m2_on,
                        'm1_xyz': m1_xyz, 'm2_xyz': m2_xyz,
                    })
    except Exception:
        return None
    return rows if rows else None


def visualize_plan(planner: MotionPlanner, surface: EngineSurface, 
                   start: Tuple[float, float], goal: Tuple[float, float],
                   front_start_z: float = None, rear_start_z: float = None,
                   front_goal_z: float = None,
                   table_path: str = None):
    """
    可视化规划结果。优先根据 motion_plan_table 中的吸附磁铁位置绘制足端轨迹。

    参数:
        planner: 运动规划器
        surface: 发动机表面模型
        start: 前足起始点表面坐标 (theta, z)
        goal: 前足终点表面坐标 (theta, z)
        front_start_z: 前足起始Z坐标（可选）
        rear_start_z: 后足起始Z坐标（可选）
        front_goal_z: 前足终点Z坐标（可选）
        table_path: motion_plan_table.txt 路径（可选，默认 'motion_plan_table.txt'）

    说明:
        - 若存在 table_path 且解析成功，则按表中每步 M1/M2 吸附位置绘制前足/后足轨迹。
        - 否则回退到 path_points 或简化直线路径。
    """
    fig = plt.figure(figsize=(14, 6))
    
    ax1 = fig.add_subplot(121, projection='3d')
    surface.visualize(ax1, show_bottom_only=True)
    
    table_file = table_path or 'motion_plan_table.txt'
    rows = parse_motion_plan_table(table_file)
    use_table = (rows is not None and len(rows) > 0)
    
    if use_table:
        # 按步序收集吸附落点，只记录位置发生变化的步骤（新落点）
        front_path_xyz = []
        rear_path_xyz = []
        last_front_pos = None
        last_rear_pos = None
        tolerance = 0.1  # 位置变化容差（mm）
        
        for r in rows:
            # 前足：M1=ON 且位置发生变化时记录
            if r['m1_on'] and r['m1_xyz']:
                if last_front_pos is None:
                    # 第一个前足位置
                    front_path_xyz.append(r['m1_xyz'])
                    last_front_pos = r['m1_xyz']
                else:
                    # 检查位置是否变化
                    dx = abs(r['m1_xyz'][0] - last_front_pos[0])
                    dy = abs(r['m1_xyz'][1] - last_front_pos[1])
                    dz = abs(r['m1_xyz'][2] - last_front_pos[2])
                    if dx > tolerance or dy > tolerance or dz > tolerance:
                        front_path_xyz.append(r['m1_xyz'])
                        last_front_pos = r['m1_xyz']
            
            # 后足：M2=ON 且位置发生变化时记录
            if r['m2_on'] and r['m2_xyz']:
                if last_rear_pos is None:
                    # 第一个后足位置
                    rear_path_xyz.append(r['m2_xyz'])
                    last_rear_pos = r['m2_xyz']
                else:
                    # 检查位置是否变化
                    dx = abs(r['m2_xyz'][0] - last_rear_pos[0])
                    dy = abs(r['m2_xyz'][1] - last_rear_pos[1])
                    dz = abs(r['m2_xyz'][2] - last_rear_pos[2])
                    if dx > tolerance or dy > tolerance or dz > tolerance:
                        rear_path_xyz.append(r['m2_xyz'])
                        last_rear_pos = r['m2_xyz']
        
        # 转换为2D表面坐标
        front_foot_theta = [np.arctan2(p[1], p[0]) for p in front_path_xyz]
        front_foot_z = [p[2] for p in front_path_xyz]
        rear_foot_theta = [np.arctan2(p[1], p[0]) for p in rear_path_xyz]
        rear_foot_z = [p[2] for p in rear_path_xyz]
        
        # 转换为3D绘图坐标：(X,Y,Z) -> (Z, X, Y)
        front_path_x_3d = [p[2] for p in front_path_xyz]
        front_path_y_3d = [p[0] for p in front_path_xyz]
        front_path_z_3d = [p[1] for p in front_path_xyz]
        rear_path_x_3d = [p[2] for p in rear_path_xyz]
        rear_path_y_3d = [p[0] for p in rear_path_xyz]
        rear_path_z_3d = [p[1] for p in rear_path_xyz]
    else:
        front_start_theta = start[0]
        front_z_start = front_start_z if front_start_z is not None else start[1]
        rear_start_theta = start[0]
        rear_z_start = rear_start_z if rear_start_z is not None else max(0, front_z_start - 40)
        front_goal_theta = goal[0]
        front_z_goal = front_goal_z if front_goal_z is not None else goal[1]
        if hasattr(planner, 'path_points') and len(planner.path_points) > 0:
            front_foot_theta = []
            front_foot_z = []
            for theta, z in planner.path_points:
                front_foot_theta.append(theta)
                front_foot_z.append(z)
            rear_foot_theta = []
            rear_foot_z = []
            fd = front_z_start - rear_z_start
            for theta, z in planner.path_points:
                rear_foot_theta.append(theta)
                rear_foot_z.append(max(0, z - fd))
        else:
            front_foot_theta = [front_start_theta, front_goal_theta]
            front_foot_z = [front_z_start, front_z_goal]
            rear_foot_theta = [rear_start_theta, front_goal_theta]
            rear_foot_z = [rear_z_start, max(0, front_z_goal - 40)]
        front_path_x_3d = []
        front_path_y_3d = []
        front_path_z_3d = []
        for theta, z_orig in zip(front_foot_theta, front_foot_z):
            r = surface.get_radius(z_orig)
            xo, yo = r * np.cos(theta), r * np.sin(theta)
            front_path_x_3d.append(z_orig)
            front_path_y_3d.append(xo)
            front_path_z_3d.append(yo)
        rear_path_x_3d = []
        rear_path_y_3d = []
        rear_path_z_3d = []
        for theta, z_orig in zip(rear_foot_theta, rear_foot_z):
            r = surface.get_radius(z_orig)
            xo, yo = r * np.cos(theta), r * np.sin(theta)
            rear_path_x_3d.append(z_orig)
            rear_path_y_3d.append(xo)
            rear_path_z_3d.append(yo)
    
    # 绘制轨迹
    if len(front_path_x_3d) > 1:
        ax1.plot(front_path_x_3d, front_path_y_3d, front_path_z_3d,
                 'b-', linewidth=2, label='Front Foot Path', alpha=0.8)
    if len(rear_path_x_3d) > 1:
        ax1.plot(rear_path_x_3d, rear_path_y_3d, rear_path_z_3d,
                 'orange', linewidth=2, label='Rear Foot Path', alpha=0.8)
    
    # 固定起点和终点标记（用户要求的位置）
    # 前足起点: (X=0, Y=-900, Z=40) -> 3D(40, 0, -900)
    ax1.scatter([40], [0], [-900], 
               c='green', s=100, marker='o', 
               label='Start (Front) X=0,Y=-900,Z=40', edgecolors='black')
    
    # 前足终点: (X=0, Y=-900, Z=1000) -> 3D(1000, 0, -900)
    ax1.scatter([1000], [0], [-900], 
               c='red', s=150, marker='*', 
               label='Goal (Front) X=0,Y=-900,Z=1000', edgecolors='black')
    
    # 后足起点: (X=0, Y=-900, Z=0) -> 3D(0, 0, -900)
    ax1.scatter([0], [0], [-900], 
               c='lime', s=80, marker='^', 
               label='Start (Rear) X=0,Y=-900,Z=0', edgecolors='black')
    
    # 后足终点: (X=0, Y=-900, Z=960) -> 3D(960, 0, -900)
    ax1.scatter([960], [0], [-900], 
               c='darkorange', s=80, marker='D', 
               label='End (Rear) X=0,Y=-900,Z=960', edgecolors='black')
    
    ax1.set_xlabel('Axial (Z)')
    ax1.set_ylabel('Radial X')
    ax1.set_zlabel('Radial Y')
    ax1.set_title('3D Path Visualization - Inner Surface (Horizontal)')
    
    # 设置3D轴等比例尺（确保X、Y、Z轴单位长度相同）
    set_axes_equal_3d(ax1)
    
    ax1.legend(loc='upper left', fontsize=8)
    
    # 2D表面坐标可视化
    ax2 = fig.add_subplot(122)
    z_vals = np.linspace(surface.z_min, surface.z_max, 100)
    theta_vals = np.linspace(0, 2 * np.pi, 100)
    Z, Theta = np.meshgrid(z_vals, theta_vals)
    
    # 绘制障碍物区域
    valid = np.ones_like(Z, dtype=bool)
    for blade in surface.blades:
        z_mask = np.abs(Z - blade['z']) < 0.5
        theta_mask = np.zeros_like(Theta, dtype=bool)
        for i, theta in enumerate(Theta.flat):
            theta_diff = abs(theta - blade['theta'])
            theta_diff = min(theta_diff, 2 * np.pi - theta_diff)
            if theta_diff < blade['width'] / 2:
                theta_mask.flat[i] = True
        valid[z_mask & theta_mask] = False
    
    ax2.contourf(Theta, Z, valid.astype(float), levels=[0, 0.5, 1], 
                 colors=['red', 'white'], alpha=0.3)
    
    # 绘制前足轨迹（2D图）
    if len(front_foot_theta) > 0 and len(front_foot_z) > 0:
        ax2.plot(front_foot_theta, front_foot_z, 'b-', linewidth=2, 
                markersize=4, label='Front Foot Path', alpha=0.7)
    
    # 绘制后足轨迹（2D图）
    if len(rear_foot_theta) > 0 and len(rear_foot_z) > 0:
        ax2.plot(rear_foot_theta, rear_foot_z, 'orange', linewidth=2, 
                markersize=4, label='Rear Foot Path', alpha=0.7)
    
    # 绘制起点和终点标记
    if len(front_foot_theta) > 0:
        ax2.plot(front_foot_theta[0], front_foot_z[0], 'go', markersize=10, label='Start (Front)')
        ax2.plot(front_foot_theta[-1], front_foot_z[-1], 'ro', markersize=10, label='Goal (Front)')
    
    if len(rear_foot_theta) > 0:
        ax2.plot(rear_foot_theta[0], rear_foot_z[0], 'g^', markersize=10, label='Start (Rear)')
        ax2.plot(rear_foot_theta[-1], rear_foot_z[-1], 'r^', markersize=10, label='Goal (Rear)')
    ax2.set_xlabel('Angle theta (rad)')
    ax2.set_ylabel('Axial Position Z (mm)')
    ax2.set_title('Surface Coordinate Path (Red = Blade Obstacles)')
    ax2.legend(loc='upper left', fontsize=8)
    ax2.grid(True)
    
    plt.tight_layout()
    return fig


if __name__ == "__main__":
    # 创建表面模型（恒定半径900mm）
    surface = EngineSurface(z_min=0, z_max=100, r_constant=900.0)
    
    # 创建机器人（新参数）
    robot = Robot(L1=40, W1=30, L2=40, W2=30, h=30, s_max=8, 
                  theta3_min=-90, theta3_max=90)
    
    # 创建规划器
    planner = MotionPlanner(robot, surface)
    
    # 设置起点和终点
    start = (0.5, 20.0)  # (theta, z)
    goal = (3.0, 80.0)   # (theta, z)
    
    # 执行规划
    print("开始运动规划...")
    plan = planner.plan_path(start, goal, step_size=2.0)
    
    print(f"\n规划完成！共 {len(plan)} 个步骤")
    print("\n运动序列:")
    total_time = 0
    for i, (state, duration) in enumerate(plan, 1):
        total_time += duration
        print(f"步骤 {i}: 状态={state}, 持续时间={duration:.2f}s")
    
    print(f"\n总时间: {total_time:.2f}s")
    
    # 保存规划结果
    planner.save_plan('motion_plan.json')
    print("\n规划结果已保存到 motion_plan.json")
    
    planner.save_plan_table('motion_plan_table.txt')
    print("表格格式已保存到 motion_plan_table.txt")
    
    # 可视化机器人结构
    print("\n生成机器人结构图...")
    fig_robot = visualize_robot_structure(robot)
    plt.savefig('robot_structure.png', dpi=150, bbox_inches='tight')
    print("机器人结构图已保存到 robot_structure.png")
    plt.close(fig_robot)
    
    # 生成特定状态的机器人图像
    print("\n生成特定状态的机器人图像...")
    
    # 状态1：θ3 = +45°, s = 0
    theta3_1 = np.radians(45)
    s_1 = 0.0
    fig_state1 = visualize_robot_state(robot, theta3_1, s_1, 
                                      f"θ3 = +45°, s = {s_1:.1f}mm")
    plt.savefig('robot_state_theta3_45_s_0.png', dpi=150, bbox_inches='tight')
    print("状态1图像已保存到 robot_state_theta3_45_s_0.png")
    plt.close(fig_state1)
    
    # 状态2：θ3 = -45°, s = s_max
    theta3_2 = np.radians(-45)
    s_2 = robot.s_max
    fig_state2 = visualize_robot_state(robot, theta3_2, s_2,
                                      f"θ3 = -45°, s = {s_2:.1f}mm")
    plt.savefig('robot_state_theta3_-45_s_max.png', dpi=150, bbox_inches='tight')
    print("状态2图像已保存到 robot_state_theta3_-45_s_max.png")
    plt.close(fig_state2)
