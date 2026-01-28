import numpy as np


def solve_quadratic(a, b, c):
    """
    解二次方程 ax^2 + bx + c = 0
    返回实根
    """
    if abs(a) < 1e-12:  # 如果a接近0，方程退化为一次方程
        if abs(b) < 1e-12:
            return []
        return [-c / b]
    
    discriminant = b**2 - 4*a*c
    if discriminant < 0:
        return []
    elif discriminant == 0:
        return [-b / (2*a)]
    else:
        sqrt_disc = np.sqrt(discriminant)
        return [(-b + sqrt_disc) / (2*a), (-b - sqrt_disc) / (2*a)]


def ellipse_circle_intersection_analytical(cx, cy, r_circle, a_ellipse, b_ellipse):
    """
    解析方法求椭圆与圆的交点
    椭圆: x^2/a^2 + y^2/b^2 = 1
    圆: (x-cx)^2 + (y-cy)^2 = r^2
    """
    # 从圆方程得：(x-cx)^2 = r^2 - (y-cy)^2
    # 所以 x = cx ± sqrt(r^2 - (y-cy)^2)
    # 从椭圆方程得：x^2 = a^2 * (1 - y^2/b^2)
    # 代入得：[cx ± sqrt(r^2 - (y-cy)^2)]^2 = a^2 * (1 - y^2/b^2)
    
    # 展开左边：cx^2 ± 2*cx*sqrt(r^2-(y-cy)^2) + (r^2-(y-cy)^2) = a^2 - a^2*y^2/b^2
    # 移项：± 2*cx*sqrt(r^2-(y-cy)^2) = a^2 - a^2*y^2/b^2 - cx^2 - (r^2-(y-cy)^2)
    # 令 RHS = a^2 - a^2*y^2/b^2 - cx^2 - r^2 + (y-cy)^2
    # 两边平方：4*cx^2*(r^2-(y-cy)^2) = RHS^2
    
    # 展开 (y-cy)^2 = y^2 - 2*cy*y + cy^2
    # RHS = a^2 - a^2*y^2/b^2 - cx^2 - r^2 + y^2 - 2*cy*y + cy^2
    # RHS = (a^2 - cx^2 - r^2 + cy^2) + y^2*(1 - a^2/b^2) - 2*cy*y
    # RHS = C1 + C2*y^2 + C3*y，其中：
    C1 = a_ellipse**2 - cx**2 - r_circle**2 + cy**2
    C2 = 1 - a_ellipse**2 / b_ellipse**2
    C3 = -2*cy
    
    # LHS = 4*cx^2*(r^2 - (y^2 - 2*cy*y + cy^2))
    # LHS = 4*cx^2*(r^2 - y^2 + 2*cy*y - cy^2)
    # LHS = 4*cx^2*(r^2 - cy^2) + 4*cx^2*(2*cy*y - y^2)
    # LHS = 4*cx^2*(r^2 - cy^2) + 8*cx^2*cy*y - 4*cx^2*y^2
    
    # RHS^2 = (C1 + C2*y^2 + C3*y)^2
    # RHS^2 = C1^2 + C2^2*y^4 + C3^2*y^2 + 2*C1*C2*y^2 + 2*C1*C3*y + 2*C2*C3*y^3
    
    # LHS = 4*cx^2*(r^2 - cy^2) + 8*cx^2*cy*y - 4*cx^2*y^2
    
    # 方程为: LHS = RHS^2
    # 即: 0 = RHS^2 - LHS
    # 0 = C1^2 + C2^2*y^4 + C3^2*y^2 + 2*C1*C2*y^2 + 2*C1*C3*y + 2*C2*C3*y^3
    #      - 4*cx^2*(r^2 - cy^2) - 8*cx^2*cy*y + 4*cx^2*y^2
    
    # 整理系数:
    # y^4: C2^2
    # y^3: 2*C2*C3
    # y^2: C3^2 + 2*C1*C2 + 4*cx^2
    # y^1: 2*C1*C3 - 8*cx^2*cy
    # y^0: C1^2 - 4*cx^2*(r^2 - cy^2)
    
    coeff_4 = C2**2
    coeff_3 = 2*C2*C3
    coeff_2 = C3**2 + 2*C1*C2 + 4*cx**2
    coeff_1 = 2*C1*C3 - 8*cx**2*cy
    coeff_0 = C1**2 - 4*cx**2*(r_circle**2 - cy**2)
    
    # 这是一个四次方程，使用numpy求解
    coeffs = [coeff_4, coeff_3, coeff_2, coeff_1, coeff_0]
    
    # 使用numpy求解四次方程
    roots = np.roots(coeffs)
    
    # 筛选实根
    real_roots = []
    for root in roots:
        if np.isreal(root):
            real_roots.append(np.real(root))
    
    # 验证解的有效性并找到最准确的解
    valid_solutions = []
    
    for y_val in real_roots:
        # 检查是否满足圆方程的约束条件 (r^2 - (y-cy)^2 >= 0)
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
                circle_error = abs((x_candidate - cx)**2 + (y_val - cy)**2 - r_circle**2)
                total_error = ellipse_error + circle_error
                valid_solutions.append((x_candidate, y_val, total_error))
    
    if not valid_solutions:
        # 如果没有找到精确解，返回一个近似的解
        # 使用原始的迭代方法作为备选
        return ellipse_circle_intersection_iterative(cx, cy, r_circle, a_ellipse, b_ellipse, n_points=720)
    
    # 从有效解中选择误差最小的
    best_solution = min(valid_solutions, key=lambda x: x[2])
    return best_solution[0], best_solution[1], best_solution[2]


def ellipse_circle_intersection_iterative(cx, cy, r_circle, a_ellipse, b_ellipse, n_points=360):
    """
    当前使用的迭代方法
    """
    best_x, best_y = 0.0, -b_ellipse
    min_error = float('inf')
    
    for angle in np.linspace(0, 2 * np.pi, n_points):
        x = cx + r_circle * np.cos(angle)
        y = cy + r_circle * np.sin(angle)
        if not (y < 0 and y > -b_ellipse - 10):
            continue
        err = abs((x**2 / a_ellipse**2) + (y**2 / b_ellipse**2) - 1)
        if err < min_error:
            min_error = err
            best_x, best_y = x, y
    
    return best_x, best_y, min_error


def test_analytical_method():
    """
    测试解析方法
    """
    print("解析方法测试")
    print("="*60)
    
    # 测试参数
    test_cases = [
        # (cx, cy, r_circle, a_ellipse, b_ellipse)
        (-20, -870, 50, 900, 900),      # φ=45°情况
        (0, -900, 40, 1272.8, 900),     # φ=30°情况  
        (-22, -870, 50, 636.4, 900),    # φ=60°情况
        (-20, -870, 50, 90000, 900),    # φ≈0°近似纯轴向
    ]
    
    for i, (cx, cy, r_circle, a_ellipse, b_ellipse) in enumerate(test_cases):
        print(f"\n测试案例 {i+1}: cx={cx}, cy={cy}, r_circle={r_circle}, a_ellipse={a_ellipse:.1f}, b_ellipse={b_ellipse}")
        
        # 测试迭代方法
        x_iter, y_iter, err_iter = ellipse_circle_intersection_iterative(cx, cy, r_circle, a_ellipse, b_ellipse)
        
        # 测试解析方法
        x_ana, y_ana, err_ana = ellipse_circle_intersection_analytical(cx, cy, r_circle, a_ellipse, b_ellipse)
        
        print(f"迭代方法: x={x_iter:.6f}, y={y_iter:.6f}, 误差={err_iter:.2e}")
        print(f"解析方法: x={x_ana:.6f}, y={y_ana:.6f}, 误差={err_ana:.2e}")
        
        # 验证解的有效性
        # 检查迭代解
        eq1_iter = (x_iter**2 / a_ellipse**2) + (y_iter**2 / b_ellipse**2)
        eq2_iter = ((x_iter - cx)**2 + (y_iter - cy)**2)
        print(f"迭代解验证: 椭圆方程残差={abs(eq1_iter-1):.2e}, 圆方程残差={abs(eq2_iter-r_circle**2):.2e}")
        
        # 检查解析解
        eq1_ana = (x_ana**2 / a_ellipse**2) + (y_ana**2 / b_ellipse**2)
        eq2_ana = ((x_ana - cx)**2 + (y_ana - cy)**2)
        print(f"解析解验证: 椭圆方程残差={abs(eq1_ana-1):.2e}, 圆方程残差={abs(eq2_ana-r_circle**2):.2e}")
        
        print(f"解的差异: dx={abs(x_iter-x_ana):.6f}, dy={abs(y_iter-y_ana):.6f}")


if __name__ == "__main__":
    test_analytical_method()