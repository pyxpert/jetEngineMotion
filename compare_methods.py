import numpy as np
from scipy.optimize import fsolve
import time


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


def ellipse_circle_intersection_analytical(cx, cy, r_circle, a_ellipse, b_ellipse):
    """
    解析方法：使用数值求解器求解椭圆与圆的交点
    """
    # 定义方程组：F([x,y]) = [椭圆方程, 圆方程]
    def equations(vars):
        x, y = vars
        eq1 = (x**2 / a_ellipse**2) + (y**2 / b_ellipse**2) - 1  # 椭圆方程
        eq2 = (x - cx)**2 + (y - cy)**2 - r_circle**2             # 圆方程
        return [eq1, eq2]
    
    # 使用不同的初值尝试找到不同的解
    initial_guesses = [
        [cx + r_circle, cy],      # 圆右侧
        [cx - r_circle, cy],      # 圆左侧
        [cx, cy + r_circle],      # 圆上方
        [cx, cy - r_circle],      # 圆下方
        [0, b_ellipse],           # 椭圆上顶点附近
        [a_ellipse, 0],           # 椭圆右顶点附近
    ]
    
    solutions = []
    for guess in initial_guesses:
        try:
            sol = fsolve(equations, guess, xtol=1e-12)
            x, y = sol
            
            # 验证解的准确性
            eq1_val = (x**2 / a_ellipse**2) + (y**2 / b_ellipse**2) - 1
            eq2_val = (x - cx)**2 + (y - cy)**2 - r_circle**2
            
            if abs(eq1_val) < 1e-6 and abs(eq2_val) < 1e-6:  # 解是有效的
                # 检查是否已经找到过这个解（避免重复）
                is_duplicate = False
                for existing_x, existing_y in solutions:
                    if abs(existing_x - x) < 1e-6 and abs(existing_y - y) < 1e-6:
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    solutions.append((x, y))
        except:
            continue  # 如果求解失败，跳过这个初值
    
    if not solutions:
        # 如果没找到解，返回一个近似解
        return cx, cy, float('inf')
    
    # 从所有解中选择一个最符合要求的（例如，y<0的部分）
    best_solution = None
    best_error = float('inf')
    
    for x, y in solutions:
        if y < 0:  # 选择y<0的解（底部）
            # 计算总的方程误差
            eq1_err = abs((x**2 / a_ellipse**2) + (y**2 / b_ellipse**2) - 1)
            eq2_err = abs((x - cx)**2 + (y - cy)**2 - r_circle**2)
            total_error = eq1_err + eq2_err
            
            if total_error < best_error:
                best_error = total_error
                best_solution = (x, y)
    
    if best_solution is None:
        # 如果没有y<0的解，返回误差最小的解
        best_solution = solutions[0]
        x, y = best_solution
        eq1_err = abs((x**2 / a_ellipse**2) + (y**2 / b_ellipse**2) - 1)
        eq2_err = abs((x - cx)**2 + (y - cy)**2 - r_circle**2)
        best_error = eq1_err + eq2_err
    
    return best_solution[0], best_solution[1], best_error


def test_comparison():
    """
    测试两种方法的精度对比
    """
    print("椭圆与圆求交点方法精度对比测试")
    print("="*60)
    
    # 测试参数
    test_cases = [
        # (cx, cy, r_circle, a_ellipse, b_ellipse)
        (-20, -870, 50, 900, 900),      # φ=45°情况
        (0, -900, 40, 1272.8, 900),     # φ=30°情况  
        (-22, -870, 50, 636.4, 900),    # φ=60°情况
        (-20, -870, 50, 900/0.01, 900), # φ≈0°近似纯轴向
    ]
    
    for i, (cx, cy, r_circle, a_ellipse, b_ellipse) in enumerate(test_cases):
        print(f"\n测试案例 {i+1}: cx={cx}, cy={cy}, r_circle={r_circle}, a_ellipse={a_ellipse:.1f}, b_ellipse={b_ellipse}")
        
        # 测试迭代方法
        start_time = time.time()
        x_iter, y_iter, err_iter = ellipse_circle_intersection_iterative(cx, cy, r_circle, a_ellipse, b_ellipse)
        iter_time = time.time() - start_time
        
        # 测试解析方法
        start_time = time.time()
        x_ana, y_ana, err_ana = ellipse_circle_intersection_analytical(cx, cy, r_circle, a_ellipse, b_ellipse)
        ana_time = time.time() - start_time
        
        print(f"迭代方法: x={x_iter:.6f}, y={y_iter:.6f}, 误差={err_iter:.2e}, 时间={iter_time*1000:.2f}ms")
        print(f"解析方法: x={x_ana:.6f}, y={y_ana:.6f}, 误差={err_ana:.2e}, 时间={ana_time*1000:.2f}ms")
        
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
    test_comparison()