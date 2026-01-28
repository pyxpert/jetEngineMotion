import numpy as np
import matplotlib.pyplot as plt

def plot_robot_geometry_model_eng():
    """
    Draw the geometric simplified model of the robot and the geometric relationship of the ellipse/small circle intersection algorithm
    """
    # Robot parameters
    d1 = 20  # Horizontal distance from front foot to middle motor
    d2 = 20  # Horizontal distance from rear foot to middle motor
    h = 30   # Height from center motor shaft to top surface
    a = 50   # Linear motor stroke (example value)
    R = 900  # Cylinder radius
    
    # Create figure
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    
    # Subplot 1: Robot geometric simplified model
    ax1 = axes[0, 0]
    ax1.set_title('Robot Geometric Simplified Model')
    
    # Draw cylinder wall (for illustration)
    ax1.add_patch(plt.Rectangle((-100, -R-20), 200, 20, fill=True, color='lightgray', alpha=0.5, label='Cylinder Wall'))
    
    # Draw robot structure
    # Middle motor position
    motor_x, motor_y = 0, -R + h
    ax1.plot(motor_x, motor_y, 'bo', markersize=8, label='Middle Motor')
    
    # Front foot position (when a=0)
    front_foot_x = motor_x - d1
    front_foot_y = motor_y
    ax1.plot(front_foot_x, front_foot_y, 'ro', markersize=8, label='Front Foot')
    
    # Rear foot position (when a=0)
    rear_foot_x = motor_x + d2
    rear_foot_y = motor_y
    ax1.plot(rear_foot_x, rear_foot_y, 'go', markersize=8, label='Rear Foot')
    
    # Rear foot position when linear motor stroke is a
    rear_foot_extended_x = motor_x + d2 + a
    ax1.plot(rear_foot_extended_x, rear_foot_y, 'mo', markersize=8, label='Rear Foot Extended')
    
    # Connection lines
    ax1.plot([front_foot_x, motor_x], [front_foot_y, motor_y], 'r-', linewidth=2, label='d1 connection')
    ax1.plot([rear_foot_x, motor_x], [rear_foot_y, motor_y], 'g-', linewidth=2, label='d2 connection')
    ax1.plot([rear_foot_x, rear_foot_extended_x], [rear_foot_y, rear_foot_y], 'm--', linewidth=2, label='Stroke a')
    
    # Height annotation
    ax1.annotate(f'h={h}', xy=(motor_x+5, motor_y), xytext=(motor_x+5, -R), 
                arrowprops=dict(arrowstyle='<->', color='blue'), fontsize=10, color='blue')
    
    ax1.set_xlim(-100, 100)
    ax1.set_ylim(-R-50, -R+h+20)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper left', fontsize=8)
    ax1.invert_yaxis()  # Make y-axis positive direction downward
    
    # Subplot 2: Ellipse equation diagram
    ax2 = axes[0, 1]
    ax2.set_title('Ellipse Constraint Equation')
    
    # Ellipse parameters (with φ=45° as example)
    phi = np.pi/4  # 45 degrees
    a_ellipse = abs(R / np.sin(phi)) if abs(np.sin(phi)) >= 0.01 else R/0.01
    b_ellipse = R
    
    # Draw ellipse
    theta_ellipse = np.linspace(0, 2*np.pi, 1000)
    x_ellipse = a_ellipse * np.cos(theta_ellipse)
    y_ellipse = b_ellipse * np.sin(theta_ellipse)
    
    ax2.plot(x_ellipse, y_ellipse, 'purple', linewidth=2, label=f'Ellipse: y²/{R}² + x²/{a_ellipse:.0f}² = 1')
    
    # Annotate ellipse parameters
    ax2.annotate(f'a_ellipse={a_ellipse:.1f}', xy=(a_ellipse, 0), xytext=(a_ellipse*0.7, 20),
                arrowprops=dict(arrowstyle='->', color='purple'), fontsize=10, color='purple')
    ax2.annotate(f'b_ellipse={b_ellipse}', xy=(0, b_ellipse), xytext=(20, b_ellipse*0.7),
                arrowprops=dict(arrowstyle='->', color='purple'), fontsize=10, color='purple')
    
    ax2.set_xlim(-a_ellipse*1.1, a_ellipse*1.1)
    ax2.set_ylim(-b_ellipse*1.1, b_ellipse*1.1)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_xlabel('X Axis (Axial Direction)')
    ax2.set_ylabel('Y Axis (Radial Direction)')
    
    # Subplot 3: Small circle radius calculation (Rear fixed, find front)
    ax3 = axes[1, 0]
    ax3.set_title('Small Circle Radius Calculation - Rear Fixed, Find Front')
    
    # Fixed rear foot position
    rear_fixed_x, rear_fixed_y = 0, -R
    motor_pos_x, motor_pos_y = rear_fixed_x + d2, rear_fixed_y + h
    
    # Small circle center and radius
    circle_center_x, circle_center_y = motor_pos_x, motor_pos_y
    circle_radius = np.sqrt(d1**2 + h**2)
    
    # Draw geometric elements
    ax3.plot(rear_fixed_x, rear_fixed_y, 'go', markersize=8, label='Fixed Rear Foot')
    ax3.plot(circle_center_x, circle_center_y, 'bo', markersize=8, label='Motor Center')
    
    # Draw small circle
    circle = plt.Circle((circle_center_x, circle_center_y), circle_radius, 
                       fill=False, color='orange', linewidth=2, label=f'Small Circle (radius={circle_radius:.1f})')
    ax3.add_patch(circle)
    
    # Draw distance lines
    ax3.plot([circle_center_x, circle_center_x-d1], [circle_center_y, circle_center_y], 
             'k-', linewidth=1, alpha=0.7)
    ax3.plot([circle_center_x, circle_center_x], [circle_center_y, circle_center_y-h], 
             'k-', linewidth=1, alpha=0.7)
    ax3.plot([circle_center_x-d1, circle_center_x], [circle_center_y-h, circle_center_y], 
             'r-', linewidth=2, alpha=0.8, label=f'Real distance={circle_radius:.1f}')
    
    # Annotations
    ax3.annotate(f'd1={d1}', xy=(circle_center_x, circle_center_y), 
                xytext=(circle_center_x-d1/2, circle_center_y+5), fontsize=9)
    ax3.annotate(f'h={h}', xy=(circle_center_x, circle_center_y), 
                xytext=(circle_center_x+5, circle_center_y-h/2), fontsize=9)
    
    ax3.set_xlim(circle_center_x - circle_radius - 20, circle_center_x + circle_radius + 20)
    ax3.set_ylim(circle_center_y - circle_radius - 20, circle_center_y + circle_radius + 20)
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=8)
    
    # Subplot 4: Small circle radius calculation (Front fixed, find rear)
    ax4 = axes[1, 1]
    ax4.set_title('Small Circle Radius Calculation - Front Fixed, Find Rear')
    
    # Fixed front foot position
    front_fixed_x, front_fixed_y = 0, -R
    motor_pos_x2, motor_pos_y2 = front_fixed_x - d1, front_fixed_y + h
    
    # Small circle center and radius
    circle_center_x2, circle_center_y2 = motor_pos_x2, motor_pos_y2
    circle_radius2 = np.sqrt((a + d2)**2 + h**2)
    
    # Draw geometric elements
    ax4.plot(front_fixed_x, front_fixed_y, 'ro', markersize=8, label='Fixed Front Foot')
    ax4.plot(circle_center_x2, circle_center_y2, 'bo', markersize=8, label='Motor Center')
    
    # Draw small circle
    circle2 = plt.Circle((circle_center_x2, circle_center_y2), circle_radius2, 
                        fill=False, color='orange', linewidth=2, label=f'Small Circle (radius={circle_radius2:.1f})')
    ax4.add_patch(circle2)
    
    # Draw distance lines
    ax4.plot([circle_center_x2, circle_center_x2+(a+d2)], [circle_center_y2, circle_center_y2], 
             'k-', linewidth=1, alpha=0.7)
    ax4.plot([circle_center_x2, circle_center_x2], [circle_center_y2, circle_center_y2-h], 
             'k-', linewidth=1, alpha=0.7)
    ax4.plot([circle_center_x2+(a+d2), circle_center_x2], [circle_center_y2-h, circle_center_y2], 
             'r-', linewidth=2, alpha=0.8, label=f'Real distance={circle_radius2:.1f}')
    
    # Annotations
    ax4.annotate(f'a+d2={a+d2}', xy=(circle_center_x2, circle_center_y2), 
                 xytext=(circle_center_x2+(a+d2)/2, circle_center_y2+5), fontsize=9)
    ax4.annotate(f'h={h}', xy=(circle_center_x2, circle_center_y2), 
                 xytext=(circle_center_x2-15, circle_center_y2-h/2), fontsize=9)
    
    ax4.set_xlim(circle_center_x2 - circle_radius2 - 20, circle_center_x2 + circle_radius2 + 20)
    ax4.set_ylim(circle_center_y2 - circle_radius2 - 20, circle_center_y2 + circle_radius2 + 20)
    ax4.set_aspect('equal')
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=8)
    
    plt.tight_layout()
    plt.savefig('robot_geometry_detailed_eng.png', dpi=150, bbox_inches='tight')
    plt.show()

def explain_physical_quantities_eng():
    """
    Explain the meaning of physical quantities in detail
    """
    print("="*80)
    print("Detailed Explanation of Physical Quantities in Ellipse-Circle Intersection Algorithm")
    print("="*80)
    print()
    print("[Robot Structural Parameters]")
    print("d1: Horizontal distance from front foot to middle motor shaft (when a=0)")
    print("d2: Horizontal distance from rear foot to middle motor shaft (when a=0)")
    print("h: Height from center motor shaft to top surface")
    print("a: Linear motor stroke, range [0, s_max], controls distance change between feet")
    print("φ: Robot forward direction angle (angle between robot plane and environment axis)")
    print("R: Inner surface radius of cylindrical environment")
    print()
    print("[Ellipse Equation Parameters]")
    print("Ellipse equation: y²/R² + x²/(R/sin(φ))² = 1")
    print("a_ellipse = |R/sin(φ)|: Semi-axis length of ellipse in x direction")
    print("b_ellipse = R: Semi-axis length of ellipse in y direction")
    print("Ellipse represents the geometric constraints of robot movement, φ angle determines ellipse shape")
    print()
    print("[Small Circle Equation Parameters]")
    print("cx, cy: Center coordinates of small circle")
    print("r_circle: Radius of small circle, varies by situation:")
    print("  - When finding front foot (rear fixed): r_circle = √(d1² + h²)")
    print("  - When finding rear foot (front fixed): r_circle = √((a+d2)² + h²)")
    print("Small circle represents the possible position range of active foot relative to fixed foot")
    print()
    print("[Geometric Significance]")
    print("The ellipse-circle intersection algorithm essentially finds foot positions satisfying kinematic constraints:")
    print("1. Ellipse constraint: Geometric limitation of overall robot movement")
    print("2. Small circle constraint: Reachable range of active foot relative to fixed foot")
    print("3. Intersection: Position satisfying both constraints, i.e., feasible foot landing position")
    print("="*80)

if __name__ == "__main__":
    plot_robot_geometry_model_eng()
    explain_physical_quantities_eng()