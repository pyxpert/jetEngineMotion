import numpy as np
import matplotlib.pyplot as plt

def visualize_geometric_relationship():
    """
    Visualize geometric relationship, verify pyxpert's correction suggestion
    """
    # Robot parameters (from requirements_log.txt)
    d1 = 20  # Distance from front foot to middle motor (horizontal)
    d2 = 20  # Distance from rear foot to middle motor (horizontal)
    h = 30   # Height from center motor shaft to top surface
    a = 50   # Linear motor stroke (example value)
    R = 900  # Cylinder radius
    
    # Create plots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    
    # Case 1: Rear foot adsorbed, front foot will drop (find_front=True)
    ax1.set_title('Case 1: Rear Foot Fixed, Find Front Foot')
    
    # Fixed rear foot position (assuming at bottom)
    rear_foot_x = 0
    rear_foot_y = -R
    
    # Middle motor position (relative to rear foot)
    motor_x = rear_foot_x + d2
    motor_y = rear_foot_y + h
    
    # Possible front foot positions (circle centered at motor with radius sqrt(d1^2+h^2))
    current_radius = np.sqrt(d1**2 + h**2)
    
    # Draw geometric relationships
    ax1.plot(rear_foot_x, rear_foot_y, 'ro', markersize=8, label='Rear Foot (Fixed)')
    ax1.plot(motor_x, motor_y, 'bo', markersize=8, label='Middle Motor')
    
    # Draw current small circle (incorrect method)
    current_circle = plt.Circle((motor_x, motor_y), d1 + h, fill=False, 
                               color='red', linestyle='--', label=f'Current Circle (radius={d1+h})')
    ax1.add_patch(current_circle)
    
    # Draw corrected small circle (correct method)
    corrected_circle = plt.Circle((motor_x, motor_y), current_radius, fill=False, 
                                 color='green', linestyle='-', label=f'Corrected Circle (radius=sqrt({d1}^2+{h}^2)={current_radius:.1f})')
    ax1.add_patch(corrected_circle)
    
    # Draw distance lines
    ax1.plot([motor_x, motor_x + d1], [motor_y, motor_y], 'k-', alpha=0.5, label=f'Horizontal distance d1={d1}')
    ax1.plot([motor_x, motor_x], [motor_y, motor_y - h], 'k--', alpha=0.5, label=f'Vertical distance h={h}')
    
    ax1.set_xlim(motor_x - 100, motor_x + 100)
    ax1.set_ylim(motor_y - 100, motor_y + 50)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Case 2: Front foot adsorbed, rear foot will drop (find_front=False)
    ax2.set_title('Case 2: Front Foot Fixed, Find Rear Foot')
    
    # Fixed front foot position (assuming at bottom)
    front_foot_x = 0
    front_foot_y = -R
    
    # Middle motor position (relative to front foot)
    motor_x2 = front_foot_x - d1
    motor_y2 = front_foot_y + h
    
    # Possible rear foot positions (circle centered at motor with radius sqrt((a+d2)^2+h^2))
    corrected_radius2 = np.sqrt((a + d2)**2 + h**2)
    
    # Draw geometric relationships
    ax2.plot(front_foot_x, front_foot_y, 'go', markersize=8, label='Front Foot (Fixed)')
    ax2.plot(motor_x2, motor_y2, 'bo', markersize=8, label='Middle Motor')
    
    # Draw current small circle (incorrect method)
    current_circle2 = plt.Circle((motor_x2, motor_y2), a + d2 + h, fill=False, 
                                color='red', linestyle='--', label=f'Current Circle (radius={a+d2+h})')
    ax2.add_patch(current_circle2)
    
    # Draw corrected small circle (correct method)
    corrected_circle2 = plt.Circle((motor_x2, motor_y2), corrected_radius2, fill=False, 
                                  color='green', linestyle='-', label=f'Corrected Circle (radius=sqrt({a+d2}^2+{h}^2)={corrected_radius2:.1f})')
    ax2.add_patch(corrected_circle2)
    
    # Draw distance lines
    ax2.plot([motor_x2, motor_x2 + a + d2], [motor_y2, motor_y2], 'k-', alpha=0.5, label=f'Horizontal distance a+d2={a+d2}')
    ax2.plot([motor_x2, motor_x2], [motor_y2, motor_y2 - h], 'k--', alpha=0.5, label=f'Vertical distance h={h}')
    
    ax2.set_xlim(motor_x2 - 150, motor_x2 + 150)
    ax2.set_ylim(motor_y2 - 100, motor_y2 + 50)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig('geometric_correction_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    # Print analysis results
    print("Geometric Relationship Analysis Results:")
    print("="*50)
    print(f"Case 1 (Rear foot fixed, find front foot):")
    print(f"  Current method: radius = {d1 + h:.1f}")
    print(f"  Corrected method: radius = sqrt({d1}^2+{h}^2) = {current_radius:.1f}")
    print(f"  Difference: {abs(current_radius - (d1 + h)):.1f}")
    print()
    print(f"Case 2 (Front foot fixed, find rear foot):")
    print(f"  Current method: radius = {a + d2 + h:.1f}")
    print(f"  Corrected method: radius = sqrt({a+d2}^2+{h}^2) = {corrected_radius2:.1f}")
    print(f"  Difference: {abs(corrected_radius2 - (a + d2 + h)):.1f}")

if __name__ == "__main__":
    visualize_geometric_relationship()