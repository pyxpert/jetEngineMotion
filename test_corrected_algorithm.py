import sys
sys.path.append('.')

from robot_motion_planner import Robot, EngineSurface
import numpy as np

def test_corrected_algorithm():
    """
    Test the corrected algorithm with geometrically accurate circle radius
    """
    print("Testing Corrected Algorithm with Geometrically Accurate Circle Radius")
    print("="*70)
    
    # Create robot and surface instances
    robot = Robot()
    surface = EngineSurface()
    
    # Test parameters
    test_cases = [
        {
            'description': 'Case 1: Rear foot fixed, find front foot',
            'fixed_foot_X': 0.0,
            'fixed_foot_theta': -np.pi/2,
            'phi': np.pi/4,  # 45 degrees
            'a': 50.0,
            'R': 900.0,
            'find_front': True
        },
        {
            'description': 'Case 2: Front foot fixed, find rear foot', 
            'fixed_foot_X': 0.0,
            'fixed_foot_theta': -np.pi/2,
            'phi': np.pi/4,  # 45 degrees
            'a': 50.0,
            'R': 900.0,
            'find_front': False
        }
    ]
    
    for i, case in enumerate(test_cases):
        print(f"\nTest Case {i+1}: {case['description']}")
        print(f"  Fixed foot: X={case['fixed_foot_X']}, θ={case['fixed_foot_theta']:.3f}rad")
        print(f"  Forward angle φ={case['phi']:.3f}rad ({np.degrees(case['phi']):.1f}°)")
        print(f"  Stroke a={case['a']}, Radius R={case['R']}")
        
        try:
            delta_X, delta_theta = robot.calc_other_foot_position(
                case['fixed_foot_X'], 
                case['fixed_foot_theta'], 
                case['phi'], 
                case['a'], 
                case['R'], 
                case['find_front']
            )
            
            print(f"  Result: ΔX={delta_X:.6f}, Δθ={delta_theta:.6f}rad ({np.degrees(delta_theta):.3f}°)")
            
            # Calculate new position
            new_X = case['fixed_foot_X'] + delta_X
            new_theta = case['fixed_foot_theta'] + delta_theta
            print(f"  New position: X={new_X:.6f}, θ={new_theta:.6f}rad ({np.degrees(new_theta):.3f}°)")
            
            # Verify the geometric constraint
            if case['find_front']:
                # Rear foot fixed, front foot moving
                expected_radius = np.sqrt(robot.d1**2 + robot.h**2)
                print(f"  Expected circle radius: {expected_radius:.3f}")
            else:
                # Front foot fixed, rear foot moving  
                expected_radius = np.sqrt((case['a'] + robot.d2)**2 + robot.h**2)
                print(f"  Expected circle radius: {expected_radius:.3f}")
                
        except Exception as e:
            print(f"  Error: {e}")
    
    print("\n" + "="*70)
    print("Testing completed!")

if __name__ == "__main__":
    test_corrected_algorithm()