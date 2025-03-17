#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import sys
import tty
import termios

def getKey():
    # Get a single keypress from the user
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        # Handle arrow keys (escape sequences)
        if ch == '\x1b':
            ch = ch + sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rospy.init_node('landing_pad_teleop')
    
    # Set up the service proxy for setting model state
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    # Parameters
    model_name = "landing_pad"  # Make sure this matches your landing pad model name in Gazebo
    step_size = 0.1  # meters per keypress
    
    # Current position
    x_pos = 0.0
    y_pos = 10.0  # Based on your world file, the pad starts at (0, 10, 0.01)
    z_pos = 0.01
    
    print("Landing Pad Teleop Ready!")
    print("Controls: Arrow keys - move pad in x-y plane")
    print("         w/a/s/d - alternative controls to move pad")
    print("         i/k - increase/decrease step size")
    print("         ESC - quit")
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        key = getKey()
        
        if key == '\x1b':  # ESC key
            break
            
        # Handle arrow keys
        if key == '\x1b[A':  # Up arrow
            y_pos += step_size
        elif key == '\x1b[B':  # Down arrow
            y_pos -= step_size
        elif key == '\x1b[C':  # Right arrow
            x_pos += step_size
        elif key == '\x1b[D':  # Left arrow
            x_pos -= step_size
        # Alternative WASD controls
        elif key == 'w':
            y_pos += step_size
        elif key == 's':
            y_pos -= step_size
        elif key == 'd':
            x_pos += step_size
        elif key == 'a':
            x_pos -= step_size
        # Step size adjustment
        elif key == 'i':
            step_size += 0.05
            print(f"Step size increased to {step_size:.2f}")
        elif key == 'k':
            step_size = max(0.05, step_size - 0.05)  # Don't go below 0.05
            print(f"Step size decreased to {step_size:.2f}")
        
        # Create and send model state
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose.position.x = x_pos
        model_state.pose.position.y = y_pos
        model_state.pose.position.z = z_pos
        model_state.pose.orientation.w = 1.0  # No rotation
        
        try:
            resp = set_model_state(model_state)
            if resp.success:
                print(f"Pad position: x={x_pos:.2f}, y={y_pos:.2f}, z={z_pos:.2f}, step_size={step_size:.2f}")
            else:
                rospy.logwarn("Failed to set model state")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
