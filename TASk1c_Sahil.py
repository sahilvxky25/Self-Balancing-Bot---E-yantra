def sysCall_init():
    sim = require('sim')
    
    # Get robot components
    global bot_body, right_joint, left_joint
    bot_body = sim.getObject('/body')
    right_joint = sim.getObject('/body/right_joint')
    left_joint = sim.getObject('/body/left_joint')
    print("? Handles found:", bot_body, right_joint, left_joint)
    
    # LQR Gain matrix [K_theta, K_theta_dot, K_x, K_x_dot]
    global K
    K = [119, 40,-43, -169]
    
    # Control parameters
    global MOVE_INCREMENT, YAW_RATE, MAX_SPEED
    MOVE_INCREMENT = 0.05  # Position increment per keypress
    YAW_RATE = 0.2         # Rotation speed
    MAX_SPEED = 50.0       # Maximum wheel velocity
    
    # Reference/desired values
    global x_ref, yaw_input, desired_theta
    x_ref = 0.0          # Desired forward position
    yaw_input = 0.0      # Manual rotation input
    desired_theta = 0.0  # Desired tilt angle (usually 0 for upright)
    
    # Current state variables
    global theta, theta_dot, x_pos, x_dot
    theta = theta_dot = x_pos = x_dot = 0.0
    
    # Previous values for velocity computation
    global prev_theta, prev_x
    prev_theta = 0.0
    prev_x = 0.0
    
    # Time step
    global dt
    dt = sim.getSimulationTimeStep()
    
    # Low-pass filter parameters
    global alpha, filtered_state
    alpha = 0.7  # Filter coefficient (0-1, higher = less smoothing)
    filtered_state = {
        'theta': 0.0,
        'theta_dot': 0.0,
        'x_pos': 0.0,
        'x_dot': 0.0
    }
    
    # Key press tracking
    global key_pressed
    key_pressed = False
    
    print("? Initialization complete. Use arrow keys to control:")
    print("  ?/? : Move forward/backward")
    print("  ?/? : Turn left/right")


def sysCall_sensing():
    sim = require('sim')
    
    global theta, theta_dot, x_pos, x_dot, x_ref, yaw_input
    global bot_body, prev_theta, prev_x, dt
    global filtered_state, alpha, key_pressed
    global MOVE_INCREMENT, YAW_RATE
    
    # Get current robot state
    euler = sim.getObjectOrientation(bot_body, -1)
    pos = sim.getObjectPosition(bot_body, -1)
    
    # Extract roll angle (tilt around X-axis) and position
    theta = euler[0]  # Roll angle
    x_pos = pos[0]    # X position
    
    # Calculate velocities using finite differences
    theta_dot = (theta - prev_theta) / dt
    x_dot = (x_pos - prev_x) / dt
    
    # Update previous values for next iteration
    prev_theta = theta
    prev_x = x_pos
    
    # Handle keyboard input
    message, auxiliaryData, aux2 = sim.getSimulatorMessage()
    
    if message == sim.message_keypress:
        key_pressed = True
        key_code = auxiliaryData[0]
        
        # Arrow key codes in CoppeliaSim:
        # 2007: Up arrow
        # 2008: Down arrow
        # 2009: Left arrow
        # 2010: Right arrow
        
        if key_code == 2007:  # Up arrow - move forward
            x_ref += MOVE_INCREMENT
            print(f"Forward: target position = {x_ref:.2f}")
        elif key_code == 2008:  # Down arrow - move backward
            x_ref -= MOVE_INCREMENT
            print(f"Backward: target position = {x_ref:.2f}")
        elif key_code == 2009:  # Left arrow - turn left
            yaw_input = YAW_RATE
        elif key_code == 2010:  # Right arrow - turn right
            yaw_input = -YAW_RATE
    else:
        # Reset yaw input when no key is pressed
        if key_pressed:
            yaw_input = 0.0
            key_pressed = False
    
    # Apply low-pass filter to reduce sensor noise
    # This smooths out rapid fluctuations in measurements
    filtered_state['theta'] = alpha * theta + (1 - alpha) * filtered_state['theta']
    filtered_state['theta_dot'] = alpha * theta_dot + (1 - alpha) * filtered_state['theta_dot']
    filtered_state['x_pos'] = alpha * x_pos + (1 - alpha) * filtered_state['x_pos']
    filtered_state['x_dot'] = alpha * x_dot + (1 - alpha) * filtered_state['x_dot']


def sysCall_actuation():
    sim = require('sim')
    
    global K, desired_theta, x_ref, yaw_input
    global filtered_state
    global right_joint, left_joint
    global MAX_SPEED
    
    # Extract filtered states
    theta = filtered_state['theta']
    theta_dot = filtered_state['theta_dot']
    x_pos = filtered_state['x_pos']
    x_dot = filtered_state['x_dot']
    
    # Compute state errors
    theta_error = theta - desired_theta      # Tilt error
    theta_dot_error = theta_dot              # Angular velocity error
    pos_error = x_pos - x_ref                # Position error
    pos_dot_error = x_dot                    # Linear velocity error
    
    # LQR control law: U = -K * x
    # The control input balances the robot and moves it to desired position
    U = -(K[0] * theta_error + 
          K[1] * theta_dot_error + 
          K[2] * pos_error + 
          K[3] * pos_dot_error)
    
    # Differential drive control
    # Left wheel gets U + yaw, right wheel gets U - yaw
    # This creates rotation when yaw_input is non-zero
    left_cmd = U + yaw_input
    right_cmd = U - yaw_input
    
    # Saturate commands to maximum speed
    left_cmd = max(-MAX_SPEED, min(MAX_SPEED, left_cmd))
    right_cmd = max(-MAX_SPEED, min(MAX_SPEED, right_cmd))
    
    # Apply velocity commands to wheel joints
    sim.setJointTargetVelocity(left_joint, left_cmd)
    sim.setJointTargetVelocity(right_joint, right_cmd)
    
    # Optional: Print debug info periodically
    if sim.getSimulationTime() % 1.0 < dt:  # Every 1 second
        print(f"?={theta:.3f}rad, x={x_pos:.2f}m, U={U:.2f}, L={left_cmd:.2f}, R={right_cmd:.2f}")


def sysCall_cleanup():
    sim = require('sim')
    
    global right_joint, left_joint
    
    # Stop the motors when simulation ends
    try:
        sim.setJointTargetVelocity(right_joint, 0)
        sim.setJointTargetVelocity(left_joint, 0)
        print("? Motors stopped")
    except:
        pass
    
    print("?? Cleanup complete.")