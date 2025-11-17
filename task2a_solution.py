import numpy as np

def sysCall_init():
    sim = require('sim')
    # Declare globals
    global robot_handle, right_wheel_handle, left_wheel_handle
    global lift_joint_handle, arm_joint_handle
    global gain_matrix
    global ref_position, ref_pitch, ref_orientation
    global drive_bias, turn_bias
    global lift_speed, arm_speed
    global last_turn_signal
    global max_turn_rate, orientation_gain
    
    # Retrieve object handles
    robot_handle = sim.getObject('/body')
    right_wheel_handle = sim.getObject('/right_joint')
    left_wheel_handle = sim.getObject('/left_joint')
    lift_joint_handle = sim.getObject('/Prismatic_joint')
    arm_joint_handle = sim.getObject('/arm_joint')
    
    # State feedback gain
    gain_matrix = np.array([[-10, -7.80757335, 127.4382531, 23.19836625]])
    
    # Target posture
    ref_position = 0.0
    ref_pitch = 0.0
    ref_orientation = np.array([0.0, 1.0])  # Initial forward direction (world +Y)
    
    # Manual controls
    drive_bias = 0.0
    turn_bias = 0.0
    lift_speed = 0.0
    arm_speed = 0.0
    
    # Turning control parameters
    last_turn_signal = 0.0
    max_turn_rate = 0.8
    orientation_gain = 2.0
    
    print("? Initialization complete with rotation matrix support!")

def sysCall_actuation():
    global gain_matrix
    global drive_bias, turn_bias
    global ref_position, ref_pitch, ref_orientation
    global lift_speed, arm_speed
    global last_turn_signal, max_turn_rate, orientation_gain
    
    # ============================================
    # READ ROBOT STATE WITH ROTATION MATRIX
    # ============================================
    
    # Get world position
    pos_world = np.array(sim.getObjectPosition(robot_handle, -1))
    
    # Get velocities
    vel_lin, vel_ang = sim.getObjectVelocity(robot_handle)
    
    # Get orientation
    orientation = sim.getObjectOrientation(robot_handle, -1)
    pitch_angle = orientation[0]  # Forward/back tilt
    yaw = orientation[2]          # Heading
    
    # ============================================
    # EXTRACT ROTATION MATRIX
    # ============================================
    M = sim.getObjectMatrix(robot_handle, -1)
    
    # Extract robot's forward direction (local Y-axis) in world frame
    forward_3d = np.array([M[1], M[5], M[9]])
    forward_2d = np.array([forward_3d[0], forward_3d[1]])
    forward_2d_norm = np.linalg.norm(forward_2d)
    
    if forward_2d_norm > 0.001:
        forward_2d = forward_2d / forward_2d_norm
    else:
        forward_2d = np.array([0.0, 1.0])  # Fallback
    
    # Build 3x3 rotation matrix for velocity transformation
    rot_matrix = np.array([
        [M[0], M[1], M[2]],    # X-axis (robot right)
        [M[4], M[5], M[6]],    # Y-axis (robot forward)
        [M[8], M[9], M[10]]    # Z-axis (robot up)
    ])
    
    # ============================================
    # COMPUTE POSITION IN ROBOT FRAME
    # ============================================
    # Project world position onto robot's forward direction
    pos_xy = np.array([pos_world[0], pos_world[1]])
    position = np.dot(pos_xy, forward_2d)
    
    # ============================================
    # COMPUTE VELOCITY IN ROBOT FRAME
    # ============================================
    # Transform world velocity to robot's local frame
    vel_world = np.array(vel_lin)
    vel_local = rot_matrix.T.dot(vel_world)
    forward_velocity = 20 * vel_local[1]  # Velocity along robot's forward axis
    
    # ============================================
    # COMPUTE ANGULAR VELOCITY (PITCH RATE)
    # ============================================
    pitch_rate = 3 * vel_ang[1]  # FIXED: 15x for proper damping
    
    # ============================================
    # HEADING ERROR & AUTOMATIC TURNING
    # ============================================
    # Compute angle between current forward and desired orientation
    dot_product = np.clip(np.dot(forward_2d, ref_orientation), -1.0, 1.0)
    cross_z = forward_2d[0] * ref_orientation[1] - forward_2d[1] * ref_orientation[0]
    orientation_error = np.arctan2(cross_z, dot_product)
    
    # Dead zone to prevent micro-corrections
    if abs(orientation_error) < 0.10:  # ~5.7 degrees
        orientation_error = 0.0
    
    # Compute automatic turning signal
    auto_turn_signal = orientation_gain * orientation_error
    auto_turn_signal = np.clip(auto_turn_signal, -max_turn_rate, max_turn_rate)
    
    # Smooth the turning signal
    auto_turn_signal = 0.7 * auto_turn_signal + 0.3 * last_turn_signal
    last_turn_signal = auto_turn_signal
    
    # ============================================
    # LQR CONTROL
    # ============================================
    state_vector = np.array([position, forward_velocity, pitch_angle, pitch_rate])
    target_vector = np.array([ref_position, 0, ref_pitch, 0])
    control_output = -np.dot(gain_matrix, (state_vector - target_vector))
    control_output = np.clip(control_output, -10, 10)
    
    # ============================================
    # KEYBOARD INPUT
    # ============================================
    msg, key, _ = sim.getSimulatorMessage()
    if msg == sim.message_keypress:
        key_code = key[0]
        
        # Forward / backward drive
        if key_code == 2007:       # Up arrow
            drive_bias += 0.5
        elif key_code == 2008:     # Down arrow
            drive_bias -= 0.1
        
        # 90? ROTATION COMMANDS
        elif key_code == 2009:     # Left arrow - turn LEFT 90?
            ref_orientation = rotate_vector_2d(forward_2d, np.pi/2)
            ref_position = position  # Lock current position during turn
        
        elif key_code == 2010:     # Right arrow - turn RIGHT 90?
            ref_orientation = rotate_vector_2d(forward_2d, -np.pi/2)
            ref_position = position  # Lock current position during turn
        
        # Emergency stop
        elif key_code == 32:       # Space bar
            drive_bias = 0.0
            turn_bias = 0.0
            ref_position = position
            ref_orientation = forward_2d  # Lock current heading
            sim.setJointTargetVelocity(left_wheel_handle, 0.0)
            sim.setJointTargetVelocity(right_wheel_handle, 0.0)
            print("? EMERGENCY STOP")
        
        # Lift control (Q/E)
        if key_code == 113:        # q
            lift_speed -= 0.07
        elif key_code == 101:      # e
            lift_speed += 0.07
        
        # Arm control (W/S)
        if key_code == 119:        # w
            arm_speed += 0.5
        elif key_code == 115:      # s
            arm_speed -= 0.5
    
    # ============================================
    # APPLY LIMITS
    # ============================================
    lift_speed = np.clip(lift_speed, -1.0, 1.0)
    arm_speed = np.clip(arm_speed, -1.0, 1.0)
    drive_bias = np.clip(drive_bias, -1.0, 2.0)
    
    # Reduce drive speed during sharp turns
    if abs(orientation_error) > 0.3:  # ~17 degrees
        drive_bias *= 0.5
    
    # ============================================
    # UPDATE REFERENCE POSITION
    # ============================================
    if abs(drive_bias) > 0.01:
        ref_position += drive_bias * 0.012
    
    # ============================================
    # COMBINE MANUAL AND AUTOMATIC TURNING
    # ============================================
    # Manual turn_bias is now ADDITIVE to automatic correction
    total_turn_signal = auto_turn_signal + turn_bias
    total_turn_signal = np.clip(total_turn_signal, -max_turn_rate * 1.5, max_turn_rate * 1.5)
    
    # ============================================
    # WHEEL VELOCITY COMMANDS
    # ============================================
    left_speed = np.clip(control_output[0] - total_turn_signal, -10, 10)
    right_speed = np.clip(control_output[0] + total_turn_signal, -10, 10)
    
    sim.setJointTargetVelocity(left_wheel_handle, left_speed)
    sim.setJointTargetVelocity(right_wheel_handle, right_speed)
    
    # ============================================
    # LIFT & ARM ACTUATION
    # ============================================
    sim.setJointTargetVelocity(lift_joint_handle, lift_speed)
    sim.setJointTargetVelocity(arm_joint_handle, arm_speed)
    
    # ============================================
    # DEBUG OUTPUT (reduced frequency)
    # ============================================
    if sim.getSimulationTime() % 0.5 < 0.05:  # Every 0.5 seconds
        heading_deg = np.degrees(yaw)
        error_deg = np.degrees(orientation_error)
        print(f"Ctrl: {control_output[0]:5.2f} | Pitch: {pitch_angle:6.3f} | "
              f"Pos: {position:6.2f} | Head: {heading_deg:6.1f}? | Err: {error_deg:5.1f}?")


def rotate_vector_2d(v, angle):
    """Rotate a 2D vector by given angle (radians)"""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([v[0]*c - v[1]*s, v[0]*s + v[1]*c])


def sysCall_sensing():
    pass

def sysCall_cleanup():
    pass