import numpy as np

def sysCall_init():
    sim = require('sim')
    global body_handle, right_joint_handle, left_joint_handle
    global prismatic_joint_handle, arm_joint_handle
    global K
    global desired_position, desired_angle, desired_orientation
    global manual_velocity_bias, manual_turning_bias
    global prismatic_joint_velocity, arm_joint_velocity
    global last_turning_signal, key_pressed_this_frame
    global max_turning_rate, orientation_gain

    # Handles
    body_handle = sim.getObject('/body')
    right_joint_handle = sim.getObject('/right_joint')
    left_joint_handle = sim.getObject('/left_joint')
    prismatic_joint_handle = sim.getObject('/Prismatic_joint')
    arm_joint_handle = sim.getObject('/arm_joint')

    # Desired state
    desired_position = -2.85366
    desired_angle = 0.0
    desired_orientation = np.array([0.0, 1.0])  # robot initially faces +Y

    # LQR Gain matrix
    K = np.array([[-10, -7.80757335, 127.4382531, 23.19836625]])

    # Turning control
    max_turning_rate = 0.8
    orientation_gain = 2.0

    # Manual control variables
    manual_velocity_bias = 0.0
    manual_turning_bias = 0.0
    prismatic_joint_velocity = 0.0
    arm_joint_velocity = 0.0

    last_turning_signal = 0.0
    key_pressed_this_frame = False



def sysCall_actuation():
    global manual_velocity_bias, manual_turning_bias
    global desired_position, desired_angle, desired_orientation
    global prismatic_joint_velocity, arm_joint_velocity
    global max_turning_rate, orientation_gain
    global last_turning_signal, key_pressed_this_frame

    key_pressed_this_frame = False

    # -------------------------------------
    # READ ROBOT STATE
    # -------------------------------------
    pos_world = np.array(sim.getObjectPosition(body_handle, -1))
    linear_velocity, ang_vel_vec = sim.getObjectVelocity(body_handle)
    orientation = sim.getObjectOrientation(body_handle, -1)

    angle = orientation[0]              # body pitch (forward/back tilt)
    yaw = orientation[2]                # robot heading

    # Rotation matrix
    M = sim.getObjectMatrix(body_handle, -1)

    # robot forward axis (local Y) in world-frame
    forward = np.array([M[1], M[5], M[9]])
    forward_2d = np.array([forward[0], forward[1]])
    forward_2d /= np.linalg.norm(forward_2d)

    # -------------------------------------
    # CORRECTLY COMPUTE FORWARD POSITION
    # Project world position onto robot forward direction
    # -------------------------------------
    pos_xy = np.array([pos_world[0], pos_world[1]])
    position = np.dot(pos_xy, forward_2d)

    # Forward velocity (local Y)
    rot = np.array([
        [M[0], M[1], M[2]],
        [M[4], M[5], M[6]],
        [M[8], M[9], M[10]]
    ])
    linear_local = rot.T.dot(np.array(linear_velocity))
    velocity = 15 * linear_local[1]

    # Angular velocity (pitch) ? around Y axis
    angular_velocity = 15 * ang_vel_vec[1]

    # -------------------------------------
    # HEADING ERROR USING atan2 (robust)
    # -------------------------------------
    dot = np.clip(np.dot(forward_2d, desired_orientation), -1.0, 1.0)
    cross_z = forward_2d[0]*desired_orientation[1] - forward_2d[1]*desired_orientation[0]
    orientation_error = np.arctan2(cross_z, dot)

    if abs(orientation_error) < 0.10:
        orientation_error = 0.0

    # Turning signal
    turning_signal = orientation_gain * orientation_error
    turning_signal = np.clip(turning_signal, -max_turning_rate, max_turning_rate)

    # Smoother turning
    turning_signal = 0.7*turning_signal + 0.3*last_turning_signal
    last_turning_signal = turning_signal

    # -------------------------------------
    # LQR CONTROL
    # -------------------------------------
    state = np.array([position, velocity, angle, angular_velocity])
    target = np.array([desired_position, 0, desired_angle, 0])
    control_signal = -np.dot(K, (state - target))

    # -------------------------------------
    # KEY INPUTS
    # -------------------------------------
    msg, data, data2 = sim.getSimulatorMessage()
    if msg == sim.message_keypress:
        key_pressed_this_frame = True
        key = data[0]

        if key == 2007: manual_velocity_bias += 0.5      # up
        elif key == 2008: manual_velocity_bias -= 0.5     # down

        # LEFT 90? TURN
        elif key == 2009:
            desired_orientation = rotate_vector_2d(forward_2d, (3/2)*np.pi)
            desired_position = position

        # RIGHT 90? TURN
        elif key == 2010:
            desired_orientation = rotate_vector_2d(forward_2d, -(3/2)*np.pi)
            desired_position = position

        elif key == 32:  # Space bar
            manual_velocity_bias = 0.0
            desired_position = position
            desired_orientation = forward_2d
            sim.setJointTargetVelocity(left_joint_handle, 0.0)
            sim.setJointTargetVelocity(right_joint_handle, 0.0)

        if key == 113: prismatic_joint_velocity -= 0.02
        elif key == 101: prismatic_joint_velocity += 0.02
        if key == 119: arm_joint_velocity += 0.2
        elif key == 115: arm_joint_velocity -= 0.2


    # -------------------------------------
    # UPDATE TARGET POSITION SMOOTHLY
    # -------------------------------------
    manual_velocity_bias = np.clip(manual_velocity_bias, -1.0, 1.0)
    if abs(manual_velocity_bias) > 0.01:
        desired_position += manual_velocity_bias * 0.012


    # -------------------------------------
    # APPLY WHEEL COMMANDS
    # -------------------------------------
    left_vel = np.clip(control_signal[0] - turning_signal, -10, 10)
    right_vel = np.clip(control_signal[0] + turning_signal, -10, 10)

    sim.setJointTargetVelocity(left_joint_handle, left_vel)
    sim.setJointTargetVelocity(right_joint_handle, right_vel)
    sim.setJointTargetVelocity(prismatic_joint_handle, prismatic_joint_velocity)
    sim.setJointTargetVelocity(arm_joint_handle, arm_joint_velocity)



def rotate_vector_2d(v, a):
    c, s = np.cos(a), np.sin(a)
    return np.array([v[0]*c - v[1]*s, v[0]*s + v[1]*c])


def sysCall_sensing():
    pass

def sysCall_cleanup():
    pass