import numpy as np

def sysCall_init():
    sim = require('sim')

    # Declare globals
    global robot_handle, right_wheel_handle, left_wheel_handle
    global lift_joint_handle, arm_joint_handle
    global gain_matrix
    global ref_position, ref_pitch
    global drive_bias, turn_bias
    global lift_speed, arm_speed

    # Retrieve object handles
    robot_handle = sim.getObject('/body')
    right_wheel_handle = sim.getObject('/right_joint')
    left_wheel_handle = sim.getObject('/left_joint')
    lift_joint_handle = sim.getObject('/Prismatic_joint')
    arm_joint_handle = sim.getObject('/arm_joint')

    # State feedback gain (unchanged logic)
    gain_matrix = np.array([[-10, -7.80757335, 127.4382531, 23.19836625]])

    # Target posture
    ref_position = 0.0
    ref_pitch = 0.0

    # Manual controls
    drive_bias = 0.0         # Forward/Backward bias
    turn_bias = 0.0          # Left/Right steering bias
    lift_speed = 0.0         # Vertical lift joint speed
    arm_speed = 0.0          # Arm joint speed


def sysCall_actuation():
    global gain_matrix
    global drive_bias, turn_bias
    global ref_position, ref_pitch
    global lift_speed, arm_speed

    # --------- READ CURRENT STATE ---------
    pos_y = sim.getObjectPosition(robot_handle, -1)[1]
    vel_lin, vel_ang = sim.getObjectVelocity(robot_handle)

    forward_velocity = 10 * vel_lin[1]
    pitch_rate = 10 * vel_ang[1]
    pitch_angle = sim.getObjectOrientation(robot_handle, -1)[0]

    state_vector = np.array([pos_y, forward_velocity, pitch_angle, pitch_rate])
    target_vector = np.array([ref_position, 0, ref_pitch, 0])

    control_output = -np.dot(gain_matrix, (state_vector - target_vector))

    # --------- KEYBOARD INPUT ---------
    msg, key, _ = sim.getSimulatorMessage()

    if msg == sim.message_keypress:

        key_code = key[0]

        # Forward / backward drive
        if key_code == 2007:       # Up arrow
            drive_bias += 0.1
        elif key_code == 2008:     # Down arrow
            drive_bias -= 0.1

        # Steering control
        elif key_code == 2009:     # Left arrow
            turn_bias += 0.2
        elif key_code == 2010:     # Right arrow
            turn_bias -= 0.2

        # Emergency brake / reverse pulse
        elif key_code == 32:       # Space bar
            drive_bias = 0.0                # manual_velocity_bias
            ref_position = current_position # desired_position
            ref_heading = forward_vector    # desired_orientation

            sim.setJointTargetVelocity(left_wheel_handle, 0.0)
            sim.setJointTargetVelocity(right_wheel_handle, 0.0)


        # Default fallback for other keys
        else:
            drive_bias = 0
            turn_bias = 0

        # Lift control (Q/E)
        if key_code == 113:
            lift_speed -= 0.02
        elif key_code == 101:
            lift_speed += 0.02

        # Arm control (W/S)
        if key_code == 119:
            arm_speed += 0.2
        elif key_code == 115:
            arm_speed -= 0.2

    # --------- LIMITS ---------
    lift_speed = np.clip(lift_speed, -1.0, 1.0)
    arm_speed = np.clip(arm_speed, -1.0, 1.0)
    drive_bias = np.clip(drive_bias, -1.0, 2.0)
    turn_bias = np.clip(turn_bias, -1.0, 1.0)

    # --------- UPDATE REFERENCE POSITION ---------
    ref_position += drive_bias * 0.012

    # --------- WHEEL VELOCITY COMMANDS ---------
    left_speed = control_output[0] - turn_bias
    right_speed = control_output[0] + turn_bias

    sim.setJointTargetVelocity(left_wheel_handle, left_speed)
    sim.setJointTargetVelocity(right_wheel_handle, right_speed)

    # --------- LIFT & ARM ACTUATION ---------
    sim.setJointTargetVelocity(lift_joint_handle, lift_speed)
    sim.setJointTargetVelocity(arm_joint_handle, arm_speed)

    print(f"{control_output[0]}")


def sysCall_sensing():
    pass


def sysCall_cleanup():
    pass