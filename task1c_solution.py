import numpy as np
import scipy.linalg

def sysCall_init():
    sim = require('sim')
    
    # Robot part handles
    global chassis, left_motor, right_motor
    global K_gain
    global ref_pos, ref_angle
    global speed_offset, turn_offset

    chassis = sim.getObject('/body')
    left_motor = sim.getObject('/left_joint')
    right_motor = sim.getObject('/right_joint')

    # Desired states
    ref_pos = 0.5
    ref_angle = 0.0

    K_gain = np.array([[[-10,-7.80757335,127.4382531,23.19836625]]])
    # Give robot a slight spin at start
    start_torque = 0.9
    sim.addForceAndTorque(chassis, [0, 0, 0], [0, 0, start_torque])

    # Manual control modifiers
    speed_offset = 0.0
    turn_offset = 0.0


def sysCall_actuation():
    global K_gain, speed_offset, turn_offset, ref_pos, ref_angle

    # Get robot state info
    pos_y = sim.getObjectPosition(chassis, -1)[1]
    lin_vel, ang_vel_vec = sim.getObjectVelocity(chassis)
    vel_y = 10 * lin_vel[1]
    ang_vel_y = 10 * ang_vel_vec[1]
    angle_y = sim.getObjectOrientation(chassis, -1)[1]

    # Build state and reference vectors
    x_state = np.array([pos_y, vel_y, angle_y, ang_vel_y])
    ref_state = np.array([ref_pos, 0, ref_angle, 0])

    # Compute control output (u = -K(x - ref))
    u_cmd = -K_gain @ (x_state - ref_state)

    # Check for keyboard inputs (manual override)
    msg, key_data, _ = sim.getSimulatorMessage()
    if msg == sim.message_keypress:
        key = key_data[0]
        if key == 2007:      # Up
            speed_offset += 1.2
        elif key == 2008:    # Down
            speed_offset -= 1.2
        elif key == 2009:    # Left
            turn_offset = 1
        elif key == 2010:    # Right
            turn_offset = -1
        elif key == 32:      # Space
            speed_offset = -1
            turn_offset = 0
        else:
            speed_offset = 0
            turn_offset = 0

    # Limit manual inputs
    speed_offset = np.clip(speed_offset, -1.0, 1.0)
    turn_offset = np.clip(turn_offset, -1.0, 1.0)

    # Update reference point
    ref_pos += speed_offset * 0.012
    spin_torque = turn_offset * 0.1

    # Debug print
    print(f"State = {x_state}")
    print(f"Control = {u_cmd}")
    print(f"Speed offset = {speed_offset}")
    print(f"Turn offset = {turn_offset}")
    print(f"Ref position = {ref_pos}")
    print(f"Ref angle = {ref_angle}")

    # Apply motor control (including turning correction)
    v_left = u_cmd.item() + turn_offset
    v_right = u_cmd.item() - turn_offset
    
    sim.setJointTargetVelocity(left_motor, v_left)
    sim.setJointTargetVelocity(right_motor, v_right)


def sysCall_sensing():
    pass


def sysCall_cleanup():
    pass