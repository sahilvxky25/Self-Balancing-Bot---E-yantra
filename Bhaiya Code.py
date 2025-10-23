import numpy as np
import scipy.linalg

def sysCall_init():
    sim = require('sim')
    global body_handle, right_joint_handle, left_joint_handle
    global A, B, K
    global desired_position, desired_angle
    global manual_velocity_bias, manual_turning_bias

    body_handle = sim.getObject('/body')
    right_joint_handle = sim.getObject('/right_joint')
    left_joint_handle = sim.getObject('/left_joint')
    desired_position = 0.5  
    desired_angle = 0.0  

    A = np.array([[0, 1, 0, 0],
                  [0, 0, -9.81, 0],
                  [0, 0, 0, 1],
                  [0, 0, 24.52, 0]])  

    B = np.array([[0],
                  [1],
                  [0],
                  [1]])

    Q = np.diag([100, 1, 300, 1])  
    R = np.array([[1]]) 
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    K = np.dot(np.linalg.inv(R), np.dot(B.T, P))

    initial_rotation_impulse = 0.9  
    sim.addForceAndTorque(body_handle, [0, 0, 0], [0, 0, initial_rotation_impulse])
    manual_velocity_bias = 0.0
    manual_turning_bias = 0.0  

def sysCall_actuation():
    global A, B, K, manual_velocity_bias, manual_turning_bias, desired_position, desired_angle

    position = sim.getObjectPosition(body_handle, -1)[1]  
    linear_velocity, angular_velocity_vector = sim.getObjectVelocity(body_handle)
    velocity = 10 * linear_velocity[1]  
    angular_velocity = 10 * angular_velocity_vector[1]  
    angle = sim.getObjectOrientation(body_handle, -1)[1]  
    state = np.array([position, velocity, angle, angular_velocity])
    target = np.array([desired_position, 0, desired_angle, 0])  
    control_signal = -np.dot(K, (state - target))

    message, data, data2 = sim.getSimulatorMessage()
    if message == sim.message_keypress:
        if data[0] == 2007:  # Up arrow 
            manual_velocity_bias += 1.2 
        elif data[0] == 2008:  # Down arrow 
            manual_velocity_bias += -1.2 
        elif data[0] == 2009:  # Left arrow
            manual_turning_bias = 1 
        elif data[0] == 2010:  # Right arrow
            manual_turning_bias = -1 
        elif data[0] == 32:  # Right arrow
            manual_velocity_bias = -1  
            manual_turning_bias = 0 
        else:
            manual_velocity_bias = 0 
            manual_turning_bias = 0  

    manual_velocity_bias = np.clip(manual_velocity_bias, -1.0, 1.0)
    manual_turning_bias = np.clip(manual_turning_bias, -1.0, 1.0)
    desired_position += manual_velocity_bias * 0.012
    initial_rotation_impulse = manual_turning_bias * 0.1  

    print(f"State vector: {state}")
    print(f"Control signal: {control_signal}")
    print(f"Manual velocity bias: {manual_velocity_bias}")
    print(f"Manual turning bias: {manual_turning_bias}")
    print(f"Desired position: {desired_position}")
    print(f"Desired angle: {desired_angle}")

    # Apply control signal to both wheels, considering turning bias
    left_wheel_velocity = control_signal[0] + manual_turning_bias 
    right_wheel_velocity = control_signal[0] - manual_turning_bias 

    sim.setJointTargetVelocity(left_joint_handle, left_wheel_velocity)
    sim.setJointTargetVelocity(right_joint_handle, right_wheel_velocity)

def sysCall_sensing():
    pass

def sysCall_cleanup():
    pass