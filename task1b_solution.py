def sysCall_init():
    sim = require('sim')

    global bot_body, right_joint, left_joint
    bot_body = sim.getObject('/body')
    right_joint = sim.getObject('/body/right_joint')
    left_joint = sim.getObject('/body/left_joint')

    print("? Handles found:", bot_body, right_joint, left_joint)
    

    # Gain matrix [K_theta, K_theta_dot, K_x, K_x_dot]
    global K
    K = [215, 115, 25, 15]

    # Desired state
    global desired_theta, desired_pos
    desired_theta = 0.0
    desired_pos = 0.0

    # State variables
    global theta, theta_dot, x_pos, x_dot
    theta = theta_dot = x_pos = x_dot = 0.0

    # Previous values (for velocity computation)
    global prev_theta, prev_x, prev_time,dt
    prev_theta = 0.0
    prev_x = 0.0
    dt=sim.getSimulationTimeStep()

    # Low-pass filter coefficient
    global alpha
    alpha = 0.7

    # Filtered (smoothed) states
    global new_theta, new_theta_dot, new_pos, new_pos_dot
    new_theta = new_theta_dot = new_pos = new_pos_dot = 0.0

    print("? Initialization complete.")


def sysCall_sensing():
    sim = require('sim')
    global theta, theta_dot, x_pos, x_dot
    global bot_body, prev_theta, prev_x, dt
    global new_theta, new_theta_dot, new_pos, new_pos_dot, alpha

    # Get current values
    euler = sim.getObjectOrientation(bot_body, -1)
    pos = sim.getObjectPosition(bot_body, -1)
    theta = euler[0]  # roll (around X)
    x_pos = pos[0]


    # Calculate angular and linear velocity numerically
    theta_dot = (theta - prev_theta) / dt
    x_dot = (x_pos - prev_x) / dt

    # Update previous values
    prev_theta = theta
    prev_x = x_pos
    

    # Apply low-pass filter (smooth out noise)
    new_theta = alpha * theta + (1 - alpha) * new_theta
    new_theta_dot = alpha * theta_dot + (1 - alpha) * new_theta_dot
    new_pos = alpha * x_pos + (1 - alpha) * new_pos
    new_pos_dot = alpha * x_dot + (1 - alpha) * new_pos_dot


def sysCall_actuation():
    sim = require('sim')
    global K, desired_theta, desired_pos
    global new_theta, new_theta_dot, new_pos, new_pos_dot
    global right_joint, left_joint

    # Compute state errors
    theta_error = new_theta - desired_theta
    theta_dot_error = new_theta_dot
    pos_error = new_pos - desired_pos
    pos_dot_error = new_pos_dot

    # LQR control
    U = -(K[0] * theta_error + K[1] * theta_dot_error + K[2] * pos_error + K[3] * pos_dot_error)

    # Saturate
    U = max(min(U, 50), -50)

    # Apply velocity to both wheels
    sim.setJointTargetVelocity(right_joint, U)
    sim.setJointTargetVelocity(left_joint, U)
    #Print debug info periodically
    if sim.getSimulationTime() % 1.0 < dt:  # Every 1 second
        print(f"?={theta:.3f}rad, x={x_pos:.2f}m, U={U:.2f}")



def sysCall_cleanup():
    sim = require('sim')
    try:
        sim.setJointTargetVelocity(right_joint, 0)
        sim.setJointTargetVelocity(left_joint, 0)
    except:
        pass
    print("?? Cleanup complete.")