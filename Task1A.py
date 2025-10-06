import sympy as sp
import numpy as np
import control as ct

########## Initialization of  variables & provided  
# Define the symbolic variables
x1, x2, u = sp.symbols('x1 x2 u')

# Define the differential equations
x1_dot = -x1 + x2 + 4*u
x2_dot = -x1 - x2 + 4*x1*(x2**2) + 2*u
##################################################


def find_equilibrium_points():
    '''
    1. Substitute input(u) = 0 in both equation for finding equilibrium points 
    2. Equate x1_dot, x2_dot equal to zero for finding equilibrium points 
    3. solve the x1_dot, x2_dot equations for the unknown variables and save the value to the variable namely "equi_points"
    4. Convert solutions to list of tuples
    '''

    ###### WRITE YOUR CODE HERE ################
    # Substitute u = 0
    eq1 = x1_dot.subs(u, 0)
    eq2 = x2_dot.subs(u, 0)
    
    # Equate both to zero and solve
    sol = sp.solve([eq1, eq2], (x1, x2))
    
    if isinstance(sol, list):
        if isinstance(sol[0], dict):  # if sympy returns list of dicts
            equi_points = [(s[x1], s[x2]) for s in sol]
        else:  # if sympy returns list of tuples
            equi_points = [(s[0], s[1]) for s in sol]
    else:
        equi_points = [(sol[x1], sol[x2])]
    ############################################

    return equi_points

def find_A_B_matrices(eq_points):
    '''
    1. Substitute every equilibrium points that you have already find in the find_equilibrium_points() function 
    2. After substituting the equilibrium points, Save the Jacobian matrices A and B as A_matrices, B_matrices  
    3. Return the list of A and B matrices
    '''
    A_matrix = sp.Matrix([
        [sp.diff(x1_dot, x1), sp.diff(x1_dot, x2)],
        [sp.diff(x2_dot, x1), sp.diff(x2_dot, x2)]
    ])
    
    B_matrix = sp.Matrix([
        [sp.diff(x1_dot, u)],
        [sp.diff(x2_dot, u)]
    ])
    A_matrices, B_matrices = [], []
    
    ###### WRITE YOUR CODE HERE ################
    for point in eq_points:
        # Substitute equilibrium values (x1, x2)
        subs_dict = {x1: point[0], x2: point[1], u: 0}

        # Evaluate A and B matrices at equilibrium point
        A_eval = A_matrix.subs(subs_dict)
        B_eval = B_matrix.subs(subs_dict)

        # Append to list
        A_matrices.append(A_eval)
        B_matrices.append(B_eval)
    ############################################
    
    return A_matrices,B_matrices


def find_eigen_values(A_matrices):
    '''
    1.  Find the eigen values of all A_matrices (You can use the eigenvals() function of sympy) 
        and append it to the 'eigen_values' list
    2.  With the eigen values, determine whether the system is stable or not and
        append the string 'Stable' if system is stable, else append the string 'Unstable'
        to the 'stability' list 
    '''
    
    eigen_values = []
    stability = []

    ###### WRITE YOUR CODE HERE ################
    for A in A_matrices:
        # Compute eigenvalues
        eigs = A.eigenvals()
        eigen_values.append(eigs)

        # Check stability
        if all(sp.re(val) < 0 for val in eigs):
            stability.append('Stable')
        else:
            stability.append('Unstable')
    ############################################
    return eigen_values, stability

def compute_lqr_gain(jacobians_A, jacobians_B):
    K = 0
    '''
    This function is use to compute the LQR gain matrix K
    1. Use the Jacobian A and B matrix at the unstable equilibrium point, and assign it to A and B respectively for computing LQR gain
    2. Compute the LQR gain of the given system equation (You can use lqr() of control module) 
    3. Take the A matrix corresponding to the Unstable Equilibrium point, that you have already found for computing LQR gain.
    4. Assign the value of gain to the variable K
    '''

    # Define the Q and R matrices
    Q = np.eye(2)  # State weighting matrix
    R = np.array([1])  # Control weighting matrix

    ###### WRITE YOUR CODE HERE ################

    # Define state and control weighting matrices
    Q = np.eye(2)   # Penalize state deviation equally
    R = np.array([[1]])  # Penalize control effort

    ###### WRITE YOUR CODE HERE ################
    # Step 1: Select A, B corresponding to the unstable equilibrium
    # (Assuming unstable point is the second matrix in the list)
    A = np.array(jacobians_A[1])
    B = np.array(jacobians_B[1])

    # Step 2: Compute LQR gain
    K, S, E = ct.lqr(A, B, Q, R)

    # Step 3: Convert K to a numpy array if needed
    K = np.array(K)
    ############################################
    return K

def main_function(x1_dot, x2_dot, u): # Don't change anything in this function 
    # Find equilibrium points
    eq_points = find_equilibrium_points()
    
    if not eq_points:
        print("No equilibrium points found.")
        return None, None, None, None, None, None
    
    # Find Jacobian matrices
    jacobians_A, jacobians_B = find_A_B_matrices(eq_points)
    
    # For finding eigenvalues and stability of the given equation
    eigen_values, stability = find_eigen_values(jacobians_A)
    
    
    # Compute the LQR gain matrix K
    K = compute_lqr_gain(jacobians_A, jacobians_B)
    
    return eq_points, jacobians_A,  eigen_values, stability, K


def task1a_output():
    '''
    This function will print the results that you have obtained 
    '''
    print("Equilibrium Points:")
    for i, point in enumerate(eq_points):
        print(f"  Point {i + 1}: x1 = {point[0]}, x2 = {point[1]}")
    
    print("\nJacobian Matrices at Equilibrium Points:")
    for i, matrix in enumerate(jacobians_A):
        print(f"  At Point {i + 1}:")
        print(sp.pretty(matrix, use_unicode=True))
    
    print("\nEigenvalues at Equilibrium Points:")
    for i, eigvals in enumerate(eigen_values):
        eigvals_str = ', '.join([f"{val}: {count}" for val, count in eigvals.items()])
        print(f"  At Point {i + 1}: {eigvals_str}")
    
    print("\nStability of Equilibrium Points:")
    for i, status in enumerate(stability):
        print(f"  At Point {i + 1}: {status}")
    
    print("\nLQR Gain Matrix K at the selected Equilibrium Point:")
    print(K)


if __name__ == "__main__":
    # Run the main function
    results = main_function(x1_dot, x2_dot, u)
    
    # This will get the equilibrium points, A_matrix, eigen values, stability of the system
    eq_points, jacobians_A, eigen_values, stability, K = results

    # print the results
    task1a_output()