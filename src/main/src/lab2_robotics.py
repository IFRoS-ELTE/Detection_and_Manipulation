import numpy as np  # Import Numpy
import math

def DH(d, theta, a, alpha):
    '''
        Function builds elementary Denavit-Hartenberg transformation matrices 
        and returns the transformation matrix resulting from their multiplication.

        Arguments:
        d (double): displacement along Z-axis
        theta (double): rotation around Z-axis
        a (double): displacement along X-axis
        alpha (double): rotation around X-axis

        Returns:
        (Numpy array): composition of elementary DH transformations
    '''
    Rotz = np.array([[np.cos(theta), -np.sin(theta), 0, 0], [np.sin(theta), np.cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    # Computing the basic rotation around z axis
    Tranz = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])
    # Computing the basic translation along z axis
    Rotx = np.array([[1,0,0,0],[0, np.cos(alpha), -np.sin(alpha),0],[0,np.sin(alpha), np.cos(alpha),0], [0,0,0,1]])
    # Computing the basic rotation around x axis
    Tranx = np.array([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    # Computing the basic translation along x axis

    T = np.dot(np.dot(np.dot(Rotz, Tranz), Rotx), Tranx)
    # Final transformation is product of the previous three
    
    return T

def kinematics(d, theta, a, alpha):
    '''
        Functions builds a list of transformation matrices, for a kinematic chain,
        descried by a given set of Denavit-Hartenberg parameters. 
        All transformations are computed from the base frame.

        Arguments:
        d (list of double): list of displacements along Z-axis
        theta (list of double): list of rotations around Z-axis
        a (list of double): list of displacements along X-axis
        alpha (list of double): list of rotations around X-axis

        Returns:
        (list of Numpy array): list of transformations along the kinematic chain (from the base frame)
    '''
    T = [np.eye(4)] # Base transformation

    for i in range(len(a)):
        N = T[i] @ DH(d[i], theta[i], a[i], alpha[i])
        # For all DH parameters the transformation between the respective frame and base frame is computed by using previously implemented function DH().

        T.append(N)     
        # All those transformations are then put in the list so they can be used 
  
    return T

# Inverse kinematics

def jacobian(T, revolute):
    """
    Computes the Jacobian of the end-effector of a robot given its kinematic transformations and joint types.

    Parameters:
    T (list of numpy.ndarray): List of transformations along the kinematic chain of the robot (from the base frame).
    revolute (list of bool): List of flags specifying if the corresponding joint is a revolute joint.

    Returns:
    numpy.ndarray: End-effector Jacobian.
    """
    
    num_joints = len(T) - 1
    # Helping variable which defines number of joints
    J = np.zeros((6, num_joints))
    # Initialization of Jacobian with zeros

    O = [T[i][0:3, 3] for i in range(len(T))]
    # Making a list of translation vectors of all the  matrices
    z = [T[i][0:3, 2] for i in range(num_joints)]
    # Making a list of z coordinates of all the transtransformationformation matrices

    for i in range(num_joints):
        if revolute[i]:
        # In case that joint is revolute, the following action will be applied. This part is necessary since array revolute contains Boolean and not integer type
            J[:3, i] = np.cross(z[i], O[-1] - O[i])
            J[3:, i] = z[i]
        else:
        # Otherwise, the formula for prismatic joints will be applied
            J[:3, i] = z[i]

    return J

# Damped Least-Squares

def DLS(A, damping):
    '''
        Function computes the damped least-squares (DLS) solution to the matrix inverse problem.

        Arguments:
        A (Numpy array): matrix to be inverted
        damping (double): damping factor

        Returns:
        (Numpy array): inversion of the input matrix
    '''
    x = A @ np.transpose(A)
    # Helping variable for computing product of matrix and its inverse

    DLS = np.transpose(A) @ np.linalg.inv(x + damping**2 * np.identity(x.shape[0]))
    # Implementation of the formula to compute the DLS of matrix A

    return  DLS

def robotPoints3D(T):
    '''
        Function extracts the characteristic points of a kinematic chain in 3D space,
        based on the list of transformations that describe it.

        Arguments:
        T (list of Numpy array): list of transformations along the kinematic chain of the robot (from the base frame)

        Returns:
        (Numpy array): an array of 3D points
    '''
    P = np.zeros((3, len(T)))
    for i in range(len(T)):
        P[:, i] = T[i][0:3, 3]
    return P

def weighted_dls(J, weights, lambda_d):
    """
    Weighted Damped Least Squares solver for computing the joint velocity.
    J: The Jacobian matrix
    weights: The weight vector
    lambda_d: The damping factor
    """
    W = np.diag(weights)
    x = J @ np.linalg.inv(W) @ np.transpose(J)
    v = np.linalg.inv(W) @ np.transpose(J) @ np.linalg.inv(x + lambda_d**2 * np.identity(x.shape[0]))
    # Implementing the formula for weighter DLS

    return v