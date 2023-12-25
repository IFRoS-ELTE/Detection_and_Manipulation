from lab2_robotics import * # Includes numpy import
import math

def jacobianLink(T, revolute, link): # Needed in Exercise 2

    J = np.zeros((6, len(revolute)))

    O = [T[i][0:3, 3] for i in range(link+1)]
    z = [T[i][0:3, 2] for i in range(link+1)]
    for i in range(link):
        if revolute[i]:
            J[:3, i] = np.cross(z[i], O[-1] - O[i])
            J[3:, i] = z[i]
        else:
            J[:3, i] = z[i]
    return J
    '''
        Function builds a Jacobian for the end-effector of a robot,
        described by a list of kinematic transformations and a list of joint types.

        Arguments:
        T (list of Numpy array): list of transformations along the kinematic chain of the robot (from the base frame)
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint
        link(integer): index of the link for which the Jacobian is computed

        Returns:
        (Numpy array): end-effector Jacobian
    '''

'''
    Class representing a robotic manipulator.
'''
class Manipulator:
    '''
        Constructor.

        Arguments:
        d (Numpy array): list of displacements along Z-axis
        theta (Numpy array): list of rotations around Z-axis
        a (Numpy array): list of displacements along X-axis
        alpha (Numpy array): list of rotations around X-axis
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint
    '''
    def __init__(self, d, theta, a, alpha, revolute):
        self.d = d
        self.theta = theta
        self.a = a
        self.alpha = alpha
        self.revolute = revolute
        self.dof = len(self.revolute)
        self.q = np.zeros(self.dof).reshape(-1, 1)
        self.update(0.0, 0.0)

    '''
        Method that updates the state of the robot.

        Arguments:
        dq (Numpy array): a column vector of joint velocities
        dt (double): sampling time
    '''
    def update(self, dq, dt):
        self.q += dq * dt
        for i in range(len(self.revolute)):
            if self.revolute[i]:
                self.theta[i] = self.q[i]
            else:
                self.d[i] = self.q[i]
        self.T = kinematics(self.d, self.theta, self.a, self.alpha)

    def update_T(self, q):
        self.q = q
        for i in range(len(self.revolute)):
            if self.revolute[i]:
                self.theta[i] = self.q[i]
            else:
                self.d[i] = self.q[i]
        self.T = kinematics(self.d, self.theta, self.a, self.alpha)


    ''' 
        Method that returns the characteristic points of the robot.
    '''
    def drawing(self):
        return robotPoints2D(self.T)

    '''
        Method that returns the end-effector Jacobian.
    '''
    def getEEJacobian(self):
        return jacobian(self.T, self.revolute)

    '''
        Method that returns the end-effector transformation.
    '''
    def getEETransform(self):
        return self.T[-1]

    '''
        Method that returns the position of a selected joint.

        Argument:
        joint (integer): index of the joint

        Returns:
        (double): position of the joint
    '''
    def getJointPos(self, joint):
        return self.q[joint]

    '''
        Method that returns number of DOF of the manipulator.
    '''
    def getDOF(self):
        return self.dof
    
    def getSelectedTransf(self, link):
        return self.T[link]
    
    def setT(self, T):
        self.T = T
    # Function that returns the transformation from the selected link frame to the base frame
    
    def getSelectedJacob(self, link):
        a = jacobianLink(self.T, self.revolute, link)
        return a
    # Function that returns Jacobian vector of a specified link

'''
    Base class representing an abstract Task.
'''
class Task:
    '''
        Constructor.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired sigma (goal)
    '''
    def __init__(self, name, desired):
        self.name = name # task title
        self.sigma_d = desired # desired sigma
        self.ar = False
        self.act = 0
        
        
    '''
        Method updating the task variables (abstract).

        Arguments:
        robot (object of class Manipulator): reference to the manipulator
    '''
    def update(self, robot):
        pass

    ''' 
        Method setting the desired sigma.

        Arguments:
        value(Numpy array): value of the desired sigma (goal)
    '''
    def setDesired(self, value):
        self.sigma_d = value

    '''
        Method returning the desired sigma.
    '''
    def getDesired(self):
        return self.sigma_d

    '''
        Method returning the task Jacobian.
    '''
    def getJacobian(self):
        return self.J

    '''
        Method returning the task error (tilde sigma).
    '''    
    def getError(self):
        return self.err

    def setFvv(self, value):
        self.fvv = value
    # Function for setting the feedforward velocity

    def setK(self, value):
        self.K = value
    # Function for seeting the parameter K

    def getFvv(self):
        return self.fvv
    # Function for getting the feedforward velocity
    
    def getK(self):
        return self.K
    # Function for geeting the parameter K

    def is_active(self):
        return self.ar
    # Function for getting the obstacle activation function value
    
    def joint_active(self):
        return self.act
    # Funtion for getting the joint limit activation function value

'''
    Subclass of Task, representing the 2D position task.
'''
class Position2D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)

        #self.fvv = (np.zeros((2, 1)))
        # Initializing feedforward velocity with zeros of the acceptable dimension
        #self.K = (np.eye(2))
        # Initializing parameter K with identity matrix of a desirable dimension
        self.link = link
        
    def update(self, robot):
        self.J = robot.getSelectedJacob(self.link)[0:3, :]
        # Update task Jacobian, taking into account only x and y coordinates for the position
        sigma = robot.getSelectedTransf(self.link)[0:3, 3].reshape(3, 1)
        self.err = (self.sigma_d.reshape(3, 1) - sigma).reshape(3, 1)# Update task error
        self.ar = True
        self.act = 1
        
'''
    Subclass of Task, representing the 2D orientation task.
'''
class Orientation2D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.fvv = (np.zeros((1, 1)))
        # Initializing feedforward velocity with zeros of the acceptable dimension
        self.K = (np.eye(1))
        # Initializing parameter K with identity matrix of a desirable dimension
        self.link = link
        
    def update(self, robot):
        sigma = np.array([[0],
                          [0],
                          [0]])
        self.J = robot.getSelectedJacob(self.link)[3:6, :].reshape(3, robot.getDOF())
        # Update task Jacobian, taking into account only rotation around z axis since the robot is in x-y plane
        sigma[2] = np.arctan2(robot.getSelectedTransf(self.link)[1, 0], robot.getSelectedTransf(self.link)[1, 1])
        sigma[1] = np.arctan2(-robot.getSelectedTransf(self.link)[2, 0], np.sqrt(robot.getSelectedTransf(self.link)[2, 1]**2 +robot.getSelectedTransf(self.link)[2, 2]**2 ))
        sigma[0] = np.arctan2(robot.getSelectedTransf(self.link)[2, 1], robot.getSelectedTransf(self.link)[2, 2])
        self.err = self.sigma_d.reshape(3,1) - sigma.reshape(3,1) # Update task error
        
'''
    Subclass of Task, representing the 2D configuration task.
'''
class Configuration2D(Task):
    def __init__(self, name, desired, link):
        super()._init_(name, desired)
        self.fvv = (np.zeros((3, 1)))
        # Initializing feedforward velocity with zeros of the acceptable dimension
        self.K = (np.eye(3))
        # Initializing parameter K with identity matrix of a desirable dimension
        self.link = link
        
    def update(self, robot):
        a = robot.getSelectedJacob(self.link)[0:2, :]
        b = robot.getSelectedJacob(self.link)[5, :].reshape(1, robot.getDOF())
        self.J = np.concatenate([a, b])
        # Combining necessary data for both position and orientation
        s1 = (robot.getSelectedTransf(self.link)[0:2, 3]).reshape(2, 1)
        s2 = np.arctan2(robot.getSelectedTransf(self.link)[1, 0], robot.getSelectedTransf(self.link)[1, 1])
        sigma = np.vstack([s1, s2])
        # Combining the both errors of position and orientation with function vstack since it is not possible to perform desired operation with function concatenate
        self.err = self.sigma_d - sigma # Update task error

''' 
    Subclass of Task, representing the joint position task.
'''
class JointPosition(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.fvv = (np.zeros((1, 1)))
        # Initializing feedforward velocity with zeros of the acceptable dimension
        self.K = (np.eye(1))
        # Initializing parameter K with identity matrix of a desirable dimension
        self.link = link
        
    def update(self, robot):
        self.J = np.zeros((1, robot.getDOF()))
        self.J[0, self.link] = 1
        sigma = robot.getJointPos(self.link)
        self.err = self.sigma_d - sigma# Update task error

class Obstacle2D(Task):
        def __init__(self, name, pose, thresh):
            super().__init__(name, pose)
            self.pose = pose
            # Initializing the pose of the centre of the obstacle
            self.ralpha =  thresh[0]
            self.rdelta = thresh[1]
            # Assigning values to delta and alpha

        def update(self, robot):
            self.J = robot.getEEJacobian()[0:2, :]
            # Update task Jacobian, taking into account only x and y coordinates for the position
            sigma = robot.getEETransform()[0:2, 3].reshape(2, 1)
            self.err = (sigma - self.sigma_d.reshape(2, 1))/(np.linalg.norm(sigma - self.sigma_d).reshape(1, 1))# Update task error
            if self.ar == True and np.linalg.norm(sigma - self.sigma_d) >= self.rdelta:
                self.ar = False
            elif self.ar == False and np.linalg.norm(sigma - self.sigma_d) <= self.ralpha:
                self.ar = True
            # Implementing the activation function

class JointLimit(Task):
        def __init__(self, name, thresh, min_angle, max_angle, link):
            super().__init__(name, thresh)
            self.min_angle = min_angle
            self.max_angle = max_angle
            # Assigning the value to the maximal and minimal possible angle
            self.ralpha =  thresh[0]
            self.rdelta = thresh[1]
            # Assigning the value to delta and alpha
            self.link = link
            self.act = 0
            # Initializing activation indicator to 0
            # Defining elements of this subclass

        def update(self, robot):
            self.J = robot.getSelectedJacob(self.link)[5, :].reshape(1, robot.getDOF())
            # Update task Jacobian, taking into account only rotation around z axis since the robot is in x-y plane
            qi = np.arctan2(robot.getSelectedTransf(self.link)[1, 0], robot.getSelectedTransf(self.link)[1, 1])
            # Initializing the angle of the specified part of the manipulator
            if self.act == 0 and qi >= (self.max_angle - self.ralpha):
                self.act = -1
            elif self.act == 0 and qi <= (self.min_angle + self.ralpha):
                self.act = 1
            elif self.act == -1 and qi <= (self.max_angle - self.rdelta):
                self.act = 0
            elif self.act == 1 and qi >= (self.min_angle + self.rdelta):
                self.act = 0
            # Implementing the activation function
            self.err = self.act