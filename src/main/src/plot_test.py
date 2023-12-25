from lab4_robotics import * # Includes numpy import
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
import random
from random import uniform

# Robot model - 6-link manipulator
d = np.array([267, 0, 0, 342.5, 0, 97])
q = np.array([0, -1.3849179, 1.3849179, 0, 0, 0])
alpha = np.array([-math.pi/2, 0, -math.pi/2, math.pi/2, - math.pi/2, 0])
a = np.array([0, 289.48866, 77.5, 0, 76, 0])
        
revolute = [True, True, True, True, True, True]
sigma_d = np.array([0, 0, 0])
robot = Manipulator(d, q, a, alpha, revolute)

# Task hierarchy definition
tasks = [ 
            Position2D("End-effector position", sigma_d.reshape(3,1), 6),
            Orientation2D("End-effector orientation", np.array([0,0,-1.384]).reshape(3,1), 2)
        ] 
# Different types of tasks are defined, uncomment to use

# Simulation params
dt = 1.0/60.0

# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d', autoscale_on=False)
ax.set_xlim([-400, 400])
ax.set_ylim([-400, 400])
ax.set_zlim([-400, 400])
ax.set_title('Simulation')
ax.set_aspect('equal')
ax.grid()
line, = ax.plot([], [], 'o-', lw=2) # Robot structure
path, = ax.plot([], [], 'c-', lw=1) # End-effector path
point, = ax.plot([], [], 'rx') # Target
PPx = []
PPy = []
PPz = []

#tasks[0].setK(np.diag([2, 2]))
#tasks[0].setK(np.diag([0.9, 0.9]))
#tasks[0].setK(np.diag([4, 0.6]))
# Setting K to different values
#tasks[0].setFvv(np.array([0.01]))
# Setting feedforward velocity
l = len(tasks)
# Number of tasks

# time = []
# error1 = []
# error2 = []
# error3 = []
# error4 = []
# error5 = []
# error6 = []
# Arrays added for the axes intervals to be stored

# Simulation initialization
def init():
    global tasks
    #tasks[0].setDesired(np.array([uniform(-2, 2), uniform(-2, 2)]))
    # Defining tasks[0] again so the simulation can be repeated for each new goal point
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])
    return line, path, point

# Simulation loop
def simulate(t):
    global tasks
    global robot
    global d, q, a, alpha, revolute, sigma_d
    global PPx, PPy, PPz
    T = kinematics(d, q, a, alpha)
    dof = robot.getDOF()
    # Getting the number of degrees of freedom of the manipulator 
    print(len(T))
    # Recursive Task-Priority algorithm
    P = np.eye(dof)
    # Initialize null-space projector
    dq = np.zeros((dof, 1))
    # Initialize output vector (joint velocity)

    for i in range(len(tasks)):
        tasks[i].update(robot)
        # Update task state
        Jbar = tasks[i].getJacobian() @ P
        # Compute augmented Jacobian
        #var = tasks[i].getFvv() + tasks[i].getK() @ tasks[i].getError()
        # Adding the variable in which feedforward velocity and matrix K are figuring
        dq = dq + DLS(Jbar, 0.5) @ (tasks[i].getError() - (tasks[i].getJacobian() @ dq))
        # Task velocity
        P = P - np.linalg.pinv(Jbar) @ Jbar
        # Update null-space projector
    
    # Update robot
    robot.update(dq, dt)
    
    # Update drawing
    
    P = robotPoints3D(T)
    line.set_data(P[0, :], P[1, :])
    line.set_3d_properties(P[2, :])  # Set z-coordinates
    PPx.append(P[0,-1])
    PPy.append(P[1,-1])
    PPz.append(P[2,-1])
    path.set_data(PPx, PPy)
    path.set_3d_properties(PPz)
    point.set_data(sigma_d[0], sigma_d[1])
    point.set_3d_properties(sigma_d[2]) 
    
    
    return line, path, point

# Run simulation
animation = anim.FuncAnimation(fig, simulate, np.arange(0, 10, dt), 
                                interval=10, blit=True, init_func=init, repeat=True)

plt.show()
# Argument repeat is initialized as True since we want to repeat the simulation multiple times
