import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from lab4_robotics import * 
import numpy as np
from random import uniform
from geometry_msgs.msg import Pose, PoseStamped 
from control_msgs.msg import GripperCommandActionGoal


class XArmControlNode:
    def __init__(self):

        # Robot model - 6-link manipulator
        self.d = np.array([267, 0, 0, 342.5, 0, 277])
        self.q = np.array([0, -1.3849179, 1.3849179, 0, 0, 0])
        self.alpha = np.array([-math.pi/2, 0, -math.pi/2, math.pi/2, - math.pi/2, 0])
        self.a = np.array([0, 289.48866, 77.5, 0, 76, 0])
        self.x=1 
             
        self.revolute = [True, True, True, True, True, True]
        self.sigma_d = np.array([0, 0, 0])
        self.sigma_d2 = np.array([0, 0, 100])
        self.robot = Manipulator(self.d, self.q, self.a, self.alpha, self.revolute)

        # Task hierarchy definition
        self.tasks = [ 
            Position2D("End-effector position", self.sigma_d.reshape(3,1), 6),
            Position2D("End-effector position", self.sigma_d2.reshape(3,1), 5)
            #Orientation2D("End-effector orientation", np.array([0,0,0]).reshape(3,1), 4)
        ] 

        # Simulation params
        self.dt = 0.3
        self.joint_pub = rospy.Publisher('/xarm/xarm6_traj_controller/command', JointTrajectory, queue_size=10)
        self.gripper_pub = rospy.Publisher('/xarm/xarm_gripper/gripper_action/goal', GripperCommandActionGoal, queue_size=10)
        rospy.Subscriber('ball_positions', PoseStamped, self.ball_position_callback)

        # Subscriber for joint states
        rospy.Subscriber('/xarm/joint_states', JointState, self.joint_states_callback)
    
    
    def ball_position_callback(self, msg):
        if (msg.pose.position.x is not None) and (self.x == 1):
            goal = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            T_EE = self.robot.getEETransform()
            # T_EE[:3, -1] = T_EE[:3, -1] /1000
            goal.append(1)
            final_goal = np.array(goal).reshape(4, 1)
            print('TEE = ', T_EE)
            
            final_goal = T_EE@final_goal
            
            # final_goal = np.array([final_goal[0, 0], final_goal[1,0], final_goal[2,0]])#1000
            final_goal = np.array([final_goal[0, 0], final_goal[1,0], -402])
            if self.check_goal(final_goal):
                # Update robot's goal pose
                self.update_task(final_goal)
                self.x = 0
                print ('final_goal = ',final_goal)
                print("Goal position is valid")
            else:
                print ('final_goal = ',final_goal)
                print('Goal position is out of range')
                self.x = 2

    def joint_states_callback(self, data):
        # Assuming the joint states are in the same order as the DH parameters
        self.q = np.array([0, -1.3849179, 1.3849179, 0, 0, 0])
        current_joint_positions = np.array(data.position[0:6] + self.q)
        
    
        self.abc = data.position[0:6]  
        T_EE = self.robot.getEETransform()

        #COMMENT THE BELOW 2 LINES TO MOVE THE ROBOT TO THE BALL POSITION
        # if self.x == 0 and self.check_goal(T_EE[0:3,-1]):
        #     self.move(current_joint_positions)

    def update_task(self, goal):
        self.sigma_d = goal
        self.sigma_d2 = np.array([goal[0], goal[1], goal[2] + 180])
        self.tasks = [
            Position2D("End-effector position", self.sigma_d.reshape(3, 1), 6)
            #Position2D("End-effector position", self.sigma_d2.reshape(3, 1), 5)
        ]

    def move(self, current_joint_positions):
        self.robot.update_T(current_joint_positions)
        dof = self.robot.getDOF()

        # Recursive Task-Priority algorithm
        P = np.eye(dof)
        dq = np.zeros((dof, 1))

        for i in range(len(self.tasks)):
            self.tasks[i].update(self.robot)
            Jbar = self.tasks[i].getJacobian() @ P
            dq = dq + DLS(Jbar, 0.2) @ (self.tasks[i].getError() - (self.tasks[i].getJacobian() @ dq))
            P = P - np.linalg.pinv(Jbar) @ Jbar

        # Update robot
        self.abc += self.dt * dq.flatten()

        # Publish joint velocities
        vel_msg = JointTrajectory()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = self.abc.tolist()
        point.time_from_start = rospy.Duration(1)
        vel_msg.points.append(point)
        T_EE = self.robot.getEETransform()
        print(T_EE[0:3,-1])
        if self.distance([self.sigma_d[0], self.sigma_d[1], self.sigma_d[2]], T_EE[0:3, -1]) > 5:
            self.joint_pub.publish(vel_msg)
            print("Moving ...")
        else:
            print("Position reached")

            gripper_goal = GripperCommandActionGoal()
            self.gripper_pub.publish(gripper_goal)
            rospy.signal_shutdown("User requested shutdown")

    def check_goal(self, goal):
        if 630 >goal[0] > 20 and 700 > goal[2] > -420:
            return True

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('xarm_control_node', anonymous=True)
    xarm_control_node = XArmControlNode()
    xarm_control_node.run()

        


    
