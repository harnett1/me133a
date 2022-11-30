'''drivemotion.py

   Force Gazebo to track a trajectory.

   Node:        /drive
   Subscribe:   /gui/joint_states                 sensor_msgs/JointState
   Subscribe:   /joint_states                     sensor_msgs/JointState
   Publish:     /velocity_controllers/commands    std_msgs/Float64MultiArray

'''

import rclpy
import numpy as np

from rclpy.node         import Node
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Float64MultiArray

from gazebodemo.KinematicChainGravity import *
from gazebodemo.TransformHelpers import *

from hw3code.Segments       import Hold, Stay, GotoCubic, SplineCubic
from hw4code.hw4p3          import fkin, Jac


#
#   Spline Helper
#
#   We could also the Segments module.  But this seemed quicker and easier?
#
def spline(t, T, p0, pf):
    p = p0 + (pf-p0) * (3*t**2/T**2 - 2*t**3/T**3)
    v =      (pf-p0) * (6*t   /T**2 - 6*t**2/T**3)
    return (p, v)

def ikin(self, x_d, q_guess):
    # Return the ikin using the given conditions
    x_guess = fkin(q_guess)
    dx = x_d - x_guess
    J = Jac(q_guess)
    # If the position is within an arbitrarily small threshold, we return q_guess
    if np.abs(dx[0][0]) < 10**-6 and np.abs(dx[1][0]) < 10**-6 and np.abs(dx[2][0]) < 10**-6:
       return np.array(q_guess)
    # Else, update theta_guess and run it again with updated angles
    self.theta_guess = q_guess
    return self.ikin(x_d, q_guess + np.linalg.inv(J) @ dx)

#
#   COPY THE TRAJECTORY CLASS AND ANY SUPPORTING CODE
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Define the various points.
        self.q0 = np.radians(np.array([0, 90, 0, -90, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0, 0]).reshape((-1,1))
        self.R0 = Reye()

        self.ptop  = np.array([ 0.0, 0.50, 1.0]).reshape((-1,1))
        self.Rleft  = Rotx(-np.pi/2) @ Roty(-np.pi/2)
        self.Rleft  = Rotz( np.pi/2) @ Rotx(-np.pi/2)
        self.Rtop = Reye()

        # Initialize the current joint position and chain data.
        self.q = self.q0
        self.chain.setjoints(self.q)

        # Also zero the task error.
        self.err = np.zeros((6,1))

        # Pick the convergence bandwidth.
        self.lam = 20
        self.lam_sec = 0.1

        self.tlocal = 0

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluate at the given time.
    def evaluate(self, t, dt):
        if t > 5:
            self.tlocal = 0
            self.q = self.q0
        else:
            self.tlocal = t
        s    =               np.cos(np.pi/4 * (self.tlocal-3))
        sdot = - np.pi/4 * np.sin(np.pi/4 * (self.tlocal-3))

        # Use the path variables to compute the position trajectory.
        pd = np.array([-0.3*s    , 0.5, 0.75-0.6*s**2  ]).reshape((3,1))
        vd = np.array([-0.3*sdot , 0.0,     -1.2*s*sdot]).reshape((3,1))

        alpha    = - np.pi/3 * (s-1)
        alphadot = - np.pi/3 * sdot

        eleft = np.array([1, 1, -1]).reshape((3,1)) / np.sqrt(3)
        Rd    = Rote(eleft, -alpha)
        wd    = - eleft * alphadot


        # Grab the last joint value and task error.
        q   = self.q
        err = self.err

        # Compute the inverse kinematics
        # J    = np.vstack((self.chain.Jv(),self.chain.Jw(), np.array([0.5, 0, 1, 0, 0, 0, 0]).reshape((1, 7)))) # added last line in (b)
        J    = np.vstack((self.chain.Jv(),self.chain.Jw())) # added last line in (b)
        xdot = np.vstack((vd, wd)) # added 0 in (b)
        # for (a)
        qdot = np.linalg.pinv(J) @ (xdot + self.lam * err) # changed inv-->pinv in (a)
        # for (c)
        # qdot_sec = np.array([-self.q[0]-np.pi/4, -self.q[1]-np.pi/4,-self.q[2]+np.pi/2, -self.q[3]-np.pi/2, -self.q[4], -self.q[5], -self.q[6]]).reshape((7, 1))
        # for part (d)
        # qdot_sec = (1/(self.q[0]**2 + self.q[1]**2)) * np.array([self.q[0][0], max(abs(self.q[0]), self.q[1])[0], 0, 0, 0, 0, 0]).reshape(-1, 1)

        # for parts (c) and (d)
        # qdot = np.linalg.pinv(J) @ (xdot + self.lam * self.err) + (np.eye(7) - np.linalg.pinv(J) @ J) @ (self.lam_sec * qdot_sec)

        # Integrate the joint position and update the kin chain data.
        q = q + dt * qdot
        self.chain.setjoints(q)

        # Compute the resulting task error (to be used next cycle).
        # err  = np.vstack((ep(pd, self.chain.ptip()), eR(Rd, self.chain.Rtip()), np.array([-0.5*self.q[0] - self.q[2]]))) # added last line in (b)
        err  = np.vstack((ep(pd, self.chain.ptip()), eR(Rd, self.chain.Rtip()))) # added last line in (b)

        # Save the joint value and task error for the next cycle.
        self.q   = q
        self.err = err

        # Return the position and velocity as python lists.
        return (self.q.flatten().tolist(), qdot.flatten().tolist())


#
#   Gazebo Interface Node Class
#
class GazeboInterfaceNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Set up the trajectory.
        self.trajectory = Trajectory(self)
        
        # Set up the list of expected joint names and associated numbers.
        self.joints = {'theta1':0,
                       'theta2':1,
                       'theta3':2,
                       'theta4':3,
                       'theta5':4,
                       'theta6':5,
                       'theta7':6}
        self.dofs = len(self.joints)

        # Clear the actual positions and time (as read from Gazebo)
        self.q = [0.0] * self.dofs
        self.t = 0.0

        # Add a publisher to send the joint commands.
        self.pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)

        # And, as the last item, add a subscriber to listen to Gazebo
        # for actual positions.
        self.sub_gazebo = self.create_subscription(
            JointState, '/joint_states' ,self.cb_gazebo, 10)

        # Report.
        self.get_logger().info("Tracking the trajectory...")


    # Callback from Gazebo, which sends actual positions.
    def cb_gazebo(self, msg):
        # Extract the information from the joint states message.  Note
        # the message may order the joints however it likes.  So we
        # check the joint names to determine each dof number.
        for i in range(len(msg.position)):
            self.q[self.joints[msg.name[i]]] = msg.position[i]

        # Also pull out the current time and time step (limit 0<dt<0.1)
        t  = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        dt = t - self.t
        dt = min(0.1, max(0, dt))
        self.t = t

        # Compute the desired joint positions and velocities for this time.
        desired = self.trajectory.evaluate(t, dt)
        if desired is None:
            # If the trajectory has ended, hold where it is!
            qd    = self.q
            qddot = [0.0] * self.dofs
        else:
            (qd, qddot) = desired

        # Compute the command velocties, to correct any position mismatch.
        lam   = 10.0
        qrdot = [0.0] * self.dofs
        for i in range(self.dofs):
            qrdot[i] = qddot[i] + lam * (qd[i] - self.q[i])

        # Build up and send a command message.
        velmsg = Float64MultiArray()
        velmsg.data = qrdot
        self.pub.publish(velmsg)



#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the interface node (100Hz).
    rclpy.init(args=args)
    node = GazeboInterfaceNode('drive')

    # Spin, until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
