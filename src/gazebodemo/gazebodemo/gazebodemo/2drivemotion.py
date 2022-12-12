'''drivemotion.py

   Force Gazebo to track a trajectory.

   Node:        /drive
   Subscribe:   /gui/joint_states                 sensor_msgs/JointState
   Subscribe:   /joint_states                     sensor_msgs/JointState
   Publish:     /velocity_controllers/commands    std_msgs/Float64MultiArray

'''

import rclpy
import numpy as np

from hw6code.GeneratorNode     import GeneratorNode
from hw6code.KinematicChain    import KinematicChain
from hw5code.TransformHelpers  import *

from rclpy.node         import Node
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Float64MultiArray


#
#   Spline Helper
#
#   We could also the Segments module.  But this seemed quicker and easier?
#
def spline(t, T, p0, pf):
    p = p0 + (pf-p0) * (3*t**2/T**2 - 2*t**3/T**3)
    v =      (pf-p0) * (6*t   /T**2 - 6*t**2/T**3)
    return (p, v)


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):

        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Define the various points.
        self.q0 = np.radians(np.array([0, -90, -90, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0, -0.5, 0.5]).reshape((-1,1))
        self.p1 = np.array([0.5, 0, 0.5]).reshape((-1,1))
        self.p2 = np.array([0, 0.5, 0.5]).reshape((-1,1))
        self.p3 = np.array([0.5, 0, 0.5]).reshape((-1,1))
        self.R0 = Reye()

        # Initialize the current joint position and chain data.
        self.q = self.q0
        self.chain.setjoints(self.q)

        # Also zero the task error.
        self.err = np.zeros((6,1))

        # Pick the convergence bandwidth.
        self.lam = 20


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):

        # Decide which phase we are in:
        #if t < 3:
            # Approach movement:
            #(s0, s0dot) = spline(t, 3, 0, 1)

            #pd = self.p0 + (self.p_end - self.p0) * s0
            #vd =           (self.p_end - self.p0) * s0dot

            #Rd = Rotz(np.pi/2 * s0) @ Rotx(-np.pi/2 * s0)
            #wd = ez() * (np.pi/2 * s0dot) * ex() * (-np.pi/2 * s0dot)

        #else:
        # Cyclic movements, after the first 0s, repeat every 1s.
        #t1 = (t) % 2.0
        t1 = (t) % 1.0

        # Pre-compute the path variables.  To show different
        # options, we split into position/orientation path
        # variable and compute using both a sinusoid and splines.
        if t1 < 0.25:
            (sR, sRdot) = spline(t1, 0.25, 0,  1)
            pd = self.p1 + (self.p0 - self.p1) * sR
            vd = (self.p0 - self.p1) * sRdot

            Rd = Reye() #Rotz(np.pi/2) @ Rotx(-np.pi/2)
            wd = ex() * 0 #(-np.pi/2)
        
        elif t1 < 0.5:
            (sR, sRdot) = spline(t1 - 0.25, 0.25, 0,  1)
            pd = self.p2 + (self.p1 - self.p2) * sR
            vd =          (self.p1 - self.p2) * sRdot

            Rd = Reye() #Rotz(np.pi/2) @ Rotx(-np.pi/2)
            wd = ex() * 0 #(-np.pi/2)

        elif t1 < 0.75:
            (sR, sRdot) = spline(t1 - 0.5, 0.25, 0,  1)
            pd = self.p3 + (self.p2 - self.p3) * sR
            vd =          (self.p2 - self.p3) * sRdot

            Rd = Reye() # @ Rotx(-np.pi/2)
            wd = ex() * 0 #(-np.pi/2)

        else:
            (sR, sRdot) = spline(t1 - 0.75, 0.25, 0,  1)
            pd = self.p0 + (self.p3 - self.p0) * sR
            vd =          (self.p3 - self.p0) * sRdot

            Rd = Reye() #Rotz(np.pi/2) @ Rotx(-np.pi/2)
            wd = ex() * 0 #(-np.pi/2)

        q = self.q
        j = np.linalg.pinv(np.vstack((self.chain.Jw(), self.chain.Jv()))) # stack Jw and Jv
        qdot = j @ (np.vstack((wd,vd)) + self.lam * self.err) # err should be 6x1 # stack wd and vd
        
        q = q + dt * qdot

        # Store stuff we'll need next cycle.
        self.err = np.vstack((eR(Rd, self.chain.Rtip()), ep(pd, self.chain.ptip()))) #stack eR and ep
        self.q = q
        self.Rd = Rd
        self.chain.setjoints(self.q) 

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


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
                       'theta6':5}
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