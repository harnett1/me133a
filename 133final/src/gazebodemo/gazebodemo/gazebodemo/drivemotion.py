'''drivemotion.py

   Force Gazebo to track a trajectory. Used to initially make a 7 or 6 DOF 
   move from Point A to Point B in a convex, parabolic motion, simulating
   a racquet swing.

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

from hw6code.GeneratorNode     import GeneratorNode
from hw6code.KinematicChain    import KinematicChain
from hw5code.TransformHelpers  import *
#
#   COPY THE TRAJECTORY CLASS AND ANY SUPPORTING CODE
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Define known tip/joint positions
        self.x = 0
        self.A_matrix = np.array([[0.0, 0.0, 0.0, 0.0],
                                [2.0, 0.0, 0.0, 0.0]])
        
        self.N = 3 # highest degree of the spline

        # Select the segment duration. Used for looping motion.
        self.T = 2.0
        self.t = 0.0

        # Initialize the current joint location and the matching tip
        # error (which should be zero).
        self.q   = np.array([0, 0.39, 0, 0.72, 0, 0, 0]).reshape(7,1) # arbitrary point
        self.err = 0

        # Pick the convergence bandwidth.
        self.lam = 10


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, tabsolute, dt):
        if tabsolute > self.T: # reset the time for each segment
            return self.evaluate(tabsolute - self.T, dt)
        self.t = tabsolute
        self.A_matrix = np.array([[0.0, -0.6, 0.1, 0.0],
                                [2.0, -0.1, 0.6, 0.2]])
        c_x = self.getSplineCoeffs(self.A_matrix[:, 0], self.A_matrix[:, 1])
        c_y = self.getSplineCoeffs(self.A_matrix[:, 0], self.A_matrix[:, 2])
        c_z = self.getSplineCoeffs(self.A_matrix[:, 0], self.A_matrix[:, 3])
        c_x_vel = [c_x[n] for n in range(self.N+1)]
        c_y_vel = [c_y[n] for n in range(self.N+1)]
        c_z_vel = [c_z[n] for n in range(self.N+1)]
        x_x, x_y, x_z = 0, 0, 0
        v_x, v_y, v_z = 0, 0, 0
        for deg in range(self.N+1):
            x_x += c_x.tolist()[deg] * self.t ** deg
            x_y += c_y.tolist()[deg] * self.t ** deg
            x_z += c_z.tolist()[deg] * self.t ** deg
            if deg != 0:
                v_x += c_x.tolist()[deg] * deg * self.t ** (deg - 1)
                v_y += c_y.tolist()[deg] * deg * self.t ** (deg - 1)
                v_z += c_z.tolist()[deg] * deg * self.t ** (deg - 1)  
        x = np.array([x_x, x_y, x_z]).reshape(3,1)
        self.x = x
        q = self.q
        xdot = np.array([v_x, v_y, v_z]).reshape(3, 1)
        
        wd = np.zeros((3,1))
        Rd = Reye()
        
        # One option for mitigating effect of singularities with secondary task
        #qc    = np.radians(np.array([-45,-45,90,-90,0,0,0]).reshape((7,1)))
        #lams = 10.0
        #qsdot = lams * (qc - q)

        J    = np.vstack((self.chain.Jv(),self.chain.Jw()))
        xdot = np.vstack((xdot, wd))

        Jinv = np.linalg.pinv(J)
        qdot = Jinv @ (xdot + self.lam * self.err) #+ qsdot - Jinv @ (J @ qsdot) # part of secondary task option
        self.qdot = qdot

        q = q + dt * qdot
        self.chain.setjoints(q)
        self.q = q

        self.err  = np.vstack((ep(x, self.chain.ptip()),
                      eR(Rd, self.chain.Rtip())))

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


    def getSplineCoeffs(self, t_vals, coord_vals):
        # Calculates the spline coefficients for a given set of points along a path
        # in the form of s(t) = c_x_n * t^n + c_x_n-1 * t^(n+1) + ... + c_x_0
        # using linear regression to minimize the error, e = s - t * c, for each term
        # being a matrix.
        # The error is minimized by c_x = pinv(t_x) * x, similar for c_y and c_z
        t_matrix = []
        t_vals = t_vals.tolist()
        for t in t_vals:
            row = []
            for deg in range(self.N+1):
                row.append(t ** deg)
            t_matrix.append(row)
        t_matrix = np.array(t_matrix)
        coeffs = np.linalg.pinv(t_matrix) @ np.array(coord_vals)
        return coeffs

#
#   Gazebo Interface Node Class
#
class GazeboInterfaceNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        ##initialize ball positions/velocities
        #self.ballpos = np.array([0, 0, 0]).reshape(-1,1)
        #self.ballvel = np.array([0, 0, 0]).reshape(-1,1)

        ## Create a subscriber to the gazebo state.
        #self.sub_links = self.create_subscription(
        #    LinkStates, '/gazebo/link_states' ,self.cb_links, 10)

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
        desired = self.trajectory.evaluate(t, dt) # would also input self.ballpos
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
