'''trackgui.py

   Updated version of drivemotion.py. Tracks the ball position and
   uses that information in Trajectory.evaluate() to generate the
   spline.

   Node:        /test
   Subscribe:   /gui/joint_states                 sensor_msgs/JointState
   Subscribe:   /joint_states                     sensor_msgs/JointState
   Publish:     /velocity_controllers/commands    std_msgs/Float64MultiArray
'''

import rclpy
import numpy as np

from rclpy.node         import Node
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Float64MultiArray
from gazebo_msgs.msg    import LinkStates

from hw6code.GeneratorNode     import GeneratorNode
# from hw4code.hw4p3      import fkin, Jac # from a failed attempt
from hw6code.KinematicChain    import KinematicChain
from hw5code.TransformHelpers  import *


class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())
        # Define known tip/joint positions
        self.qA = np.array([0, -0.39, 0, 0.73, 0, 0, 0]).reshape(7,1)
       
        # Pre-compute what we can.
        self.x = 0
        self.tA = 0.0
        self.tB = 3.0
        self.t_matrix = np.array([self.tA, self.tB])
        self.A_matrix = np.array([[0.0, -0.6, 0.1, 0.0],
                                [1.0, 0.0, 0.0, 0.0]])
        
        # Highest degree of the spline
        self.N = 3
        
        # qB will be determined by the ikin at runtime...

        # Select the segment duration.
        self.T = 1.0

        # Initialize the current joint location and the matching tip
        # error (which should be zero).
        self.q   = self.qA
        self.err = 0

        # Pick the convergence bandwidth.
        self.lam = 10

        self.t = 0.0


 
    def jointnames(self):
        '''
        Declare joint names.
        '''
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']



    def evaluate(self, tabsolute, dt, ballpos, slow, hit):
        '''
        tabsolute: current time
        dt: time of each tick or run
        ballpos: position of ball in current time tick
        slow: flag that indicates if ball is within 0.5m to 1m from the origin (True)
        hit: flag that indicates if ball is within 0.5m from the origin (True)
        Evaluate at the given time.  This was last called (dt) ago.
        Creates splines from current arm position to position of ball, calculates final joint positions
        using inverse kinematics and sets the joints to those positions.
        Returns joint position and velocity
        '''
        # Reset the time for each segment
        if tabsolute > self.T: 
            return self.evaluate(tabsolute - self.T, dt, ballpos, slow, hit)

        self.t = tabsolute

        # Two-point distance approach was not accurate and resulted in jittery motion
        # pos = fkin(self.q)
        # if slow:
        #     self.A_matrix = np.array([[self.t, pos[0][0], pos[1][0], pos[2][0]],
        #                             [self.t + 0.5, ballpos[0], ballpos[1], ballpos[2]]]) # should be self.t + 2.0
        # if hit:
        #     self.A_matrix = np.array([[self.t, pos[0][0], pos[1][0], pos[2][0]],
        #                             [self.t + 0.5, ballpos[0], ballpos[1], ballpos[2]]])

        # Reset position to ball position in 0.25 seconds
        self.A_matrix = np.array([[0, -0.6, 0.1, 0.0],
                                [0.25, ballpos[0], ballpos[1], ballpos[2]]])

        # Spline coefficients from polynomial interpolation
        c_x = self.getSplineCoeffs(self.A_matrix[:, 0], self.A_matrix[:, 1])
        c_y = self.getSplineCoeffs(self.A_matrix[:, 0], self.A_matrix[:, 2])
        c_z = self.getSplineCoeffs(self.A_matrix[:, 0], self.A_matrix[:, 3])
        c_x_vel = [c_x[n] for n in range(self.N+1)]
        c_y_vel = [c_y[n] for n in range(self.N+1)]
        c_z_vel = [c_z[n] for n in range(self.N+1)]
        x_x, x_y, x_z = 0, 0, 0
        v_x, v_y, v_z = 0, 0, 0
        # Create the spline polynomials in the x,y, and z directions and the respective derivative velocity splines
        for deg in range(self.N+1):
            x_x += c_x.tolist()[deg] * self.t ** deg
            x_y += c_y.tolist()[deg] * self.t ** deg
            x_z += c_z.tolist()[deg] * self.t ** deg
            if deg != 0:
                v_x += c_x.tolist()[deg] * deg * self.t ** (deg - 1)
                v_y += c_y.tolist()[deg] * deg * self.t ** (deg - 1)
                v_z += c_z.tolist()[deg] * deg * self.t ** (deg - 1)
     
        # Below line is Newton-Raphson inverse kinematics implementation
        # q = self.ikin(x, self.q) # add velocity to x if running this
        
        q = self.q
        x = np.array([x_x, x_y, x_z]).reshape(3,1)
        xdot = np.array([v_x, v_y, v_z]).reshape(3, 1)
        self.x = x

        wd = np.zeros((3,1))
        Rd = Reye()

        # Create the Jacobian
        J    = np.vstack((self.chain.Jv(),self.chain.Jw()))
        xdot = np.vstack((xdot, wd))

        # Calculate the weighted Jacobian inverse
        gamma = 0.1
        Jinv = J.T @ np.linalg.pinv(J @ J.T + gamma**2 * np.eye(6))
        # Originally tried a pseudo-inverse for the Jacobian inverse
        # Jinv = np.linalg.pinv(J)

        # Inverse kinematics
        qdot = Jinv @ (xdot + self.lam * self.err)
        self.qdot = qdot
        q = q + dt * qdot
        self.chain.setjoints(q)
        self.q = q
     
        self.err  = np.vstack((ep(x, self.chain.ptip()),
                      eR(Rd, self.chain.Rtip())))

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


    def getSplineCoeffs(self, t_vals, coord_vals):
        '''
        t_vals: time points from A matrix
        coord_vals: start and end coordinates
        Calculates the spline coefficients for a given set of points along a path
        in the form of s(t) = c_x_n * t^n + c_x_n-1 * t^(n+1) + ... + c_x_0
        using linear regression to minimize the error, e = s - t * c, for each term
        being a matrix.
        The error is minimized by c_x = pinv(t_x) * x, similar for c_y and c_z
        Returns list of coefficienct for spline
        '''
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

        # Initialize ball positions/velocities
        self.ballpos = np.array([0, 0, 0]).reshape(-1,1)
        self.ballvel = np.array([0, 0, 0]).reshape(-1,1)

        # Create a subscriber to the gazebo state.
        self.sub_links = self.create_subscription(
            LinkStates, '/gazebo/link_states' ,self.cb_links, 10)

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
        self.lastcalc = 0.0

        # Indicates when to start creating slow spline
        self.slow = False
        # Indicates when ball is close enough to hit --> speed up spline
        self.hit = False

        # Add a publisher to send the joint commands
        self.pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)

        # And, as the last item, add a subscriber to listen to Gazebo for actual positions
        self.sub_gazebo = self.create_subscription(
            JointState, '/joint_states' ,self.cb_gazebo, 10)

        # Report
        self.get_logger().info("Tracking the trajectory...")



    def cb_gazebo(self, msg):
        '''
        Callback from Gazebo, which sends actual positions.
        Extract the information from the joint states message.  Note
        the message may order the joints however it likes.  So we
        check the joint names to determine each dof number.
        '''
        for i in range(len(msg.position)):
            self.q[self.joints[msg.name[i]]] = msg.position[i]

        # Also pull out the current time and time step (limit 0<dt<0.1)
        t  = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        dt = t - self.t
        dt = min(0.1, max(0, dt))
        self.t = t

        # Distance between ball and origin
        d = np.sqrt(self.ballpos[0][0]**2 + self.ballpos[1][0]**2 + self.ballpos[2][0]**2)
   
        # If ball farther than 1 meter from origin, freeze the joints
        if d > 1.0:
            qd    = self.q
            qddot = [0.0] * self.dofs
        # The approach below proved to be unreliable so it was overwritten in evaluate with a reset position
        # If ball is within 0.5m-1m, start a slow spline, if it is within 0.5m then start a fast spline
        else:
            if d <= 1.0 and d >= 0.5:
                self.slow = True
                self.hit = False
            else:
                self.slow = False
                self.hit = True
            # Compute the desired joint positions and velocities for this time.
            desired = self.trajectory.evaluate(t, dt, self.ballpos, self.slow, self.hit)
            self.lastcalc = t
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
    def cb_links(self, msg):
        '''
        Receive the link states from Gazebo.
        '''
        # Get the index (or return if none)
        try:
            index = msg.name.index('flyingball::ball')
        except:
            self.get_logger().info("No flying ball")
            return

        # Grab the position and linear velocity.
        pos = msg.pose[index].position
        vel = msg.twist[index].linear

        # Update class ball position and velocity variables
        self.ballpos = np.array([pos.x, pos.y, pos.z]).reshape(-1, 1)
        self.ballvel = np.array([vel.x, vel.y, vel.z]).reshape(-1, 1)

        # Report.
        # self.get_logger().info("Pos %6.3f %6.3f %6.3f  vel %6.3f %6.3f %6.3f" %
                               #(pos.x, pos.y, pos.z, vel.x, vel.y, vel.z))




#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the interface node (100Hz).
    rclpy.init(args=args)
    node = GazeboInterfaceNode('test')

    # Spin, until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()