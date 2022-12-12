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
from hw3code.Segments   import Hold, Stay, GotoCubic, SplineCubic
from hw4code.hw4p3      import fkin, Jac
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
        self.qA = np.array([0, -0.39, 0, 0.73, 0, 0, 0]).reshape(7,1)
        #self.qB = np.array([1.55, -0.39, 0, 0.73, 0, 0.63, 0])
       
        #self.xB = [-5.0, 0.0, 3.0]

        # Pre-compute what we can.
        #self.xA = fkin(self.qA)
        #self.xA = [0.0, 0.74, 0.58]
        self.x = 0
        self.tA = 0.0
        self.tB = 3.0
        self.t_matrix = np.array([self.tA, self.tB])
        self.A_matrix = np.array([[0.0, -0.6, 0.1, 0.0],
                                #[1.0, -0.55, 0.15, 0.02],
                                #[2.0, -0.4, 0.3, 0.05],
                                #[2.3, -0.3, 0.4, 0.1],
                                #[2.75, -0.15, 0.55, 0.15],
                                [1.0, 0.0, 0.0, 0.0]])
        #self.xA_matrix = np.array([self.tA, self.xA[0][0], self.xA[1][0], self.xA[2][0]]).reshape(1, 4)
        #self.xB_matrix = np.array([self.tB, self.xB[0][0], self.xB[1][0], self.xB[2][0]]).reshape(1, 4)
        #self.coords = np.array([self.xA_matrix, self.xB_matrix]).reshape(2, 4)
        self.N = 5 # highest degree of the spline
        #self.c_x, self.c_y, self.c_z = self.getSplineCoeffs(self.coords)
        # Better to ensure matching than hard-coding
        # self.xA = np.array([0.0,  1.0, 0.0]).reshape(3,1)

        # qB will be determined by the ikin at runtime...

        # Select the segment duration.
        self.T = 3.0

        # Define the current segment.  The first segment will be the
        # task-space tip movement from xA to xB.  Zero the start time.
        #self.t0      = 0.0
        #self.segment = GotoCubic(self.xA, self.xB, self.T, space='Tip')

        # Initialize the current joint location and the matching tip
        # error (which should be zero).
        self.q   = self.qA
        self.err = 0
        #self.err = self.xA - fkin(self.qA)

        # Pick the convergence bandwidth.
        self.lam = 10

        self.t = 0.0


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, tabsolute, dt, ballpos):
        if tabsolute > self.T: # reset the time for each segment
            return self.evaluate(tabsolute - self.T, dt)
        self.t = tabsolute
        print("ball position")
        print(ballpos)
        self.A_matrix = np.array([[0.0, -0.6, 0.1, 0.0],
                                [1.0, ballpos[0], ballpos[1], ballpos[2]]])

        c_x = self.getSplineCoeffs(self.A_matrix[:, 0], self.A_matrix[:, 1])
        c_y = self.getSplineCoeffs(self.A_matrix[:, 0], self.A_matrix[:, 2])
        c_z = self.getSplineCoeffs(self.A_matrix[:, 0], self.A_matrix[:, 3])
        c_x_vel = [c_x[n] for n in range(self.N+1)]
        c_y_vel = [c_y[n] for n in range(self.N+1)]
        c_z_vel = [c_z[n] for n in range(self.N+1)]
        #print('c_x')
        #print(c_x.tolist())
        #print('end')
        x_x = 0
        x_y = 0
        x_z = 0
        v_x = 0
        v_y = 0
        v_z = 0
        for deg in range(self.N+1):
            x_x += c_x.tolist()[deg] * self.t ** deg
            x_y += c_y.tolist()[deg] * self.t ** deg
            x_z += c_z.tolist()[deg] * self.t ** deg
            if deg != 0:
                v_x += c_x.tolist()[deg] * deg * self.t ** (deg - 1)
                v_y += c_y.tolist()[deg] * deg * self.t ** (deg - 1)
                v_z += c_z.tolist()[deg] * deg * self.t ** (deg - 1)
        #x_x = c_x.tolist()[0] * self.t 
        #x_y = c_y.tolist()[0] * self.t 
        #x_z = c_z.tolist()[0] * self.t 
        #t_coords = np.array([self.t**deg for deg in range(self.n+1)]).reshape(self.n+1, 1)
        #t_coords_vel = np.array([(self.t-1)*self.t**deg for deg in range(self.n+1)]).reshape(self.n+1, 1)
        #x_x = self.c_x.flatten() @ t_coords.flatten()    
        #x_y = self.c_y.flatten() @ t_coords.flatten()    
        #x_z = self.c_z.flatten() @ t_coords.flatten()    
        x = np.array([x_x, x_y, x_z]).reshape(3,1)
        #print('x')
        #print(x)
        #print('end')
        q = self.ikin(x, x, self.q)
        
        # calculate velocities
        xdot = np.array([v_x, v_y, v_z]).reshape(3, 1)
        #print('xdot')
        #print(xdot)
        #print('end')
        self.x = x
        #v = pn.array([v_x, v_y, v_z]).reshape(3, 1)
        #J7   = np.array([0.5, 0, 1, 0, 0, 0, 0]).reshape((1,7))
        #vd7  = 0

        wd = np.zeros((3,1))
        Rd = Reye()

        J    = np.vstack((self.chain.Jv(),self.chain.Jw()))
        xdot = np.vstack((xdot, wd))

        Jinv = np.linalg.pinv(J)
        qdot = Jinv @ (xdot + self.lam * self.err)
        self.qdot = qdot
        #print('qdot')
        #print(qdot)
        #print('end')

        q    = q + dt * qdot
        self.chain.setjoints(q)
        self.q = q
        #print('q')
        #print(q)
        #print('end')

        #xd7  = 0
        #x7   = 0.5*q[0,0] + q[2,0]
        self.err  = np.vstack((ep(x, self.chain.ptip()),
                      eR(Rd, self.chain.Rtip())))
                     # xd7-x7))
        #print('err')
        #print(self.err)
        #print('end')
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
        #t_coords = np.array([spline_coords[0][0], spline_coords[1][0]]).reshape(2, 1)
        #x_coords = np.array([spline_coords[0][1], spline_coords[1][1]]).reshape(2, 1)
        #y_coords = np.array([spline_coords[0][2], spline_coords[1][2]]).reshape(2, 1)
        #z_coords = np.array([spline_coords[0][3], spline_coords[1][3]]).reshape(2, 1)
        #t = []
        #for t_point in t_coords: # calculate the t matrix
        #    t.append(np.array([t_point**deg for deg in range(self.n+1)]).reshape(1, self.n+1))
        #t = np.array([t]).reshape(2, self.n+1)
        #c_x = np.linalg.pinv(t) @ x_coords
        #c_y = np.linalg.pinv(t) @ y_coords
        #c_z = np.linalg.pinv(t) @ z_coords
        #return c_x, c_y, c_z

    def ikin(self, x_d, x_guess, q_guess):
        # Return the ikin using the given conditions
        #x_guess = fkin(q_guess)
        dx = x_d - x_guess
        J = np.vstack((self.chain.Jv(), self.chain.Jw()))
        # If the position is within an arbitrarily small threshold, we return q_guess
        if np.abs(dx[0][0]) < 10**-6 and np.abs(dx[1][0]) < 10**-6 and np.abs(dx[2][0]) < 10**-6:
           return np.array(q_guess)
        # Else, update theta_guess and run it again with updated angles
        self.theta_guess = q_guess
        return self.ikin(x_d, x_guess + dx, q_guess + np.linalg.pinv(J) @ dx)

#
#   Gazebo Interface Node Class
#
class GazeboInterfaceNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        #initialize ball positions/velocities
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
        print("this is the position")
        print(self.ballpos)
        # Compute the desired joint positions and velocities for this time.
        desired = self.trajectory.evaluate(t, dt, self.ballpos)
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

        # Receive the link states from Gazebo.
    def cb_links(self, msg):
        # Get the index (or return if none)
        try:
            index = msg.name.index('flyingball::ball')
        except:
            self.get_logger().info("No flying ball")
            return

        # Grab the position and linear velocity.
        pos = msg.pose[index].position
        vel = msg.twist[index].linear

        # Report
        self.ballpos = np.array([pos.x, pos.y, pos.z]).reshape(-1, 1)
        self.ballvel = np.array([vel.x, vel.y, vel.z]).reshape(-1, 1)

        #self.get_logger().info("Pos %6.3f %6.3f %6.3f  vel %6.3f %6.3f %6.3f" %
                               #(pos.x, pos.y, pos.z, vel.x, vel.y, vel.z))




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
