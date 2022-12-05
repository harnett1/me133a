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

from hw5code.GeneratorNode import GeneratorNode
from hw3code.Segments   import Hold, Stay, GotoCubic, SplineCubic
from hw4code.hw4p3      import fkin, Jac

#
#   COPY THE TRAJECTORY CLASS AND ANY SUPPORTING CODE
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Define known tip/joint positions
        self.qA = np.array([0, -0.39, 0, 0.73, 0, 0, 0]).reshape(7,1)
        #self.qB = np.array([1.55, -0.39, 0, 0.73, 0, 0.63, 0])
       
        self.xB = np.array([-5.0, 0.0, 3.0]).reshape(3,1)

        # Pre-compute what we can.
        self.xA = fkin(self.qA)
        self.x = self.xA
        self.tA = 0.0
        self.tB = 3.0
        self.xA_matrix = np.array([self.tA, self.xA[0][0], self.xA[1][0], self.xA[2][0]]).reshape(1, 4)
        self.xB_matrix = np.array([self.tB, self.xB[0][0], self.xB[1][0], self.xB[2][0]]).reshape(1, 4)
        self.coords = np.array([self.xA_matrix, self.xB_matrix]).reshape(2, 4)
        self.n = 5 # highest degree of the spline
        self.c_x, self.c_y, self.c_z = self.getSplineCoeffs(self.coords)
        # Better to ensure matching than hard-coding
        # self.xA = np.array([0.0,  1.0, 0.0]).reshape(3,1)

        # qB will be determined by the ikin at runtime...

        # Select the segment duration.
        self.T = 3.0

        # Define the current segment.  The first segment will be the
        # task-space tip movement from xA to xB.  Zero the start time.
        self.t0      = 0.0
        self.segment = GotoCubic(self.xA, self.xB, self.T, space='Tip')

        # Initialize the current joint location and the matching tip
        # error (which should be zero).
        self.q   = self.qA
        self.err = self.xA - fkin(self.qA)

        # Pick the convergence bandwidth.
        self.lam = 10

        self.t = 0.0


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, tabsolute, dt):
        if tabsolute > self.T:
            self.t = tabsolute
        t_coords = np.array([self.t**deg for deg in range(self.n+1)]).reshape(self.n+1, 1)
        t_coords_vel = np.array([(self.t-1)*self.t**deg for deg in range(self.n+1)]).reshape(self.n+1, 1)
        x_x = self.c_x.flatten() @ t_coords.flatten()    
        x_y = self.c_y.flatten() @ t_coords.flatten()    
        x_z = self.c_z.flatten() @ t_coords.flatten()    
        x = np.array([x_x, x_y, x_z]).reshape(3,1)
        print(x)
        q = self.ikin(x, self.q)
        print(q)
        # calculate velocities
        v_x = self.c_x @ t_coords_vel
        v_y = self.c_y @ t_coords_vel
        v_z = self.c_z @ t_coords_vel
        self.x = x
        self.q = q
        v = pn.array([v_x, v_y, v_z]).reshape(3, 1)
        qdot = ikin(v, self.q)
    
        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


    def getSplineCoeffs(self, spline_coords):
        # Calculates the spline coefficients for a given set of points along a path
        # in the form of s(t) = c_x_n * t^n + c_x_n-1 * t^(n+1) + ... + c_x_0
        # using linear regression to minimize the error, e = s - t * c, for each term
        # being a matrix
        t_coords = np.array([spline_coords[0][0], spline_coords[1][0]]).reshape(2, 1)
        x_coords = np.array([spline_coords[0][1], spline_coords[1][1]]).reshape(2, 1)
        y_coords = np.array([spline_coords[0][2], spline_coords[1][2]]).reshape(2, 1)
        z_coords = np.array([spline_coords[0][3], spline_coords[1][3]]).reshape(2, 1)
        t = []
        for t_point in t_coords: # calculate the t matrix
            t.append(np.array([t_point**deg for deg in range(self.n+1)]).reshape(1, self.n+1))
        t = np.array([t]).reshape(2, self.n+1)
        c_x = np.linalg.pinv(t) @ x_coords
        c_y = np.linalg.pinv(t) @ y_coords
        c_z = np.linalg.pinv(t) @ z_coords
        return c_x, c_y, c_z

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
