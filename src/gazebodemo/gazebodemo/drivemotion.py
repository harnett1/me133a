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


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, tabsolute, dt):
        # Take action, based on whether the current segment's space.
        if self.segment.space() == 'Tip':
            # If the movement is completed, putting us at point B, add
            # a joint move back to qA.  Re-evaluate on new segment.
            if self.segment.completed(tabsolute - self.t0):
                self.t0      = self.t0 + self.segment.duration()
                self.segment = GotoCubic(self.q, self.qA, self.T, space='Joint')
                return self.evaluate(tabsolute, dt)

            # If the movement is ongoing, compute the current values.
            (xd, xddot) = self.segment.evaluate(tabsolute - self.t0)

            # Grab the last joint value and tip error.
            q   = self.q
            err = self.err

            # Compute the inverse kinematics
            J    = Jac(q)
            qdot = np.linalg.inv(J) @ (xddot + self.lam * err)
            q    = q + dt * qdot

            # Save the joint value and precompute the tip error for next cycle.
            self.q   = q
            self.err = xd - fkin(q)

        # For joint splines:
        else:
            # If the movement is completed, putting us at point A, add
            # a new straight line to xB.  Re-evaluate on new segment.
            if self.segment.completed(tabsolute - self.t0):
                self.t0      = self.t0 + self.segment.duration()
                self.segment = GotoCubic(self.xA, self.xB, self.T, space='Tip')
                # Return None to stop.
                return self.evaluate(tabsolute, dt)

            # If the movement is ongoing, compute the current values.
            (q, qdot) = self.segment.evaluate(tabsolute - self.t0)

            # Also, to transition back to tip space, save the joint
            # value and declare a zero tip error.
            self.q = q
            self.e = np.array([0.0, 0.0, 0.0]).reshape(3,1)


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
                       'theta6':5
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
