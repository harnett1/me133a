'''hw5p4.py

   This is skeleton code for HW5 Problem 5a.  Please EDIT.

   This creates a purely rotational movement.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from hw5code.GeneratorNode     import GeneratorNode
from hw5code.KinematicChain    import KinematicChain
from hw5code.TransformHelpers  import *


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Initialize the current joint position and chain data.
        self.q = np.zeros((3,1))
        self.chain.setjoints(self.q)

        # Also zero the task error.
        self.err = np.zeros((3,1))

        # Pick the convergence bandwidth.
        self.lam = 20


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['pan', 'tilt', 'roll']

    # Evaluate at the given time.
    def evaluate(self, t, dt):
        # Compute the desired rotation and angular velocity!
        if   t < 2:
            alpha    = FIXME
            alphadot = FIXME
        else:
            alpha    = - np.pi/2
            alphadot = 0.0

        (FIXME: similar for beta in second part.)
            
        FIXME: WHAT ARE Rd and wd?

        # Grab the stored information from last cycle.
        FIXME: WHAT DATA FROM LAST CYCLE DO WE NEED?

        # Compute the inverse kinematics
        FIXME: INVERSE KINEMATICS FOR ROTATION

        # Integrate the joint position, update, store as appropriate.
        q = q + dt * qdot
        FIXME: UPDATE AND STORE

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the generator node (100Hz) for the Trajectory.
    rclpy.init(args=args)
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
