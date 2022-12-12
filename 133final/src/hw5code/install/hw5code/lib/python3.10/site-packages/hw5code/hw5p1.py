'''hw5p1.py

   This is skeleton code for HW5 Problem 1.  Please EDIT.

   This should re-use the trajectory construction from HW4 P4.

   It is also updated to properly handle shut down when the trajectory
   runs out of segments.  And provides the time step (dt) to the
   evaluation, in preparation for the coming inverse kinematics.  The
   generator node itself is pulled into a seperately file.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from hw5code.GeneratorNode  import GeneratorNode
from hw3code.Segments       import Hold, Stay, GotoCubic, SplineCubic

DO YOU NEED TO IMPORT THE FORWARD KINEMATIC FUNCTIONS?


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Pick the target.
        xgoal = np.array([0.2, -1.0, 0.5]).reshape((3,1))
        
        # Build up the list of segments, starting with nothing.
        self.segments = []


        PLEASE ADD YOUR CODE HERE


        # Disable cyclic to end after the last segment.
        self.cyclic = False
    
        # Zero the start time of the current segment.
        self.t0 = 0.0


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3']

    # Evaluate at the given time.
    def evaluate(self, tabsolute, dt):
        # Make sure we have a segment.
        if len(self.segments) == 0:
            return None

        # Also check whether the current segment is done.
        if self.segments[0].completed(tabsolute - self.t0):
            # If the current segment is done, shift to the next
            self.t0 = self.t0 + self.segments[0].duration()
            seg = self.segments.pop(0)
            if self.cyclic:
                self.segments.append(seg)

            # Make sure we still have something to do.
            if len(self.segments) == 0:
                return None

        # Compute the positions/velocities as a function of time.
        (q, qdot) = self.segments[0].evaluate(tabsolute - self.t0)

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
