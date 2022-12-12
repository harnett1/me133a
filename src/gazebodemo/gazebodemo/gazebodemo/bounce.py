'''bounce.py

   Demostrate watching a bouncing ball in Gazebo.

   Node:            /bounce
   Subscibe:        /gazebo/link_states         gazebo_msgs/LinkStates
   Service Client:  /spawn_entity               gazebo_msgs/srv/SpawnEntity
                    /delete_entity              gazebo_msgs/srv/DeleteEntity
                    /gazebo/set_entity_state    gazebo_msgs/srv/SetEntityState

'''

import os
import sys
import rclpy
import numpy as np

from ament_index_python.packages import get_package_share_directory

from rclpy.node         import Node
from gazebo_msgs.srv    import SpawnEntity
from gazebo_msgs.srv    import DeleteEntity
from gazebo_msgs.srv    import SetEntityState
from gazebo_msgs.msg    import LinkStates
from geometry_msgs.msg  import Pose, Twist, Point, Quaternion, Vector3


#
#   Demo Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create the spawn service client.
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.get_logger().info("Connecting to spawning service...")
        if not self.spawn_client.service_is_ready():
            self.spawn_client.wait_for_service()
        self.get_logger().info("...connected!")

        # Create the delete service client.
        self.delete_client = self.create_client(DeleteEntity, "/delete_entity")
        self.get_logger().info("Connecting to deleting service...")
        if not self.delete_client.service_is_ready():
            self.delete_client.wait_for_service()
        self.get_logger().info("...connected!")

        # Create the place service client.
        self.place_client = self.create_client(SetEntityState,
                                               "/gazebo/set_entity_state")
        self.get_logger().info("Connecting to placing service...")
        if not self.place_client.service_is_ready():
            self.place_client.wait_for_service()
        self.get_logger().info("...connected!")

        # Grab the ball model.
        modelfile = os.path.join(get_package_share_directory("gazebodemo"),
                                 "models", "ball", "model.sdf")
        with open(modelfile, 'r') as file:
            self.ballmodel = file.read()

        # Clear the list of active requests.  This is only to report
        # on the result...
        self.acive_requests = []

        # Create a subscriber to the gazebo state.
        #self.sub_links = self.create_subscription(
        #    LinkStates, '/gazebo/link_states' ,self.cb_links, 10)


    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()

    # Spin
    def spin(self):
        # Keep running while ok (until interrupted).  If we didn't
        # want to report on the request results, we could just run:
        # rclpy.spin(self).
        while rclpy.ok():
            # Spin once.
            rclpy.spin_once(self)

            # Check whether there is an active request.
            for i in range(len(self.acive_requests)-1, -1, -1):
                # Check the request status.
                if self.acive_requests[i].done():
                    # Report the result and clear the active request.
                    response = self.acive_requests[i].result()
                    try:
                        msg = response.status_message
                    except:
                        msg = ''
                    self.get_logger().info("Service request done: %d '%s'" %
                                           (response.success, msg))
                    del(self.acive_requests[i])


    # Spawn a ball.
    def spawn(self):
        # Create the spawn request
        request = SpawnEntity.Request()
        request.name = 'flyingball'
        request.xml  = self.ballmodel
        request.robot_namespace = "ball"
        request.initial_pose.position.x =  0.5
        request.initial_pose.position.y =  0.6
        request.initial_pose.position.z =  0.2

        # Send the request (to be completed asynchronously).
        self.acive_requests.append(self.spawn_client.call_async(request))
        self.get_logger().info("Sent spawn service request...")

    # Delete an existing ball
    def delete(self):
        # Create the delete request
        request = DeleteEntity.Request()
        request.name = 'flyingball'

        # Send the request (to be completed asynchronously).
        self.acive_requests.append(self.delete_client.call_async(request))
        self.get_logger().info("Sent delete service request...")

    # Place an existing ball.
    def place(self, px, py, pz, vx, vy, vz):
        # Create the Pose and Twist
        pos = Point(x=px, y=py, z=pz)
        vel = Vector3(x=vx, y=vy, z=vz)
        pose  = Pose(position=pos, orientation=Quaternion())
        twist = Twist(linear=vel, angular=Vector3())

        # Create the place request
        request = SetEntityState.Request()
        request.state.name  = 'flyingball'
        request.state.pose  = pose
        request.state.twist = twist
        request.state.reference_frame = 'world'

        # Send the request (to be completed asynchronously).
        self.acive_requests.append(self.place_client.call_async(request))
        self.get_logger().info("Sent place service request...")


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
        self.get_logger().info("Pos %6.3f %6.3f %6.3f  vel %6.3f %6.3f %6.3f" %
                               (pos.x, pos.y, pos.z, vel.x, vel.y, vel.z))


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the demo node.
    rclpy.init(args=args)
    node = DemoNode('bounce')

    # The following should be tiggered at some point in time, in some
    # callback.  Putting here just to test/show...

    # Spawn a ball.  Or delete a ball.  Or place a ball.  TRY ONE.
    node.spawn()
    # node.delete()
    # node.place(0.0, 1.0, 1.0, 0.0, 0.1, 1.0)

    # Spin, until interrupted.  This handles the callbacks, so prints
    # the state as well as the request return code.
    node.spin()

    
    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
