"""Launch Gazebo with the sevenDOF robot AND a bouncing ball

This instantiates Gazebo with the "SLOW" world (1/3 normal time), but
publishing all object data and with a bouncy ground plane!  It also
loads the 7 DOF robot (including the associated ROS interface) as well
as a tetherball and beer can.

For nodes, it starts the classic GUI and a "tracking" node that forces
the robot in Gazebo to track the GUI commands.

Finally, it also starts 'bounce.py' which spawns a bouncy ball and
watches the balls movements.

This should be a ready-to-run demo.  Take the pieces that you need!

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import DeclareLaunchArgument
from launch.actions                    import IncludeLaunchDescription
from launch.actions                    import ExecuteProcess
from launch.actions                    import RegisterEventHandler
from launch.actions                    import Shutdown
from launch.conditions                 import IfCondition
from launch.conditions                 import UnlessCondition
from launch.event_handlers             import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import LaunchConfiguration
from launch.substitutions              import ThisLaunchFileDir
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the world.
    world = os.path.join(pkgdir('gazebodemo'), 'worlds/slowtime.world')

    # Locate the tetherball SDF model file.
    tetherball = os.path.join(pkgdir('gazebodemo'),
                              'models/tetherball/model.sdf')


    # Locate the robot's URDF file and load the XML.
    # urdffile = os.path.join(pkgdir('gazebodemo'), 'urdf/sevenDOF_gazebo.urdf')
    # with open(urdffile, 'r') as file:
    #     robot_description = file.read()

    # Locate the robot's XACRO file to convert into a URDF XML.
    xacrofile = os.path.join(pkgdir('gazebodemo'),
                             'urdf/sevenDOF_gazebo_velocity.xacro')
    with open(xacrofile, 'r') as file:
        doc = xacro.parse(file)
        xacro.process_doc(doc)
        robot_description = doc.toxml()


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])


    # Get the standard Gazebo launch description.  Override it's
    # verbose, gui_required, server_required arguments.
    incl_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkgdir('gazebo_ros'), 'launch/gazebo.launch.py')),
        launch_arguments={'verbose':         'true',
                          'gui_required':    'true',
                          'server_required': 'true',}.items())

    # Configure a node to spawn wall into Gazebo.
    node_spawn_wall = Node(
        name       = 'spawn_wall',
        package    = 'gazebo_ros',
        executable = 'spawn_entity.py',
        respawn    = False,
        output     = 'screen',
        arguments  = ['-entity', 'wall', '-database', 'grey_wall',
                      '-x', '0.0', '-y', '3.0', '-z', '0.0',])

    # Configure a node to spawn wall into Gazebo.
    node_spawn_wall2 = Node(
        name       = 'spawn_wall2',
        package    = 'gazebo_ros',
        executable = 'spawn_entity.py',
        respawn    = False,
        output     = 'screen',
        arguments  = ['-entity', 'wall2', '-database', 'grey_wall',
                      '-x', '-3.0', '-y', '0.0', '-z', '0.0', '-Y', '1.57'])

    # Configure a node to spawn wall into Gazebo.
    node_spawn_wall3 = Node(
        name       = 'spawn_wall3',
        package    = 'gazebo_ros',
        executable = 'spawn_entity.py',
        respawn    = False,
        output     = 'screen',
        arguments  = ['-entity', 'wall3', '-database', 'grey_wall',
                      '-x', '3.0', '-y', '0.0', '-z', '0.0', '-Y', '1.57'])

    # Configure a node to spawn wall into Gazebo.
    node_spawn_wall4 = Node(
        name       = 'spawn_wall4',
        package    = 'gazebo_ros',
        executable = 'spawn_entity.py',
        respawn    = False,
        output     = 'screen',
        arguments  = ['-entity', 'wall4', '-database', 'grey_wall',
                      '-x', '0.0', '-y', '-3.0', '-z', '0.0',])

    # Configure a node to spawn the robot into Gazebo.
    node_spawn_robot = Node(
        name       = 'spawn_robot',
        package    = 'gazebo_ros',
        executable = 'spawn_entity.py',
        respawn    = False,
        output     = 'screen',
        arguments  = ['-entity', 'sevendof', '-topic', 'robot_description'])
        # arguments  = ['-entity', 'sixdof', '-file', robot])

    # Configure a control loader for the joint state broadcaster.
    load_joint_state_broadcaster = ExecuteProcess(
        cmd    = ['ros2', 'control', 'load_controller',
                  '--set-state', 'active', 'joint_state_broadcaster'],
        output = 'screen')

    event_load_joint_state_broadcaster = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action = node_spawn_robot,
            on_exit       = [load_joint_state_broadcaster]))

    # Configure a control loader for the velocity controller.
    load_velocity_controller = ExecuteProcess(
        cmd    = ['ros2', 'control', 'load_controller',
                  '--set-state', 'active', 'velocity_controller'],
        output = 'screen')

    event_load_velocity_controller = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action = load_joint_state_broadcaster,
            on_exit       = [load_velocity_controller]))


    # Configure a node for the tracking
    node_track = Node(
        name       = 'test', 
        package    = 'gazebodemo',
        executable = 'trackgui',
        output     = 'screen',
        on_exit    = Shutdown())


    # Configure a node to handle the bouncing ball: spawning,
    # deleting, reseting, watching.
    node_bounce = Node(
        name       = 'bounce', 
        package    = 'gazebodemo',
        executable = 'bounce',
        output     = 'screen',
        on_exit    = Shutdown())


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Declared arguments can be changed in the command line as
        # 'param:=value'

        # Pick one of the above worlds.
        DeclareLaunchArgument('world', default_value=world),

        # Start paused.  Set to 'false' to start the simulation running.
        DeclareLaunchArgument('pause', default_value='true'),


        # Setup events to load the controllers after the robot is spawned.
        event_load_joint_state_broadcaster,
        event_load_velocity_controller,

        # Start the robot_state_publisher, gazebo, and the spawners.
        node_robot_state_publisher,
        incl_gazebo,
        node_spawn_wall,
        node_spawn_wall2,
        node_spawn_wall3,
        node_spawn_wall4,
        node_spawn_robot,

        node_track,
        node_bounce,
    ])

