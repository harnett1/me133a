"""Launch Gazebo with the sixDOF robot

This instantiates Gazebo with the "SLOW" world, i.e. running at 1/3
normal time.  It also loads the 6 DOF robot (including the associated
ROS interface) as well as a tetherball and beer can.

But it does NOT start anything to send commands to Gazebo.  Start the
drivemotion.py file separately or incorporate below.

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import DeclareLaunchArgument
from launch.actions                    import IncludeLaunchDescription
from launch.actions                    import ExecuteProcess
from launch.actions                    import RegisterEventHandler
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
    
    # Define the package.
    package = 'gazebodemo'

    # Locate the worlds - pick one of these.
    realtime_world = os.path.join(pkgdir(package), 'worlds/realtime.world')
    slowtime_world = os.path.join(pkgdir(package), 'worlds/slowtime.world')
    stuff_world    = os.path.join(pkgdir(package), 'worlds/stuff.world')

    # Locate the tetherball SDF model file.
    tetherball = os.path.join(pkgdir(package), 'models/tetherball/model.sdf')


    # Locate the robot's URDF file and load the XML.
    # urdffile = os.path.join(pkgdir(package), 'urdf/sixDOF_gazebo.urdf')
    # with open(urdffile, 'r') as file:
    #     robot_description = file.read()

    # Locate the robot's XACRO file to convert into a URDF XML.
    # xacrofile = os.path.join(pkgdir(package), 'urdf/sixDOF_gazebo_effort.xacro')
    # xacrofile = os.path.join(pkgdir(package), 'urdf/sixDOF_gazebo_position.xacro')
    xacrofile = os.path.join(pkgdir(package), 'urdf/sixDOF_gazebo_velocity.xacro')
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

    # Configure a node to spawn the beer can into Gazebo.
    node_spawn_beercan = Node(
        name       = 'spawn_beercan',
        package    = 'gazebo_ros',
        executable = 'spawn_entity.py',
        respawn    = False,
        output     = 'screen',
        arguments  = ['-entity', 'beercan', '-database', 'beer',
                      '-x', '0.0', '-y', '0.5', '-z', '0.0'])

    # Configure a node to spawn the tetherball into Gazebo.
    node_spawn_tetherball = Node(
        name       = 'spawn_tetherball',
        package    = 'gazebo_ros',
        executable = 'spawn_entity.py',
        respawn    = False,
        output     = 'screen',
        arguments  = ['-entity', 'tetherball', '-file', tetherball,
                      '-x', '-0.2', '-y', '0.5', '-z', '1.2'])

    # Configure a node to spawn the robot into Gazebo.
    node_spawn_robot = Node(
        name       = 'spawn_robot',
        package    = 'gazebo_ros',
        executable = 'spawn_entity.py',
        respawn    = False,
        output     = 'screen',
        arguments  = ['-entity', 'sixdof', '-topic', 'robot_description'])

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


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Declared arguments can be changed in the command line as
        # 'param:=value'

        # Pick one of the above worlds.
        DeclareLaunchArgument('world', default_value=slowtime_world),

        # Start paused.  Set to 'false' to start the simulation running.
        DeclareLaunchArgument('pause', default_value='true'),


        # Setup events to load the controllers after the robot is spawned.
        event_load_joint_state_broadcaster,
        event_load_velocity_controller,

        # Start the robot_state_publisher, gazebo, and the spawners.
        node_robot_state_publisher,
        incl_gazebo,
        node_spawn_beercan,        
        node_spawn_tetherball,
        node_spawn_robot,
    ])

