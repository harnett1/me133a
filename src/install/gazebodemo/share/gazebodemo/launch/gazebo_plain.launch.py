"""Launch Plain Gazebo

This instantiates Gazebo with the "STUFF" world plus an extra drill.

"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import DeclareLaunchArgument
from launch.actions                    import IncludeLaunchDescription
from launch.conditions                 import IfCondition, UnlessCondition
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


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Get the standard Gazebo launch description.  Override it's
    # verbose, gui_required, server_required arguments.
    incl_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkgdir('gazebo_ros'), 'launch/gazebo.launch.py')),
        launch_arguments={'verbose':         'true',
                          'gui_required':    'true',
                          'server_required': 'true',}.items())

    # Configure a node to spawn the beer can into Gazebo.
    node_spawn_drill = Node(
        name       = 'spawn_drill',
        package    = 'gazebo_ros',
        executable = 'spawn_entity.py',
        respawn    = False,
        output     = 'screen',
        arguments  = ['-entity', 'drill', '-database', 'cordless_drill',
                      '-x', '1.0', '-y', '1.5', '-z', '0.0'])


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Declared arguments can be changed in the command line as
        # 'param:=value'

        # Pick one of the above worlds.  In this case, use STUFF_WORLD :)
        DeclareLaunchArgument('world', default_value=stuff_world),

        # Start running.  Set to 'true' to start the simulation paused.
        DeclareLaunchArgument('pause', default_value='false'),


        # Start gazebo and spawn the additional object.
        incl_gazebo,
        node_spawn_drill,        
    ])
