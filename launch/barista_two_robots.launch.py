import os
import random
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix


# this is the function launch  system will look for
def generate_launch_description():

  ####### DATA INPUT ##########
    robot_base_name = "barista"
    urdf_file = 'barista_robot_model.urdf.xacro'
    package_description = "barista_robot_description"

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_bot_gazebo = get_package_share_directory('barista_robot_description')

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    install_dir = get_package_prefix(package_description)

    robot_desc_path = os.path.join(get_package_share_directory(package_description), "xacro", urdf_file)
  ####### DATA INPUT END ##########


  ####### GAZEBO ##########
    # Set the path to the WORLD model files. Is to find the models inside the models folder in barista_robot_description package
    gazebo_models_path = os.path.join(pkg_box_bot_gazebo, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'
  
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
    )  
  ####### GAZEBO END##########


    robot_entity_name_1 = 'rick'#robot_base_name+"_"+str(int(random.random()*100000))
    robot_entity_name_2 = 'morty'#robot_base_name+"_"+str(int(random.random()*100000))

    # Robot State Publisher (xacro robot description)
    robot_description_1 = xacro.process_file(robot_desc_path, mappings={"robot_name": robot_entity_name_1}).toxml()
    robot_description_2 = xacro.process_file(robot_desc_path, mappings={"robot_name": robot_entity_name_2}).toxml()

    # ROBOT ENTITY 1
    robot_state_publisher_node1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace=robot_entity_name_1,
        emulate_tty=True,
        parameters=[{'frame_prefix': robot_entity_name_1+'/', 'use_sim_time': True, 'robot_description': robot_description_1}],
        output="screen"
    )

    # ROBOT ENTITY 2
    robot_state_publisher_node2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace=robot_entity_name_2,
        emulate_tty=True,
        parameters=[{'frame_prefix': robot_entity_name_2+'/', 'use_sim_time': True, 'robot_description': robot_description_2}],
        output="screen"
    )

    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.3]
    position2 = [1.0, 0.0, 0.3]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]

    # Spawn ROBOT 1 to Gazebo
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', robot_entity_name_1,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', robot_entity_name_1 + '/robot_description'
                   ]
    )

    # Spawn ROBOT 2 to Gazebo
    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', robot_entity_name_2,
                   '-x', str(position2[0]), '-y', str(position2[1]
                                                     ), '-z', str(position2[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', robot_entity_name_2 + '/robot_description'
                   ]
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'rviz_robot_laser.rviz')
    
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=robot_entity_name_1,
        output='screen',
    )

    # create and return launch description object
    return LaunchDescription(
        [       
            gazebo,
            robot_state_publisher_node1,
            robot_state_publisher_node2,
            #rviz_node,
            #joint_state_publisher,
            spawn_robot1,
            spawn_robot2,
        ]
    )

