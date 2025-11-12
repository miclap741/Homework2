from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart

import os

def generate_launch_description():
     
     controller_which = DeclareLaunchArgument(
         'cmd_interface',
         default_value='position',
         description='Declares which controller to use'
     )
     
     controller_which_vel = DeclareLaunchArgument(
         'ctrl',
         default_value='velocity',
         description='Declares which velocity controller to use'
     )
     
     controller_iiwa = DeclareLaunchArgument(
         'command_interface',
         default_value='position',
         description='iiwa launch argument'
     )
     
     controller_iiwa_robot= DeclareLaunchArgument(
         'robot_controller',
         default_value='iiwa_arm_controller',
         description='iiwa launch argument'
     )
     
     
     # lancio il robot iiwa in simulazione
     iiwa_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(  # serve questa riga per caricare un launch.py
             PathJoinSubstitution([
                 FindPackageShare('iiwa_bringup'),  
                'launch',
                'iiwa.launch.py' 
             ])
         ),
         launch_arguments={
             'use_sim': 'true',
             'use_fake_hardware': 'false',
             'start_rviz': 'false',
             'gz_args': '-r /home/user/ros2_ws/src/ros2_iiwa/iiwa_description/gazebo/worlds/aruco_world.world -v 1',
             'command_interface': LaunchConfiguration('command_interface'),
             'robot_controller': LaunchConfiguration('robot_controller')
         }.items()
     )
     
     
     # nodo per la detection dell'aruco tag
     single_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(  # serve questa riga per caricare un launch.py
             PathJoinSubstitution([
                 FindPackageShare('aruco_ros'),  
                'launch',
                'single.launch.py' 
             ])
         ),
         launch_arguments={
             'marker_id': '201',
             'marker_size': '0.1',
             'eye': 'left',
             'marker_frame': 'aruco_marker_frame',
             'reference_frame': '',
             'corner_refinement': 'LINES',
         }.items()
     ) 
       
     
     # Percorso del file YAML dei parametri    
     config_file = os.path.join(    
         get_package_share_directory('ros2_kdl_package'),     
         'config',    
         'iiwa_parameters.yaml'    
     )     
   

     # Definizione del nodo    
     kdl_node = Node(
         package='ros2_kdl_package',
         executable='ros2_kdl_node',
         name='ros2_kdl_node',
         output='screen',
         parameters=[
             config_file, 
             {'cmd_interface': LaunchConfiguration('cmd_interface')}, 
             {'ctrl': LaunchConfiguration('ctrl')}]
     )
     
     # Ritardo di 10 secondi per permettere l'avvio della simulazione
     delay_kdl_node = TimerAction(
         period=15.0,
         actions=[kdl_node]
     )
     
     camera_bridge= Node(
         package='ros_ign_bridge',
         executable='parameter_bridge',
         name='camera_bridge',
         output='screen',
         parameters=[{'use_sim_time': True}],
         arguments=[
             '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
             '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
             '--ros-args', 
             '-r', '/camera:=/stereo/left/image_rect_color',
             '-r', '/camera_info:=/stereo/left/camera_info',
         ]
     )   

     set_pose_bridge = Node(
	    package='ros_ign_bridge',
	    executable='parameter_bridge',
	    name='set_pose_bridge',
	    output='screen',
	    arguments=[
		# Sintassi: ign_service@ros2_service@srv:ros2_type
		'/world/aruco_world/set_pose@/set_pose@srv:ros_ign_interfaces/srv/SetEntityPose'
	    ]
	)



     # Ritorna la descrizione del launch 
     return LaunchDescription([
         controller_which,
         controller_iiwa,
         controller_iiwa_robot,
         controller_which_vel,
         iiwa_launch,
         camera_bridge,
         single_launch,
         delay_kdl_node,
         set_pose_bridge
     ])
