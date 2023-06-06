"""
_______________________________________________________________________________
Launch file used to launch multiple node 
_______________________________________________________________________________
"""

# Importing modules and functions
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import sys
import yaml

def operator_launch(context, *args, **kwargs):
    """
    Setting up multiple nodes for launch
    """

    # Storing the path to YAML file (mpc_controller/params/params.yaml)
    param_path = os.path.join(
        get_package_share_directory('operator_package'),
        'params',
        'params.yaml'
    )

    # Storing dictionary of parameters in param variable
    with open(param_path, 'r') as f:
        config = yaml.safe_load(f)
        param = config['/**']['ros_parameters']

    n_multi_agent = param['n_multi_agent']  # Size of fleet
    FOV_max = param['FOV_range_deg']        # Hard constraint - Field of View
    launch_list = []                        # Empty list used to store nodes

    trajectory_node = Node(
       package='operator_package',    # Package name
       executable='setpoint',         # Node executable name from setup.py
       output='log', 
       parameters=[param]
    )
    launch_list.append(trajectory_node)

    gui_node = Node(                        # Graphical User Interface node
        package='operator_package',         # Package name
        executable='GUI',                   # Node executable name from setup.py
        output='log',
        parameters=[
        {
            'n_multi_agent': n_multi_agent, # Size of fleet
            'FOV_max': FOV_max,             # Hard constraint - Field of View
        }
        ]
    )
    launch_list.append(gui_node)

    data_node = Node(                        
        package='operator_data_interface',           
        executable='operator_data_interface_node',
        output='log',
        parameters=[param]
    )
    launch_list.append(data_node)

    return launch_list # Returns the list with nodes

#Launching of nodes
def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=operator_launch)
    ])
