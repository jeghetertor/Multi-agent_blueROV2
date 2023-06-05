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

def rov_launch(context, *args, **kwargs):
    """
    Setting up multiple nodes for launch
    """

    # Storing the path to YAML file (mpc_controller/params/params.yaml)
    param_path = os.path.join(
        get_package_share_directory('mpc_controller'),
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
    rov_id = param['rov_id']

    # Checks the size of fleet and generates a mpc node for each ROV
    ## Starting number for ROV is 2 because of the modelname bluerov2.
    agent = Node(
        package='mpc_controller',                       # Package name
        namespace='bluerov{}_mpc'.format(rov_id),       # Customize node name
        name='bluerov{}'.format(rov_id),                # Node shown in list as : /bluerov{}_mpc/bluerov{}
        executable='bluerov2_mpc',                       # Node executable name from setup.py
        output='screen', 
        parameters= [param,
        {
            'main_id': rov_id,                          # Identification of main ROV
            'n_multi_agent': n_multi_agent,             # Size of fleet
        }
        ]
    )
    launch_list.append(agent) # Add to the launch list

    # trajectory_node = Node(
    #    package='mpc_controller',    # Package name
    #    executable='setpoint',       # Node executable name from setup.py
    #    output='log', 
    #    parameters=[param]
    # )
    # launch_list.append(trajectory_node)

    return launch_list # Returns the list with nodes

#Launching of nodes
def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=rov_launch)
    ])
