"""
ROS 2 Navigation Simulator — Launch File
Usage (standalone): python3 nav_sim.launch.py
Usage (ROS 2): ros2 launch ros2_nav_sim nav_sim.launch.py
"""

import os
import sys

def generate_launch_description():
    """Simulate ROS 2 LaunchDescription pattern."""
    # Parameters
    params = {
        'map_yaml': os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'factory_map.yaml'),
        'use_sim_time': True,
        'autostart': True,
        'bt_xml_file': '',
        'waypoints_file': '',
    }
    
    # Node definitions (like ROS 2 launch descriptions)
    nodes = [
        {
            'package': 'ros2_nav_sim',
            'executable': 'map_server',
            'name': 'map_server',
            'output': 'screen',
            'parameters': [params],
        },
        {
            'package': 'ros2_nav_sim', 
            'executable': 'robot_state_publisher',
            'name': 'robot_state_publisher',
            'output': 'screen',
            'parameters': [params],
        },
        {
            'package': 'ros2_nav_sim',
            'executable': 'global_planner',
            'name': 'global_planner',
            'output': 'screen',
            'parameters': [params],
        },
        {
            'package': 'ros2_nav_sim',
            'executable': 'controller_server',
            'name': 'controller_server',
            'output': 'screen',
            'parameters': [params],
        },
        {
            'package': 'ros2_nav_sim',
            'executable': 'waypoint_navigator',
            'name': 'waypoint_follower',
            'output': 'screen',
            'parameters': [params],
        },
        {
            'package': 'ros2_nav_sim',
            'executable': 'lifecycle_manager',
            'name': 'lifecycle_manager_navigation',
            'output': 'screen',
            'parameters': [{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'global_planner',
                    'controller_server',
                    'waypoint_follower',
                ],
            }],
        },
    ]
    
    return {
        'type': 'LaunchDescription',
        'nodes': nodes,
        'params': params,
    }


if __name__ == '__main__':
    ld = generate_launch_description()
    print("=" * 60)
    print("  🚀 ROS2 Navigation Simulator — Launch")
    print("=" * 60)
    for node in ld['nodes']:
        print(f"  [{node['package']}] {node['name']} ({node['executable']})")
    print("=" * 60)
    print("  Use: python3 main.py  (to run the full simulation)")
    print("=" * 60)
