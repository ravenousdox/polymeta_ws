from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    drone1_node = Node( 
        package='mavlink_ros2_bridge', 
        executable='mavlink_ros2_arm', 
        name='mavlink_ros2_drone1', 
        namespace='drone1', 
        parameters=[ 
            {'mavlink_endpoint': 'udp:127.0.0.1:14561'}, 
            {'heartbeat_topic': '/drone1/heartbeat_mode'}, 
            {'battery_topic': '/drone1/battery_status'}, 
            {'source_system': 51}, 
            {'source_component': 51}, 
            {'target_system': 1}, 
            {'target_component': 1}, 
            {'arm_service_name': '/drone1/arm_drone'}, 
            {'mode_command': 'FLOWHOLD'} 
        ] 
    ) 
    drone2_node = Node( 
        package='mavlink_ros2_bridge', 
        executable='mavlink_ros2_arm', 
        name='mavlink_ros2_drone2', 
        namespace='drone2', 
        parameters=[ 
            {'mavlink_endpoint': 'udp:127.0.0.1:14562'}, 
            {'heartbeat_topic': '/drone2/heartbeat_mode'}, 
            {'battery_topic': '/drone2/battery_status'}, 
            {'source_system': 52}, 
            {'source_component': 52}, 
            {'target_system': 2}, 
            {'target_component': 1}, 
            {'arm_service_name': '/drone2/arm_drone'}, 
            {'mode_command': 'FLOWHOLD'} 
        ] 
    ) 
    return LaunchDescription([ 
        drone1_node, 
        drone2_node, 
    ])