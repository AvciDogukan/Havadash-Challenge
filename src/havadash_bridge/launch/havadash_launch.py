"""
Launch file for the Havadash Firmware Engineering Challenge.

This script orchestrates the startup of the Havadash Telemetry System.
It is responsible for launching the simulation layer (Mock Publisher) 
and the communication layer (MQTT Bridge) simultaneously.

FILE: havadash_launch.py

USAGE:
    ros2 launch havadash_bridge havadash_launch.py

NODES LAUNCHED:
    1. mock_publisher (havadash_bridge): 
       Simulates GPS (NavSatFix) and Battery (BatteryState) sensors.
    
    2. mqtt_bridge (havadash_bridge): 
       Subscribes to ROS topics and publishes JSON telemetry to MQTT Broker.

AUTHOR: Dogukan Avci
DATE: December 2025
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the LaunchDescription for the Havadash bridge package.

    This function defines the nodes to be executed and their configurations.
    
    Returns:
        LaunchDescription: A collection of entities (Nodes) to be launched
        by the ROS 2 launch system.
    """
    return LaunchDescription([
        
        # ---------------------------------------------------------
        # Node 1: Mock Data Publisher (Simulation Layer)
        # ---------------------------------------------------------
        # Since physical sensors are not available, this node generates 
        # synthetic GPS (NavSatFix) and Battery (BatteryState) data.
        # It mimics the behavior of hardware drivers.
        Node(
            package='havadash_bridge',
            executable='mock_publisher',
            name='mock_publisher',
            output='screen' 
            # 'output=screen' ensures that log messages (INFO, WARN, ERROR)
            # are printed to the console for real-time monitoring.
        ),

        # ---------------------------------------------------------
        # Node 2: MQTT Telemetry Bridge (Communication Layer)
        # ---------------------------------------------------------
        # This node acts as a gateway between the internal ROS 2 network 
        # and the external Cloud/IoT infrastructure.
        # It subscribes to sensor topics, serializes data to JSON, 
        # and transmits it via MQTT protocol.
        Node(
            package='havadash_bridge',
            executable='mqtt_bridge',
            name='mqtt_bridge',
            output='screen',
            parameters=[
                # Configuration parameters allow changing the target broker
                # and device ID without modifying the source code.
                # These default values target the public test broker.
                {'mqtt_broker': 'test.mosquitto.org'},
                {'drone_id': 'test-drone-01'}
            ]
        ),
    ])