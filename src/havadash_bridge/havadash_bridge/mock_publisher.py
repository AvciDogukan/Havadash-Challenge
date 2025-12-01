"""

Havadash Drone Telemetry - Mock Sensor Node

FILE: mock_publisher.py

DESCRIPTION:
    This node simulates hardware sensors for the Havadash drone project.
    Since physical hardware is not available, it generates synthetic 
    GPS (NavSatFix) and Battery (BatteryState) data and publishes it 
    to the ROS 2 network.

AUTHOR: Dogukan Avci
DATE: December 2025
"""

import rclpy
from rclpy.node import Node
import random

# Standard ROS message types for hardware abstraction
from sensor_msgs.msg import NavSatFix, BatteryState

class MockPublisher(Node):
    """
    A ROS 2 Node that generates and publishes mock sensor data.
    """

    def __init__(self):
        # Initialize the node with the name 'mock_publisher'
        super().__init__('mock_publisher')
        
        # -----------------------------------------------------------
        # PUBLISHERS
        # -----------------------------------------------------------
        # Create publishers for GPS and Battery topics.
        # Queue size is set to 10 (QoS) to buffer messages if the network is busy.
        self.gps_pub = self.create_publisher(NavSatFix, 'sensor/gps', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'sensor/battery', 10)
        
        # -----------------------------------------------------------
        # TIMERS
        # -----------------------------------------------------------
        # Set the publication frequency to 1 Hz (once per second).
        # This triggers 'timer_callback' periodically.
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # -----------------------------------------------------------
        # STATE VARIABLES (Initial Conditions)
        # -----------------------------------------------------------
        # Starting coordinates (Approx. Istanbul location)
        self.lat = 41.0123
        self.lon = 29.0567
        self.battery = 100.0
        
        self.get_logger().info('Mock Data Generator Started...')

    def timer_callback(self):
        """
        Main loop triggered by the timer.
        Generates synthetic data and publishes it to topics.
        """
        
        # ===========================================================
        # 1. GENERATE GPS DATA
        # ===========================================================
        gps_msg = NavSatFix()
        
        # Timestamp is crucial for sensor fusion and logging
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        # 'gps_link' defines the reference frame on the robot
        gps_msg.header.frame_id = "gps_link"
        
        # Simulate movement (Random Walk Logic)
        # Add small random variations to coordinates to mimic flight.
        self.lat += random.uniform(-0.0001, 0.0001)
        self.lon += random.uniform(-0.0001, 0.0001)
        
        gps_msg.latitude = self.lat
        gps_msg.longitude = self.lon
        # Simulate altitude fluctuation around 50 meters
        gps_msg.altitude = 50.0 + random.uniform(-2.0, 2.0)
        
        # Publish the message to 'sensor/gps'
        self.gps_pub.publish(gps_msg)
        
        # ===========================================================
        # 2. GENERATE BATTERY DATA
        # ===========================================================
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Simulate battery discharge (Linear decay)
        if self.battery > 0:
            self.battery -= 0.1
            
        battery_msg.percentage = self.battery
        # Simulate voltage drop based on percentage
        battery_msg.voltage = 12.0 + (self.battery / 100.0) 
        
        # Publish the message to 'sensor/battery'
        self.battery_pub.publish(battery_msg)
        
        # Log to console for real-time debugging
        self.get_logger().info(f'Publishing -> Lat: {self.lat:.4f}, Bat: {self.battery:.1f}%')

def main(args=None):
    # Initialize ROS 2 communication
    rclpy.init(args=args)
    
    # Create the node instance
    node = MockPublisher()
    
    try:
        # Keep the node running and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle graceful shutdown on Ctrl+C
        pass
    finally:
        # Clean up resources
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()