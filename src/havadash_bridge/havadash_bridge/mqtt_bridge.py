"""

Havadash Drone Telemetry - MQTT Bridge Node

FILE: mqtt_bridge.py

DESCRIPTION:
    This node acts as a gateway between the internal ROS 2 network and the 
    external Cloud/IoT infrastructure. It subscribes to high-frequency sensor 
    data, downsamples/buffers it, and publishes it to an MQTT broker using 
    a specific JSON schema.

AUTHOR: Dogukan Avci
DATE: December 2025
"""


# LIBRARY IMPORTS & DEPENDENCIES


# ROS 2 Python Client Library. Handles node lifecycle, threading, and communication.
import rclpy
from rclpy.node import Node

# Standard ROS message types. Using standard types ensures compatibility with
# real hardware drivers (e.g., generic GPS drivers) without modification.
# [Ref: NavSatFix for GPS, BatteryState for Power]
from sensor_msgs.msg import NavSatFix, BatteryState

# Paho MQTT Client. The industry-standard Python library for MQTT communication.
# Used here to push telemetry data to the remote server.
import paho.mqtt.client as mqtt

# JSON Library. Used for Data Serialization.
# Converts Python objects (dicts) into string format required by the API contract.
import json

# Datetime Library. Essential for generating accurate ISO8601 timestamps.
# Timezone awareness (UTC) is critical for synchronizing logs across servers.
from datetime import datetime, timezone


class HavadashBridge(Node):
    """
    A Bridge Node that converts ROS 2 topics into MQTT JSON payloads.
    """

    def __init__(self):
        # Initialize the ROS node with name 'havadash_bridge'
        super().__init__('havadash_bridge')
        
        # -----------------------------------------------------------
        # CONFIGURATION PARAMETERS
        # -----------------------------------------------------------
        # We declare parameters to avoid hard-coding values.
        # This allows changing the broker IP or Drone ID via launch files 
        # without modifying the source code.
        self.declare_parameter('mqtt_broker', 'test.mosquitto.org')
        self.declare_parameter('drone_id', 'test-drone-01')
        
        # Retrieve parameter values
        self.broker_address = self.get_parameter('mqtt_broker').value
        self.drone_id = self.get_parameter('drone_id').value
        
        # -----------------------------------------------------------
        # STATE VARIABLES (Data Buffer)
        # -----------------------------------------------------------
        # These variables act as a buffer. They hold the *latest* received
        # sensor values until the next MQTT publish cycle triggers.
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.current_battery = 0 
        self.current_status = "IN_FLIGHT"
        
        # -----------------------------------------------------------
        # MQTT CLIENT SETUP
        # -----------------------------------------------------------
        # Initializing Paho MQTT Client with API Version 2.
        # Note: CallbackAPIVersion.VERSION2 is required for Paho v2.x compatibility.
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, self.drone_id)
        
        try:
            # Connect to the broker (Port 1883 is standard for non-SSL MQTT)
            self.client.connect(self.broker_address, 1883, 60)
            
            # loop_start() creates a background thread to handle network I/O.
            # This ensures the ROS main loop is not blocked by network latency.
            self.client.loop_start()
            
            self.get_logger().info(f'MQTT Connection Established: {self.broker_address}')
        except Exception as e:
            self.get_logger().error(f'MQTT Connection Failed: {str(e)}')

        # -----------------------------------------------------------
        # ROS SUBSCRIBERS (Inputs)
        # -----------------------------------------------------------
        # Listening to internal sensor topics.
        # queue_size=10 prevents memory overflow if processing is slow.
        self.create_subscription(NavSatFix, 'sensor/gps', self.gps_callback, 10)
        self.create_subscription(BatteryState, 'sensor/battery', self.battery_callback, 10)
        
        # -----------------------------------------------------------
        # PUBLISH TIMER (Outputs)
        # -----------------------------------------------------------
        # According to requirements, telemetry must be sent at 1 Hz.
        # This timer triggers the 'publish_telemetry' function every second.
        self.create_timer(1.0, self.publish_telemetry)

    def gps_callback(self, msg):
        """
        Callback for GPS data. Updates the internal state buffer.
        """
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

    def battery_callback(self, msg):
        """
        Callback for Battery data. Updates the internal state buffer.
        """
        self.current_battery = int(msg.percentage)

    def publish_telemetry(self):
        """
        Constructs the JSON payload and publishes it via MQTT.
        This function runs periodically (1 Hz).
        """
        
        # Construct Payload based on the Frozen JSON Schema [Source: 20]
        payload = {
            "drone_id": self.drone_id,
            # Generate ISO8601 UTC timestamp (e.g., 2025-12-01T12:00:00Z)
            "timestamp": datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
            "lat": self.current_lat,
            "lon": self.current_lon,
            "alt": self.current_alt,
            "speed": 22.5,  # Mock speed value
            "battery": self.current_battery,
            "status": self.current_status
        }
        
        # Define the MQTT topic structure
        topic = f"havadash/telemetry/{self.drone_id}"
        
        try:
            # Serialization: Convert Dictionary -> JSON String
            json_str = json.dumps(payload)
            
            # Publish to Cloud
            self.client.publish(topic, json_str)
            
            self.get_logger().info(f'Telemetry Sent: {json_str}')
        except Exception as e:
            self.get_logger().error(f'Publish Error: {str(e)}')

def main(args=None):
    # Initialize ROS context
    rclpy.init(args=args)
    
    # Create and spin the node
    node = HavadashBridge()
    rclpy.spin(node)
    
    # Clean cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()