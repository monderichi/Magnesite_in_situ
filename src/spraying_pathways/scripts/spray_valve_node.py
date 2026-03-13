#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import serial
import time
import threading

class SprayValveNode(Node):
    def __init__(self):
        super().__init__('spray_valve_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0') # Default to ACM1 as ACM0 might be mycobot
        self.declare_parameter('baud', 115200)
        
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        
        self.get_logger().info(f"Connecting to Arduino on {self.port} at {self.baud} baud...")
        
        self.ser = None
        self.valve_state = False # False = Closed, True = Open
        self.lock = threading.Lock()
        
        self._connect()

        # Subscription to control the valve
        self.subscription = self.create_subscription(
            Bool,
            'spray_valve',
            self.valve_callback,
            10)
        
        # Service to toggle the valve
        self.srv = self.create_service(Trigger, 'toggle_spray_valve', self.toggle_valve_callback)
        
        self.get_logger().info("Spray Valve Node started.")
        self.get_logger().info(" - Topic: /spray_valve (std_msgs/Bool)")
        self.get_logger().info(" - Service: /toggle_spray_valve (std_srvs/Trigger)")

    def _connect(self):
        try:
            with self.lock:
                self.ser = serial.Serial(self.port, self.baud, timeout=1)
                time.sleep(2) # Wait for Arduino to reset
                self.get_logger().info(f"Successfully connected to Arduino on {self.port}!")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino on {self.port}: {e}")
            self.ser = None

    def _write_state(self, state):
        """Write the valve state to the Arduino."""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Serial communication not established. Attempting to reconnect...")
            self._connect()
            if self.ser is None:
                return False

        try:
            with self.lock:
                if self.ser is None:
                    return False
                if state:
                    self.ser.write(b'1')
                    self.get_logger().info("Command sent: OPEN VALVE (1)")
                else:
                    self.ser.write(b'0')
                    self.get_logger().info("Command sent: CLOSE VALVE (0)")
                self.ser.flush()
                self.valve_state = state
                return True
        except Exception as e:
            self.get_logger().error(f"Error writing to serial port: {e}")
            self.ser = None # Mark as disconnected
            return False

    def valve_callback(self, msg):
        self._write_state(msg.data)

    def toggle_valve_callback(self, request, response):
        new_state = not self.valve_state
        success = self._write_state(new_state)
        
        response.success = success
        if success:
            response.message = f"Valve {'opened' if new_state else 'closed'}"
        else:
            response.message = "Failed to communicate with Arduino"
        
        return response

    def destroy_node(self):
        if self.ser is not None:
            try:
                with self.lock:
                    if self.ser is not None and self.ser.is_open:
                        self.ser.write(b'0') # Ensure valve is closed on shutdown
                        self.ser.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SprayValveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
