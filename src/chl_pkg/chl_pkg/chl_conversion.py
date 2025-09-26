#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial

class ChlConversionNode(Node):
    def __init__(self):
        super().__init__('chl_conversion')

        self.publisher_ = self.create_publisher(Float64, 'chl_data', 10)
        
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',   
            baudrate=19200,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=1
        )

        
        self.dark_counts = 47
        self.scale_factor = 0.0078  # µg/L

        #  datos 40 veces por segundo
        self.timer = self.create_timer(0.025, self.read_and_publish)

    def read_and_publish(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return

            
            parts = line.split()
            if len(parts) < 4:
                return  # línea no válida

            
            counts = int(parts[3])

            
            chl = (counts - self.dark_counts) * self.scale_factor
            if chl < 0:
                chl = 0.0  # evitar negativos si counts < dark_counts

            
            msg = Float64()
            msg.data = chl
            self.publisher_.publish(msg)

            # el print
            self.get_logger().info(f'Counts={counts}, Chl={chl:.3f} µg/L')

        except Exception as e:
            self.get_logger().warn(f'Error reading: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ChlConversionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
