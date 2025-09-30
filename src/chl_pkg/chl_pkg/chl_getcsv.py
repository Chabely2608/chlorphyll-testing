#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import csv
import os
#from builtin_interfaces.msg import Time

class ChlCSV(Node):
    def __init__(self):
        super().__init__('chl_csv')

        # Subs
        self.create_subscription(NavSatFix, 'fix', self.gps_callback, 10)
        self.create_subscription(Float64, 'chl_data', self.chl_callback, 10)
        self.last_gps = None  

        self.filepath = os.path.join(os.getcwd(), "chl_data_log.csv")
        file_exists = os.path.isfile(self.filepath)

        self.csv_file = open(self.filepath, 'a', newline='')
        self.writer = csv.writer(self.csv_file)

        
        if not file_exists:
            self.writer.writerow(["time", "latitude", "longitude", "chl_value"])

        self.get_logger().info(f"Guardando datos en {self.filepath}")

    def gps_callback(self, msg: NavSatFix):
        #keep last msg from gps
        self.last_gps = msg

    def chl_callback(self, msg: Float64):
        if self.last_gps is None:
            self.get_logger().warn("No hay datos de GPS todavía, ignorando chl_data")
            return
    
        stamp: Time = self.last_gps.header.stamp
        time_sec = stamp.sec + stamp.nanosec * 1e-9

        row = [
            time_sec,
            self.last_gps.latitude,
            self.last_gps.longitude,
            msg.data
        ]

        self.writer.writerow(row)
        self.csv_file.flush()  
        self.get_logger().info(f"Fila guardada: {row}")

    def destroy_node(self):
        
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ChlCSV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
