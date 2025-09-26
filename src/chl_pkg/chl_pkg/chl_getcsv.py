#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import csv
import os
import time

class ChlCSV(Node):
    def __init__(self):
        super().__init__('chl_csv')
        
        self.create_subscription(Float64, 'chl_data', self.chl_callback, 10)

        # Archivo CSV
        self.filepath = os.path.join(os.getcwd(), "chl_data_log.csv")
        file_exists = os.path.isfile(self.filepath)

        self.csv_file = open(self.filepath, 'a', newline='')
        self.writer = csv.writer(self.csv_file)

        # Encabezado si el archivo es nuevo
        if not file_exists:
            self.writer.writerow(["Time (s)", "Chl μg/l"])

    def chl_callback(self, msg: Float64):
        # Tiempo del sistema (epoch en segundos con decimales)
        time_sec = time.time()

        row = [time_sec, msg.data]

        self.writer.writerow(row)
        self.csv_file.flush()  # escritura inmediata
        self.get_logger().info(f"Saved row: {row}")

    def destroy_node(self):
        # Cerrar archivo al destruir el nodo
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
