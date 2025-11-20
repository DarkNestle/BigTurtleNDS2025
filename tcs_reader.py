import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Float32

import serial
import struct

ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)

class TcsReader(Node):
    def __init__(self):
        super().__init__('tcs_reader')
        self.pub = self.create_publisher(
            Float32,
            '/tcs_reader',
            10)
        self.timer = self.create_timer(0.5, self.pub_clr)
    
    def pub_clr(self):
        clr = ser.readline().decode().strip()
        msg = Float32()
        if clr:
            clr = int(float(clr))
            if clr < 100:
                data = 111
            elif clr < 200:
                data = 222
            else:
                data = 333
            
            msg.data = data
            self.get_logger().info(f'Опубликовал команду {msg.data}, цвет {clr}') 
            self.publish(msg)
        else:
           return

        
        

def main(args=None):
    rclpy.init(args=args)
    node = TcsReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
