import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

import serial
import struct

ser = serial.Serial('/dev/ttyUSB0', baudrate=230400, timeout=1)

def get_info():
    rd = False
    Dots = []
    while not rd:
        bite = ser.read(1)
        while bite[0] != 0x54:
            bite = ser.read(1)
        
        dt_len = ser.read(1)
        while dt_len[0] != 0x2C:
            dt_len = ser.read(1)
        
        dt_len = dt_len[0] & 0x1F 
        bite = ser.read(45)
        
        #speed = struct.unpack('<H', bite[0:2])[0]
        start_ang = (struct.unpack('<H', bite[2:4])[0]/100)
        end_ang = (struct.unpack('<H', bite[40:42])[0]/100) 
        if abs(start_ang - end_ang) < 60: 
            for i in range(0, dt_len, 4):
                distance = struct.unpack('<H', bite[4+(3*i):6+(3*i)])[0]
                intensivity = bite[6+(3*i)]
                ang = start_ang + (((end_ang-start_ang)/(dt_len-1))*i)
                Dots.append([distance, intensivity, ang])

        if len(Dots) > 138:
            rd = True
    return Dots

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_pub')
        self.pub = self.create_publisher(Float32MultiArray, '/lidar_dots', 10)
        self.timer = self.create_timer(0.25, self.pub_dots)

    def pub_dots(self):
        dots = get_info()
        if not dots:
            return
        
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label="points", size=len(dots), stride=len(dots)*3))
        msg.layout.dim.append(MultiArrayDimension(label="xyz", size=3, stride=3))
        msg.data = [item for sublist in dots for item in sublist]

        self.pub.publish(msg)
        self.get_logger().info(f'Опубликовано {len(dots)} точек')

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
