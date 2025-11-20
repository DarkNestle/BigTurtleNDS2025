import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int32

import serial
import struct
import numpy as np
from time import sleep

drt = {
    1: [160, 200],
    2: [250, 290],
    3: [70, 110]
}
last_msg = None

class ArdInf(Node):
    def __init__(self):
        super().__init__('ard_cmd')

        self.ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)
        sleep(2)

        self.sub = self.create_subscription(
            Float32MultiArray,
            "/lidar_dots",
            self.mtr_mv,
            10)
        '''
        self.tcs_pub = self.create_subscription(
            Int32,
            '/tcs_reader',
            self.tcs,
            10)
        '''
        

    def mtr_mv(self, msg):
        global last_msg
        dots = msg.data.tolist()
        Dist = dots[0::3]
        Ang = dots[2::3]
        rd = False
        Temp = []
        DistR = []
        DistL = []
        AngR = []
        AngL = []
        rd = False
        nap = 0

        for i in range(3):
            Grd = drt.get(i+1)
            for j in range(len(Ang)-1, -1, -1):
                if Ang[j] > Grd[0] and Ang[j] < Grd[1]:
                    if i == 2:
                        DistR.append(Dist[j])
                        AngR.append(Ang[j])
                    elif i == 1:
                        DistL.append(Dist[j])
                        AngL.append(Ang[j])
                    if Dist[j] < 200:
                        del Ang[j]
                        del Dist[j]
                    else:
                        Temp.append(Dist[j])
                        if not rd:
                            nap = i+1
                            rd = True
            
        if len(Temp) == 0:
            nap = 2

        if last_msg == 2 and nap == 1:
            nap = self.balance(DistR, AngR, 3)
        elif last_msg == 3 and nap == 1:
            nap = self.balance(DistL, AngL, 2)

        self.send_cmd(str(nap), np.array(Temp).mean())
        last_msg = nap
    
    def balance(self, dist, ang, nap):
        global last_msg
        nap = drt.get(nap)
        size1 = []
        size2 = []
        ang1 = []
        ang2 = []
        for i in range(len(ang)-1, -1, -1):
            if ang[i] > nap[0] and ang[i] < nap[0]+10:
                size1.append(dist.pop(i))
                ang1.append(ang.pop(i))
            elif ang[i] > nap[1]-10 and ang[i] < nap[1]:
                size2.append(dist.pop(i))
                ang2.append(ang.pop(i))
        size = np.array(dist).mean()
        size1 = np.array(size1).mean()
        size2 = np.array(size2).mean()
        self.get_logger().info(f'Балансировка, {abs(size1-size2)}')
        if abs(size1-size2) < 10 and abs(size-size1) < 10 and abs(size-size2) < 10:
            return 1
        else:
            return last_msg
    '''
    def tcs(self, msg):
        move(msg.data)
        
    def move(arg=None, msg):
        try:    
            if msg == 111:
                pass
            elif msg == 222:
                pass
            else:
                pass
        except:
            Dist, Intn, Ang = msg[0], msg[1], msg[2]
    '''
    
    def send_cmd(self, cmd, dist):
        cmd = cmd + "\n"
        self.ser.write((cmd.encode()))
        self.get_logger().info(f'Послал информацию {cmd}, расстояние: {dist}')

def main(args=None):
    rclpy.init(args=args)
    ard = ArdInf()
    rclpy.spin(ard)
    ard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
