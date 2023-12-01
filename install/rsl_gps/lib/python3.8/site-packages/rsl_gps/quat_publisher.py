import time
import board
import adafruit_bno055
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import  Quaternion

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)

# If you are going to use UART uncomment these lines
# uart = board.UART()
# sensor = adafruit_bno055.BNO055_UART(uart)

last_val = 0xFFFF

class QuatPublisher(Node):

    def __init__(self):
        super().__init__('quat_publisher')
        self.publisher_ = self.create_publisher(Quaternion, 'imu_quat', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        quat_tf = sensor.quaternion
        msg_quat = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
        #print(msg_quat)
        msg = Quaternion()
        msg = msg_quat
        self.publisher_.publish(msg)
        #print(msg)
        self.i += 1

    
def main(args=None):
    rclpy.init(args=args)

    quat_publisher = QuatPublisher()

    rclpy.spin(quat_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    quat_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
