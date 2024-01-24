import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class dummyTarget(Node):

    def __init__(self):
        super().__init__('target')
        self.publisher_ = self.create_publisher(NavSatFix, 'robot2/target', 1)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = NavSatFix()

        lat = 37.35235604994516
        lon = -121.94153221539479
        if lat is not None and lon is not None:
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = 0.0
            msg.status.status = 1
            msg.status.service = 1
            msg.header.frame_id = 'target'
        else:
            msg.status.status = 0

        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    pushTarget = dummyTarget()

    rclpy.spin(pushTarget)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pushTarget.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

