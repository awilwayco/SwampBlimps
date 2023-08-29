import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import sys

class Blimp(Node):

    def __init__(self, blimpID):
        self.blimpID = blimpID
        # Define Fake Blimp's name
        self.node_name = str(blimpID)
        # Init node
        super().__init__(self.node_name)

        # Create publisher for /blimpID
        self.pub_blimpID = self.create_publisher(String, self.node_name + '/blimpID', 10)

        # Create subscriber for /autoPanic
        self.sub_identify = self.create_subscription(Bool, '/identify', self.identify_callback, 10)

        # Create timer
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def identify_callback(self, msg):
        self.get_logger().info('Identify: %s' % str(msg.data))

    def timer_callback(self):
        # Publish /blimpID
        self.publish_blimpID()

    def publish_blimpID(self):
        msg = String()
        msg.data = str(self.blimpID)
        self.pub_blimpID.publish(msg)
        self.get_logger().info("Published: %s" % msg.data)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print("Usage: python3 fake_blimp.py node_name")
        return

    node_name = str(sys.argv[1])

    node = Blimp(node_name)

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
