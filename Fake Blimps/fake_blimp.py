import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int64
import sys

class Blimp(Node):

    def __init__(self, blimpID):

        # Blimp ID
        self.blimpID = blimpID

        # State Machine
        self.state_machine = 0

        # Define Fake Blimp's name
        self.node_name = str(blimpID)

        # Init node
        super().__init__(self.node_name)

        # Create publisher for /blimpID
        self.pub_blimpID = self.create_publisher(String, self.node_name + '/blimpID', 10)

        # Create publisher for /state_machine
        self.pub_state_machine = self.create_publisher(Int64, self.node_name + '/state_machine', 10)

        # Create subscriber for /autoPanic
        self.pub_identify = self.create_publisher(String, '/identify', 10)

        # Create timer
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def identify_callback(self):
        self.get_logger().info('Identify: %s' % str(msg.data))
        msg = String()
        msg.data = str(self.blimpID)
        self.pub_identify.publish(msg)

    def timer_callback(self):
        # Publish /blimpID
        self.publish_blimpID()

        # Publish /state_machine
        self.publish_state_machine()

        msg = String()
        msg.data = str(self.blimpID)
        self.pub_identify.publish(msg)

    def publish_blimpID(self):
        msg = String()
        msg.data = str(self.blimpID)
        self.pub_blimpID.publish(msg)
        self.get_logger().info("Published: %s" % msg.data)

    def publish_state_machine(self):
        msg = Int64()
        msg.data = self.state_machine
        self.pub_state_machine.publish(msg)
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
