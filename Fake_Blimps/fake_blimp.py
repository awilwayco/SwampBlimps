import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int64, Float64, Float64MultiArray
import sys
import random

class Blimp(Node):
    def __init__(self, blimp_id):

        # Blimp ID
        self.blimp_id = blimp_id

        # State Machine
        self.state_machine = 0

        # Define Fake Blimp's name
        self.node_name = str(blimp_id)

        # Init node
        super().__init__(self.node_name, namespace=self.node_name)

        # Create publisher for /state_machine
        self.pub_identify = self.create_publisher(String, '/identify', 10)
        self.pub_state_machine = self.create_publisher(Int64, 'state_machine', 10)
        self.pub_height = self.create_publisher(Float64, 'height', 10)
        self.pub_z_vel = self.create_publisher(Float64, 'z_velocity', 10)
        self.pub_log = self.create_publisher(String, 'log', 10)

        topic_auto = "auto"
        topic_goal_color = "goal_color"
        topic_target_color = "target_color"
        topic_motor_commands = "motorCommands"
        topic_grabbing = "grabbing"
        topic_shooting = "shooting"
        topic_baseBarometer = "baseBarometer"
        topic_calibrateBarometer = "calibrateBarometer"

        self.sub_auto = self.create_subscription(Bool, topic_auto, self.auto_callback, 10)
        self.sub_goal_color = self.create_subscription(Int64, topic_goal_color, self.goal_callback, 10)
        self.sub_target_color = self.create_subscription(Int64, topic_target_color, self.target_callback, 10)
        self.sub_motor_commands = self.create_subscription(Float64MultiArray, topic_motor_commands, self.motor_callback, 10)
        self.sub_grabbing = self.create_subscription(Bool, topic_grabbing, self.grab_callback, 10)
        self.sub_shooting = self.create_subscription(Bool, topic_shooting, self.shoot_callback, 10)
        self.sub_baseBarometer = self.create_subscription(Float64, topic_baseBarometer, self.base_baro_callback, 10)
        self.sub_calibrate_barometer = self.create_subscription(Bool, topic_calibrateBarometer, self.calibrate_callback, 10)

        # Create timer
        id_timer_period = 1  # seconds
        self.id_timer = self.create_timer(id_timer_period, self.id_timer_callback)

        data_timer_period = 1/20.0  # seconds
        self.data_timer = self.create_timer(data_timer_period, self.data_timer_callback)

    def id_timer_callback(self):
        # Publish /blimpID
        #self.publish_blimpID()

        # Publish /state_machine
        self.state_machine = round(random.random()*8)
        self.publish_state_machine()

        msg = String()
        msg.data = str(self.blimp_id)
        self.pub_identify.publish(msg)

    def data_timer_callback(self):
        height = 10*random.random()
        z_vel = 5*random.random()

        height_msg = Float64()
        height_msg.data = height

        z_vel_msg = Float64()
        z_vel_msg.data = z_vel

        self.pub_height.publish(height_msg)
        self.pub_z_vel.publish(z_vel_msg)

    def auto_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Autonomous set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

        self.get_logger().info(log_msg.data)

    def goal_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Goal color set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

        self.get_logger().info(log_msg.data)

    def target_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Target color set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

        self.get_logger().info(log_msg.data)

    def motor_callback(self, msg):
        m = msg.data
        self.get_logger().info('{},{},{},{}'.format(m[0], m[1], m[2], m[3]))

    def grab_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Grab set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

    def shoot_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Shoot set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

    def base_baro_callback(self, msg):
        pass

    def calibrate_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Calibrate set to {}'.format(msg.data)

        self.pub_log.publish(log_msg)

    def publish_state_machine(self):
        msg = Int64()
        msg.data = self.state_machine
        self.pub_state_machine.publish(msg)

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
