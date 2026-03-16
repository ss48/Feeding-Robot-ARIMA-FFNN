import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand

class JointRefPublisher(Node):

    def __init__(self):
        super().__init__('joint2_ref_pub')
        self.pub = self.create_publisher(
            MultiDOFCommand,
            '/joint2_controller/reference',
            10
        )
        self.timer = self.create_timer(0.02, self.publish_ref)

    def publish_ref(self):
        msg = MultiDOFCommand()
        msg.dof_names = ['joint2']
        msg.values = [0.0]
        msg.values_dot = [0.0]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = JointRefPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

