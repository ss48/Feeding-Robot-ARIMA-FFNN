import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import csv
import os
from time import time

class PIDRecorder(Node):

    def __init__(self):
        super().__init__('pid_data_recorder')

        self.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']

        self.latest_cmd = {j: None for j in self.joint_names}
        self.latest_pos = {j: None for j in self.joint_names}

        self.start_time = time()

        log_path = os.path.expanduser('~/ros2_feedbot_ws/src/pid_logger/logs/tuned_data.csv')
        self.file = open(log_path,'w')
        self.writer = csv.writer(self.file)

        header = ['time']
        for j in self.joint_names:
            header += [f'{j}_desired', f'{j}_actual', f'{j}_error']
        self.writer.writerow(header)

        self.create_subscription(JointState,'/joint_states',self.joint_state_cb,10)

        self.create_subscription(Float64,'/joint1_controller/command',lambda msg: self.cmd_cb(msg,'joint1'),10)
        self.create_subscription(Float64,'/joint2_controller/command',lambda msg: self.cmd_cb(msg,'joint2'),10)
        self.create_subscription(Float64,'/joint3_controller/command',lambda msg: self.cmd_cb(msg,'joint3'),10)
        self.create_subscription(Float64,'/joint4_controller/command',lambda msg: self.cmd_cb(msg,'joint4'),10)
        self.create_subscription(Float64,'/joint5_controller/command',lambda msg: self.cmd_cb(msg,'joint5'),10)
        self.create_subscription(Float64,'/joint6_controller/command',lambda msg: self.cmd_cb(msg,'joint6'),10)

        self.timer = self.create_timer(0.01,self.record_data)

    def cmd_cb(self,msg,joint):
        self.latest_cmd[joint] = msg.data

    def joint_state_cb(self,msg):
        for i,name in enumerate(msg.name):
            if name in self.latest_pos:
                self.latest_pos[name] = msg.position[i]

    def record_data(self):
        t = time() - self.start_time
        row = [t]

        for j in self.joint_names:
            d = self.latest_cmd[j]
            a = self.latest_pos[j]

            if d is None or a is None:
                row += ['','','']
            else:
                row += [d,a,d-a]

        self.writer.writerow(row)

def main(args=None):
    rclpy.init(args=args)
    node = PIDRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
