from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy


class Safety(Node):
    
    def __init__(self):
        super().__init__('safety')
        
        # variables
        self.r2_min_ = -1.0
        
        self.ack_brake_msg_ = AckermannDriveStamped()
        self.ack_brake_msg_.drive.speed = 0.0
        
        # publishers
        self.ack_cmd_pub_ = self.create_publisher(AckermannDriveStamped,
                                                  'ackermann_cmd', 1)
        
        # subscribers
        self.joy_sub_ = self.create_subscription(Joy, 'joy', self.joy_cb, 1)

    def joy_cb(self, msg):
        # R2 button will be used for braking
        # it's value is on msg.axes[5], and takes on values in range [-1, 1]
        # msg.axes[5] in (-1, 1] -> button unpressed -> no brake
        # msg.axes[5] == -1 -> button fully pressed -> apply brake
        if msg.axes[5] == self.r2_min_:
            self.ack_cmd_pub_.publish(self.ack_brake_msg_)
        # TODO: should we should maintain the steering angle or set to zero

