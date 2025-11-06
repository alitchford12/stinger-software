import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        
        # TODO: 5.1.a Velocity Controller Setup
        ### STUDENT CODE HERE
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10
        )

        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            '/cmd_wrench',
            10
        )


        ### END STUDENT CODE

        # Mock Values, Tune These
        # self.Kp_surge = 1
        # self.Ki_surge = 0
        # self.Kd_surge = 0

        # self.Kp_yaw = 1
        # self.Ki_yaw = 0
        # self.Kd_yaw = 0
        
        # TODO: 5.1.g Controller Tuning
        ### STUDENT CODE HERE
        self.Kp_surge = 1
        self.Ki_surge = 0
        self.Kd_surge = 0

        self.Kp_yaw = 40
        self.Ki_yaw = 0
        self.Kd_yaw = 1
        ### END STUDENT CODE

        self.cmd_vel = Twist()
        self.prev_time = None
        self.prev_error_surge = 0
        self.prev_error_yaw = 0

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = msg

    def odometry_callback(self, msg: Odometry):
        if not self.prev_time:
            self.prev_time = self.get_clock().now()
            return

        output_force = WrenchStamped()
        output_force.header = msg.header
        dt = (self.get_clock().now() - self.prev_time).nanoseconds / 1e9
        self.prev_time = self.get_clock().now()

        error_surge = 0
        # TODO: 5.1.b Error Calculation
        error_surge = self.cmd_vel.linear.x - msg.twist.twist.linear.x
        error_yaw = self.cmd_vel.angular.z - msg.twist.twist.angular.z

        # TODO: 5.1.c Proportional Calculation
        P_surge = self.Kp_surge * error_surge
        P_yaw = self.Kp_yaw * error_yaw
        
        
        # TODO: 5.1.d Integral Calculation
        self.integral_surge = 0
        self.integral_yaw = 0
        if dt > 0:
            self.integral_surge += error_surge * dt
            self.integral_yaw += error_yaw * dt
        I_surge = self.Ki_surge * self.integral_surge
        I_yaw = self.Ki_yaw * self.integral_yaw

        D_surge = 0
        D_yaw = 0
        # TODO: 5.1.e Derivative Calculation
        if (dt >0):
            D_surge = self.Kd_surge * (error_surge - self.prev_error_surge) / dt
            D_yaw = self.Kd_yaw * (error_yaw - self.prev_error_yaw) / dt

        self.prev_error_surge = error_surge
        self.prev_error_yaw = error_yaw
     
        control_surge = P_surge + I_surge + D_surge
        output_force.wrench.force.x = control_surge

        control_yaw = P_yaw + I_yaw + D_yaw
        output_force.wrench.torque.z = control_yaw

        self.wrench_pub.publish(output_force)

        # Ignore this line, only for autograder purposes
        return error_surge, P_surge, I_surge, D_surge, control_yaw, output_force


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
