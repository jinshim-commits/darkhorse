import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

class Nav2SpeedManager(Node):
    def __init__(self):
        super().__init__('nav2_speed_manager')
        
        # '/limo_speed_cmd' 토픽 구독 (KeyboardRemote와 이름 일치)
        self.subscription = self.create_subscription(
            String,
            '/limo_speed_cmd',
            self.listener_callback,
            10)
        
        self.current_max_speed = 0.5
        self.param_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.get_logger().info('✅ Speed Manager Ready. Waiting for +/- keys...')

    def listener_callback(self, msg):
        cmd = msg.data
        if cmd == "UP":
            self.current_max_speed += 0.1
        elif cmd == "DOWN":
            self.current_max_speed -= 0.1
        elif cmd == "STOP":
            self.current_max_speed = 0.0
            
        # 속도 범위 제한 (0.0 ~ 1.0 m/s)
        self.current_max_speed = max(0.0, min(self.current_max_speed, 1.0))
        
        self.apply_speed()

    def apply_speed(self):
        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name='FollowPath.max_vel_x', 
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.current_max_speed)
            )
        ]
        if self.param_client.service_is_ready():
            self.param_client.call_async(req)
            self.get_logger().info(f"Set Speed to: {self.current_max_speed:.1f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = Nav2SpeedManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()