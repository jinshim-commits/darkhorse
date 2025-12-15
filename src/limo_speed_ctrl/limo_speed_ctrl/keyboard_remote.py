import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardRemote(Node):
    def __init__(self):
        super().__init__('keyboard_remote')
        
        # 퍼블리셔 설정
        self.speed_pub = self.create_publisher(String, '/limo_speed_cmd', 10)
        self.emerg_pub = self.create_publisher(Bool, '/emergency_trigger', 10)
        self.start_pub = self.create_publisher(Bool, '/start_trigger', 10) # [추가] BT 시작 트리거용

        # 상태 변수
        self.is_emergency = False 

        self.get_logger().info("""
        🎮 LIMO Keyboard Controller
        ------------------------------------
        [Space] : 🚨 비상 상황 발생 (안내데스크로 복귀)
        [g]     : 🟢 출발 신호 (WaitForStart 트리거)
        [+]     : 🚀 속도 증가
        [-]     : 🐢 속도 감소
        [q]     : 종료
        ------------------------------------
        """)

    def run(self):
        try:
            while rclpy.ok():
                key = get_key()
                
                if key == '+' or key == '=':
                    self.publish_speed("UP")
                        
                elif key == '-' or key == '_':
                    self.publish_speed("DOWN")
                        
                elif key == ' ': # Space Bar (비상 복귀)
                    self.trigger_emergency()
                    
                elif key == 'g' or key == 'G':
                    self.publish_start()
                        
                elif key == 'q':
                    break
                    
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except Exception as e:
            print(e)
        finally:
            self.publish_speed("STOP")

    def publish_speed(self, cmd):
        msg = String()
        msg.data = cmd
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Speed Cmd: {cmd}')

    def trigger_emergency(self):
        # [수정됨] 토글 방식 제거 -> 누르면 즉시 비상모드 발동 (BT가 처리)
        self.is_emergency = True
        
        # 1. BT에게 비상 신호 전송 -> BT가 GoToInfoDesk 실행함
        self.emerg_pub.publish(Bool(data=True))

        # 2. [중요] 속도 정지 명령(STOP) 제거! 
        # 이걸 보내면 속도가 0이 되어 안내데스크로 이동을 못합니다.
        # self.speed_pub.publish(String(data="STOP")) <--- 삭제됨
        
        self.get_logger().warn('🚨 EMERGENCY TRIGGERED! (안내데스크로 이동합니다)')

    def publish_start(self):
        # XML의 WaitForStart 노드를 위해 신호 전송
        self.start_pub.publish(Bool(data=True))
        self.get_logger().info('🟢 Start Signal Sent (Go to Waypoint)')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardRemote()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()