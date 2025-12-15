import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Bool, String
import tkinter as tk
from tkinter import font
import threading

class LimoControlUI(Node):
    def __init__(self, root):
        super().__init__('limo_control_ui')
        self.root = root
        self.root.title("LIMO 관제 시스템 🏥")
        self.root.geometry("450x650")
        self.root.configure(bg="#ecf0f1")

        # 1. Nav2 파라미터 클라이언트 (속도 조절용)
        self.client_controller_set = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.client_smoother_set = self.create_client(SetParameters, '/velocity_smoother/set_parameters')
        self.client_controller_get = self.create_client(GetParameters, '/controller_server/get_parameters')

        # 2. 비상 & 상태 통신
        self.pub_emergency = self.create_publisher(Bool, '/emergency_trigger', 10)
        self.create_subscription(String, '/hospital/nav_status', self.update_status, 10)

        # 변수 초기화
        self.current_speed = 0.4 
        self.min_speed = 0.1
        self.max_speed = 1.0
        self.is_emergency = False
        self.current_status_text = "시스템 대기 중..."

        # 3. Nav2와 속도 동기화
        self.sync_initial_speed()
        self.create_widgets()

    def sync_initial_speed(self):
        if self.client_controller_get.wait_for_service(timeout_sec=1.0):
            req = GetParameters.Request()
            req.names = ['FollowPath.max_vel_x']
            future = self.client_controller_get.call_async(req)
            future.add_done_callback(self._sync_callback)
        else:
            self.get_logger().warn("Nav2 not ready. Using default speed 0.4")

    def _sync_callback(self, future):
        try:
            result = future.result()
            if result.values:
                real_speed = result.values[0].double_value
                self.current_speed = round(real_speed, 2)
                self.root.after(0, lambda: self.lbl_speed_val.config(text=f"현재 설정: {self.current_speed:.1f} m/s"))
                self.get_logger().info(f"🔄 Synced speed: {self.current_speed} m/s")
        except Exception: pass

    def create_widgets(self):
        # 헤더
        header_frame = tk.Frame(self.root, bg="#2c3e50", pady=20)
        header_frame.pack(fill="x")
        tk.Label(header_frame, text="Current Status", bg="#2c3e50", fg="#bdc3c7").pack()
        self.lbl_status = tk.Label(header_frame, text=self.current_status_text, 
                                   bg="#2c3e50", fg="#f1c40f", font=("Gothic", 20, "bold"))
        self.lbl_status.pack(pady=10)

        # 속도 제어
        control_frame = tk.LabelFrame(self.root, text="🚀 이동 속도 제어", bg="#ecf0f1", font=("Arial", 12, "bold"), padx=20, pady=20)
        control_frame.pack(pady=20, padx=20, fill="x")
        self.lbl_speed_val = tk.Label(control_frame, text=f"현재 설정: {self.current_speed:.1f} m/s", bg="#ecf0f1", font=("Arial", 14))
        self.lbl_speed_val.pack(pady=(0, 15))

        btn_frame = tk.Frame(control_frame, bg="#ecf0f1")
        btn_frame.pack()
        tk.Button(btn_frame, text="🐢 감속 (-0.1)", command=lambda: self.change_speed(-0.1),
                  bg="#95a5a6", fg="white", font=("Arial", 11, "bold"), height=2, width=14).pack(side="left", padx=5)
        tk.Button(btn_frame, text="🐇 가속 (+0.1)", command=lambda: self.change_speed(0.1),
                  bg="#3498db", fg="white", font=("Arial", 11, "bold"), height=2, width=14).pack(side="right", padx=5)

        # 비상 버튼
        emergency_frame = tk.Frame(self.root, bg="#ecf0f1")
        emergency_frame.pack(side="bottom", fill="x", pady=30, padx=20)
        self.btn_emergency = tk.Button(emergency_frame, text="🚨 비상 정지 (EMERGENCY)", command=self.toggle_emergency,
                                       bg="#c0392b", fg="white", font=("Arial", 18, "bold"), height=3, relief="raised", borderwidth=5)
        self.btn_emergency.pack(fill="x")
        tk.Label(emergency_frame, text="※ 누르면 즉시 안내데스크로 복귀합니다.", bg="#ecf0f1", fg="#7f8c8d").pack(pady=5)

    def change_speed(self, delta):
        new_speed = self.current_speed + delta
        new_speed = max(self.min_speed, min(new_speed, self.max_speed))
        self.current_speed = round(new_speed, 2)
        self.lbl_speed_val.config(text=f"현재 설정: {self.current_speed:.1f} m/s")
        
        self._set_nav2_param('/controller_server', 'FollowPath.max_vel_x', self.current_speed)
        self._set_nav2_param('/velocity_smoother', 'max_velocity', [self.current_speed, 0.0, 1.0])

    def _set_nav2_param(self, node_name, param_name, value):
        threading.Thread(target=self._call_service_thread, args=(node_name, param_name, value), daemon=True).start()

    def _call_service_thread(self, node_name, param_name, value):
        client = self.create_client(SetParameters, f'{node_name}/set_parameters')
        if not client.wait_for_service(timeout_sec=1.0): return
        req = SetParameters.Request()
        p = Parameter()
        p.name = param_name
        if isinstance(value, list):
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[float(x) for x in value])
        else:
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        req.parameters = [p]
        client.call_async(req)

    def toggle_emergency(self):
        self.is_emergency = not self.is_emergency
        msg = Bool()
        msg.data = self.is_emergency
        self.pub_emergency.publish(msg)
        if self.is_emergency:
            self.btn_emergency.config(text="🔄 상황 종료 (초기화)", bg="#f39c12")
            self.lbl_status.config(text="🚨 비상 복귀 중!", fg="#e74c3c")
        else:
            self.btn_emergency.config(text="🚨 비상 정지 (EMERGENCY)", bg="#c0392b")
            self.lbl_status.config(text="대기 중 (Idle)", fg="#f1c40f")

    def update_status(self, msg):
        if not self.is_emergency:
            self.current_status_text = msg.data
            self.lbl_status.config(text=self.current_status_text, fg="#f1c40f")

def main():
    rclpy.init()
    root = tk.Tk()
    app = LimoControlUI(root)
    ros_thread = threading.Thread(target=rclpy.spin, args=(app,), daemon=True)
    ros_thread.start()
    root.mainloop()
    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()