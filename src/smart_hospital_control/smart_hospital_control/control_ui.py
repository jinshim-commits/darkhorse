import tkinter as tk
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped


class ControlUI(Node):
    def __init__(self):
        super().__init__('hospital_control_ui')

        # pub
        self.pub_target = self.create_publisher(String, '/dispatch_target', 10)
        self.pub_speed  = self.create_publisher(Float32, '/nav_speed_delta', 10)
        self.pub_emg    = self.create_publisher(Bool, '/nav_emergency', 10)

        # sub
        self.status = "IDLE"
        self.target = "-"
        self.speed  = 0.0
        self.goal_xy = "-"

        self.create_subscription(String, '/nav_status', self.cb_status, 10)
        self.create_subscription(String, '/nav_current_target', self.cb_target, 10)
        self.create_subscription(Float32, '/nav_current_speed', self.cb_speed, 10)
        self.create_subscription(PoseStamped, '/nav_goal_pose', self.cb_goal, 10)

    def cb_status(self, msg): self.status = msg.data
    def cb_target(self, msg): self.target = msg.data
    def cb_speed(self, msg):  self.speed = float(msg.data)
    def cb_goal(self, msg):
        self.goal_xy = f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = ControlUI()

    th = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    th.start()

    root = tk.Tk()
    root.title("Hospital Robot Control")

    # 표시 영역
    lbl_status = tk.Label(root, text="Status: IDLE", font=("Arial", 14))
    lbl_target = tk.Label(root, text="Target: -", font=("Arial", 14))
    lbl_goal   = tk.Label(root, text="Goal: -", font=("Arial", 12))
    lbl_speed  = tk.Label(root, text="Speed: 0.00 m/s", font=("Arial", 14))

    lbl_status.pack(pady=4)
    lbl_target.pack(pady=4)
    lbl_goal.pack(pady=2)
    lbl_speed.pack(pady=6)

    # 버튼 콜백
    def go(dept): node.pub_target.publish(String(data=dept))
    def sp_up():  node.pub_speed.publish(Float32(data=+0.05))
    def sp_dn():  node.pub_speed.publish(Float32(data=-0.05))
    def emg():    node.pub_emg.publish(Bool(data=True))
    def resume(): node.pub_emg.publish(Bool(data=False))

    # 목적지 버튼
    for dept in ["진단검사의학과", "영상의학과", "내과", "정형외과", "신경과"]:
        tk.Button(root, text=f"{dept} 출발", command=lambda d=dept: go(d)).pack(fill='x', padx=10, pady=2)

    tk.Label(root, text="").pack()

    tk.Button(root, text="▲ Speed Up (+0.05)", command=sp_up).pack(fill='x', padx=10, pady=2)
    tk.Button(root, text="▼ Speed Down (-0.05)", command=sp_dn).pack(fill='x', padx=10, pady=2)

    tk.Label(root, text="").pack()

    tk.Button(root, text="🚨 Emergency Stop", command=emg).pack(fill='x', padx=10, pady=2)
    tk.Button(root, text="▶ Resume", command=resume).pack(fill='x', padx=10, pady=2)

    # UI 업데이트 루프
    def refresh():
        lbl_status.config(text=f"Status: {node.status}")
        lbl_target.config(text=f"Target: {node.target}")
        lbl_goal.config(text=f"Goal: {node.goal_xy}")
        lbl_speed.config(text=f"Speed: {node.speed:.2f} m/s")
        root.after(200, refresh)

    refresh()
    root.mainloop()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
