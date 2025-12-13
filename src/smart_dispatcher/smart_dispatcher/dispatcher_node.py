import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool, String
import json
import random
import time
import os
import sys

class DeptDispatcher(Node):
    def __init__(self):
        super().__init__('dept_dispatcher')

        # 병원 좌표 데이터
        self.master_coordinates = {
            "진단검사의학과": {"x": 0.48, "y": 0.27, "w": 1.0},
            "영상의학과":    {"x": 6.57, "y": 2.62, "w": 1.0},
            "내과":          {"x": 7.44, "y": 0.51, "w": 1.0},
            "정형외과":      {"x": 0.75, "y": -2.64, "w": 1.0},
            "신경과":        {"x": 2.83, "y": 1.17, "w": 1.0},
        }

        # hospital_config.json 읽기
        self.active_departments = self.load_config()

        # 환자 UI에서 start 신호 구독
        self.sub_start = self.create_subscription(
            String, '/hospital_data',
            self.start_navigation, 10
        )

        # 도착 신호 발행
        self.arrived_pub = self.create_publisher(Bool, '/arrived', 10)

        # Nav2 navigator 활성화
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        print("🚀 Dispatcher 실행됨")

    def load_config(self):
        path = os.path.expanduser("~/hospital_config.json")
        if not os.path.exists(path):
            print("[오류] hospital_config.json 없음!")
            sys.exit(1)

        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        selected = data.get("active_departments", [])
        return [x for x in selected if x in self.master_coordinates]

    def start_navigation(self, msg):
        data = json.loads(msg.data)
        patient_name = data.get("patient_name", "Unknown")
        print(f"\n📌 환자 '{patient_name}' 도착 → 최적 진료실 선택 중…")

        # 대기열 시뮬레이션
        waiting = {dept: random.randint(0, 10) for dept in self.active_departments}
        target = min(waiting, key=waiting.get)
        coord = self.master_coordinates[target]

        print(f"👉 선택된 진료실: {target} (대기 {waiting[target]}명)")
        print("🚗 이동 시작…")

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = coord['x']
        goal.pose.position.y = coord['y']
        goal.pose.orientation.w = coord['w']

        self.navigator.goToPose(goal)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            print(f"🏁 [{target}] 도착 완료!")
            self.arrived_pub.publish(Bool(data=True))
        else:
            print("❌ 이동 실패")

def main():
    rclpy.init()
    node = DeptDispatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
