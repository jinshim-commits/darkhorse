import math
import json
from modules.base_bt_nodes import (
    BTNodeList, Status, SyncAction, Node, 
    Sequence, Fallback, ReactiveSequence, ReactiveFallback, Parallel
)
from modules.base_bt_nodes_ros import ActionWithROSAction, ConditionWithROSTopics

# ROS 2 Messages
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# ---------------------------------------------------------
# 병원 진료과 좌표 매핑 (실제 맵 좌표 확인 필수)
# ---------------------------------------------------------
DEPARTMENT_COORDINATES = {
    "진단검사의학과": {"x": 0.48, "y": 0.27, "w": 1.0},
    "영상의학과":    {"x": 6.57, "y": 2.62, "w": 1.0},
    "내과":          {"x": 7.44, "y": 0.51, "w": 1.0},
    "정형외과":      {"x": 0.75, "y": -2.64, "w": 1.0},
    "신경과":        {"x": 2.83, "y": 1.17, "w": 1.0}
}

# ---------------------------------------------------------
# 1. WaitForQR: QR 데이터 수신 대기 (문지기)
# ---------------------------------------------------------
class WaitForQR(SyncAction):
    def __init__(self, name, agent):
        # [수정] agent 대신 self._tick 전달 (에러 해결)
        super().__init__(name, self._tick)
        self.agent = agent
        self.received_msg = None
        
        # [수정] 올바른 Node 접근 경로
        self.sub = agent.ros_bridge.node.create_subscription(
            String, 
            "/hospital/patient_data", 
            self._callback, 
            10
        )
        self.home_saved = False

    def _callback(self, msg):
        self.received_msg = msg
        print(f"[WaitForQR] 데이터 수신: {msg.data}")

    def _tick(self, agent, blackboard):
        # 1. 초기 위치 저장 (한 번만)
        if not self.home_saved:
            if hasattr(agent, 'robot_pose') and agent.robot_pose is not None:
                blackboard['home_pose'] = agent.robot_pose
                self.home_saved = True
                print("[WaitForQR] 초기 위치 저장 완료")

        # 2. 메시지가 없으면 대기 (RUNNING 반환 -> 로봇 멈춤)
        if self.received_msg is None:
            return Status.RUNNING

        # 3. 데이터 처리
        try:
            data = json.loads(self.received_msg.data)
            dept_list = data.get("departments", [])
            
            blackboard['department_queue'] = dept_list
            blackboard['patient_id'] = data.get("patient_id", "Unknown")
            
            print(f"[WaitForQR] 환자({blackboard['patient_id']}) 접수 완료. 경로: {dept_list}")
            
            self.received_msg = None 
            return Status.SUCCESS # 다음 단계로 이동 허가
            
        except json.JSONDecodeError:
            print("[WaitForQR] JSON 에러")
            self.received_msg = None
            return Status.RUNNING

# ---------------------------------------------------------
# 2. Think: 다음 목적지 결정
# ---------------------------------------------------------
class Think(SyncAction):
    def __init__(self, name, agent):
        # [수정] agent 대신 self._tick 전달
        super().__init__(name, self._tick)

    def _tick(self, agent, blackboard):
        queue = blackboard.get('department_queue', [])
        
        if len(queue) > 0:
            next_dept = queue.pop(0)
            coords = DEPARTMENT_COORDINATES.get(next_dept)
            
            if coords:
                blackboard['current_target_name'] = next_dept
                blackboard['current_target_coords'] = coords
                blackboard['department_queue'] = queue
                print(f"[Think] 다음 목적지 설정: {next_dept}")
                return Status.SUCCESS
            else:
                print(f"[Think] 좌표 정보 없음: {next_dept}")
                return Status.FAILURE
        else:
            print("[Think] 모든 진료과 방문 완료.")
            return Status.FAILURE

# ---------------------------------------------------------
# 3. Move: 이동 (기존 유지)
# ---------------------------------------------------------
class Move(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, 'navigate_to_pose'))

    def _build_goal(self, agent, blackboard):
        coords = blackboard.get('current_target_coords')
        if not coords: return None

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = float(coords['w']) if 'w' in coords else 1.0
        
        print(f"[Move] {blackboard.get('current_target_name')}로 이동 시작...")
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            print("[Move] 도착 완료.")
            return Status.SUCCESS
        return Status.FAILURE

# ---------------------------------------------------------
# 4. Doctor: 진료 대기 (기존 유지 - Node로 변경 추천)
# ---------------------------------------------------------
# Doctor는 ConditionWithROSTopics를 쓰면 '대기'가 안 되므로 
# WaitForQR처럼 SyncAction으로 바꾸거나 로직 수정이 필요할 수 있습니다.
# 일단 기존 구조(ConditionWithROSTopics)가 맞다고 가정하고 유지합니다.
class Doctor(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (Bool, "/hospital/doctor_input", "doctor_signal")
        ])

    def _predicate(self, agent, blackboard):
        if "doctor_signal" in self._cache:
            msg = self._cache["doctor_signal"]
            if msg.data is True:
                print("[Doctor] 진료 완료 확인.")
                del self._cache["doctor_signal"]
                return True
        return False

# ---------------------------------------------------------
# 5. ReturnHome: 복귀 (기존 유지)
# ---------------------------------------------------------
class ReturnHome(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, 'navigate_to_pose'))

    def _build_goal(self, agent, blackboard):
        home_pose = blackboard.get('home_pose')
        if not home_pose:
            print("[Return] 홈 위치 없음, (0,0)으로 설정")
            home_pose = PoseStamped()
            home_pose.pose.position.x = 0.0
            home_pose.pose.position.y = 0.0
            home_pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose = home_pose if hasattr(home_pose, 'position') else home_pose.pose

        print("[Return] 초기 위치로 복귀합니다.")
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        return Status.FAILURE

# ---------------------------------------------------------
# 6. 루프 노드 (Async 실행 지원)
# ---------------------------------------------------------
class KeepRunningUntilFailure(Node):
    def __init__(self, name, children=None):
        super().__init__(name)
        self.children = children if children is not None else []

    async def run(self, agent, blackboard):
        if not self.children:
            return Status.FAILURE
        status = await self.children[0].run(agent, blackboard)
        if status == Status.FAILURE:
            return Status.FAILURE
        return Status.RUNNING

# ---------------------------------------------------------
# 노드 등록
# ---------------------------------------------------------
CUSTOM_ACTION_NODES = ['WaitForQR', 'Think', 'Move', 'Doctor', 'ReturnHome']
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONTROL_NODES.append('KeepRunningUntilFailure')