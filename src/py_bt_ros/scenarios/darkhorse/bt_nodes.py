import math
import json
import random
from modules.base_bt_nodes import (
    BTNodeList, Status, SyncAction, Node,
    Sequence, Fallback, ReactiveSequence, ReactiveFallback, Parallel,
)
from modules.base_bt_nodes_ros import ActionWithROSAction, ConditionWithROSTopics
from limo_interfaces.action import Speak as speakActionMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry

INFO_DESK_NAME = "안내데스크"

DEPARTMENT_COORDINATES = {
    "진단검사의학과": {"x": 0.48070189356803894, "y": 0.2762919068336487, "w": 1.0},
    "영상의학과":    {"x": 6.578537940979004,  "y": 2.621462106704712,  "w": 1.0},
    "내과":          {"x": 7.445363998413086,  "y": 0.5102964639663696, "w": 1.0},
    "정형외과":      {"x": 0.753912627696991,  "y": -2.640972375869751, "w": 1.0},
    "안내데스크":    {"x": 2.836460590362549,  "y": 1.1752597093582153, "w": 1.0},
}
DEFAULT_DEPARTMENTS = ["진단검사의학과", "영상의학과", "내과", "정형외과"]

# ---------------- HELPER: UI 상태 전송 ----------------
def publish_ui_status(ros_node, text):
    pub = ros_node.create_publisher(String, '/hospital/nav_status', 10)
    msg = String()
    msg.data = text
    pub.publish(msg)

class GoToInfoDesk(ActionWithROSAction):
    def __init__(self, name, agent): super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))
    def _build_goal(self, agent, bb):
        coords = DEPARTMENT_COORDINATES.get(INFO_DESK_NAME)
        if not coords: return None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = float(coords['w'])
        publish_ui_status(self.ros.node, "안내데스크 복귀 중 🏠")
        return goal
    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            publish_ui_status(self.ros.node, "안내데스크 도착 완료")
            return Status.SUCCESS
        return Status.FAILURE

class WaitForQR(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self.agent = agent
        self.received_msg = None
        self.done = False
        self.sub = agent.ros_bridge.node.create_subscription(String, "/hospital/qr_login", self._callback, 10)
        self.home_saved = False
        self.first_run = True
    def _callback(self, msg): self.received_msg = msg
    def _tick(self, agent, bb):
        if self.first_run:
            publish_ui_status(agent.ros_bridge.node, "환자 접수 대기 중... 📋")
            self.first_run = False
        if self.done: return Status.SUCCESS
        if not self.home_saved:
            if hasattr(agent, 'robot_pose') and agent.robot_pose is not None:
                bb['home_pose'] = agent.robot_pose
                self.home_saved = True
        if self.received_msg is None: return Status.RUNNING
        try:
            data = json.loads(self.received_msg.data)
            bb['patient_id'] = data.get("patient_id", "Unknown")
            raw_depts = data.get("departments", None)
            if not raw_depts: raw_depts = DEFAULT_DEPARTMENTS
            depts = [d for d in raw_depts if (d in DEPARTMENT_COORDINATES) and (d != INFO_DESK_NAME)]
            bb['department_queue'] = list(depts)
            bb['remaining_depts']  = list(depts)
            bb['speak_text'] = "접수가 완료되었습니다."
            self.received_msg = None
            self.done = True
            publish_ui_status(agent.ros_bridge.node, f"환자 {bb['patient_id']} 접수 완료 ✅")
            return Status.SUCCESS
        except Exception as e:
            self.received_msg = None
            return Status.RUNNING

class IsEmergencyPressed(ConditionWithROSTopics):
    def __init__(self, name, agent, **kwargs):
        super().__init__(name, agent, [(Bool, "/emergency_trigger", "emergency_flag")], **kwargs)
    async def run(self, agent, bb):
        if "emergency_flag" not in self._cache: return Status.FAILURE
        return Status.SUCCESS if self._cache["emergency_flag"].data else Status.FAILURE

class IsBatteryLow(ConditionWithROSTopics):
    def __init__(self, name, agent): super().__init__(name, agent, [(Bool, "/battery_low", "battery_flag")])
    def _predicate(self, agent, bb):
        if "battery_flag" in self._cache and self._cache["battery_flag"].data: return True
        return False

class Think(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self.wait_min = 0; self.wait_max = 20
    def _tick(self, agent, bb):
        remaining = bb.get('remaining_depts', []) or []
        if INFO_DESK_NAME in remaining: remaining = [d for d in remaining if d != INFO_DESK_NAME]
        if len(remaining) == 0: return Status.FAILURE
        waiting_counts = {d: random.randint(self.wait_min, self.wait_max) for d in remaining}
        min_wait = min(waiting_counts.values())
        candidates = [d for d, w in waiting_counts.items() if w == min_wait]
        next_dept = random.choice(candidates)
        coords = DEPARTMENT_COORDINATES.get(next_dept)
        if not coords:
            remaining.remove(next_dept)
            bb['remaining_depts'] = remaining
            return Status.RUNNING
        bb['current_target_name'] = next_dept
        bb['current_target_coords'] = coords
        remaining.remove(next_dept)
        bb['remaining_depts'] = remaining
        bb['speak_text'] = f"{next_dept}로 이동할게요."
        return Status.SUCCESS

class Move(ActionWithROSAction):
    def __init__(self, name, agent): super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))
    def _build_goal(self, agent, bb):
        coords = bb.get('current_target_coords')
        target_name = bb.get('current_target_name', '목적지')
        if not coords: return None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = 1.0
        publish_ui_status(self.ros.node, f"{target_name} 이동 중 🚑")
        return goal
    def _interpret_result(self, result, agent, bb, status_code=None):
        target_name = bb.get('current_target_name', '목적지')
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['speak_text'] = f"{target_name}에 도착했습니다."
            return Status.SUCCESS
        bb['speak_text'] = f"{target_name} 이동 실패."
        return Status.FAILURE

class WaitDoctorDone(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self._done = False
        self.sub = agent.ros_bridge.node.create_subscription(Bool, "/hospital/doctor_input", self._cb, 10)
        self.status_sent = False
    def _cb(self, msg: Bool): 
        if msg.data is True: self._done = True
    def _tick(self, agent, bb):
        if not self.status_sent:
            target_name = bb.get('current_target_name', '진료과')
            publish_ui_status(agent.ros_bridge.node, f"{target_name} 진료 중... 👨‍⚕️")
            self.status_sent = True
        if not self._done: return Status.RUNNING
        self._done = False; self.status_sent = False
        bb['speak_text'] = "진료 종료. 다음으로 이동."
        return Status.SUCCESS

class ReturnHome(ActionWithROSAction):
    def __init__(self, name, agent): super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))
    def _build_goal(self, agent, bb):
        coords = DEPARTMENT_COORDINATES.get(INFO_DESK_NAME)
        if not coords: return None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = float(coords.get('w', 1.0))
        return goal

class KeepRunningUntilFailure(Node):
    def __init__(self, name, children=None): super().__init__(name)
    self.children = children if children is not None else []
    async def run(self, agent, bb):
        if not self.children: return Status.FAILURE
        status = await self.children[0].run(agent, bb)
        if status == Status.FAILURE: return Status.FAILURE
        return Status.RUNNING

class SpeakAction(ActionWithROSAction):
    def __init__(self, name, agent): super().__init__(name, agent, (speakActionMsg, 'speak_text'))
    def _build_goal(self, agent, bb):
        text = bb.pop('speak_text', None)
        if not text: return None
        goal = speakActionMsg.Goal()
        goal.text = text
        return goal

class WaitSpeedOK(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self.limit = 0.8
        self._odom = None
        self._warned = False
        self.sub = agent.ros_bridge.node.create_subscription(Odometry, "/odom", self._cb, 10)
    def _cb(self, msg: Odometry): self._odom = msg
    def _tick(self, agent, bb):
        if self._odom is None: return Status.SUCCESS
        if abs(self._odom.twist.twist.linear.x) > self.limit:
            if not self._warned:
                bb['speak_text'] = f"속도 위반! {self.limit} 이하로 줄이세요."
                self._warned = True
            return Status.SUCCESS
        self._warned = False
        return Status.SUCCESS

class SetAbort(SyncAction):
    def __init__(self, name, agent): super().__init__(name, self._tick)
    def _tick(self, agent, bb):
        bb['abort'] = True
        bb['speak_text'] = "비상 상황 발생! 복귀합니다."
        return Status.SUCCESS

class CheckAbort(SyncAction):
    def __init__(self, name, agent): super().__init__(name, self._tick)
    def _tick(self, agent, bb):
        if bb.get('abort', False): return Status.FAILURE
        return Status.SUCCESS

class SendDiagnosisEmail(SyncAction):
    def __init__(self, name, agent, topic="/hospital/send_diagnosis_email", **kwargs):
        super().__init__(name, self._tick, **kwargs)
        self.ros = agent.ros_bridge
        self.pub = self.ros.node.create_publisher(String, topic, 10)
    def _tick(self, agent, bb):
        payload = {"patient_id": bb.get("patient_id", "Unknown"), "email": bb.get("patient_email") or bb.get("email"), "request": "send_diagnosis_email"}
        msg = String(); msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)
        return Status.SUCCESS

class ControlSiren(SyncAction):
    def __init__(self, name, agent, enable=True, **kwargs):
        super().__init__(name, self._tick, **kwargs)
        self.ros = agent.ros_bridge
        self.pub = self.ros.node.create_publisher(Bool, "/cmd_siren", 10)
        self.enable_siren = True
        if 'enable' in kwargs:
            val = str(kwargs['enable']).lower()
            self.enable_siren = (val == 'true')
    def _tick(self, agent, bb):
        msg = Bool(); msg.data = self.enable_siren
        self.pub.publish(msg)
        if self.enable_siren: publish_ui_status(self.ros.node, "🚨 비상 복귀 중!")
        return Status.SUCCESS

class ForceSuccess(Node):
    def __init__(self, name, children=None): super().__init__(name, children)
    async def run(self, agent, bb):
        if not self.children: return Status.SUCCESS
        status = await self.children[0].run(agent, bb)
        if status == Status.RUNNING: return Status.RUNNING
        return Status.SUCCESS

# LIST REGISTRATION
CUSTOM_ACTION_NODES = ['WaitForQR', 'SpeakAction', 'Think', 'WaitSpeedOK', 'Move', 'WaitDoctorDone', 'ReturnHome', 'GoToInfoDesk', 'SendDiagnosisEmail', 'SetAbort', 'CheckAbort', 'ControlSiren']
CUSTOM_CONDITION_NODES = ['IsEmergencyPressed', 'IsBatteryLow']
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)
BTNodeList.CONTROL_NODES.append('KeepRunningUntilFailure')
BTNodeList.CONTROL_NODES.append('ForceSuccess')
print(f"Registered Actions: {BTNodeList.ACTION_NODES}")