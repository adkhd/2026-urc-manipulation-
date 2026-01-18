from servo_can_bridge import x_theta as xth
from servo_can_bridge import pdosdo as drv
from servo_can_bridge import can_iface
import canopen
#!/usr/bin/env python3
import math, threading, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, String
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
RADPS_TO_RPM = 60.0 / (2.0 * math.pi)


class SERVO_CAN_BRIDGE(Node):
    """
    5축(or N축) 멀티 브릿지:
      - Servo 토픽(Float64MultiArray) → 각 축별 PDO 명령 (게이팅)
      - 각 드라이브 TPDO → /joint_states (배열)
      - /servo_ok (Bool), /bridge_state (String)

    게이트 조건(전체 AND):
      - 각 드라이브 NMT OK (PRE-OP/OP)
      - 초기화+매핑 완료
      - 안정화 대기 경과                                                                
      - TPDO 타임아웃 미발생
      - (임시) Homing OK (파라미터로 강제 True)
    """

    def __init__(self):
        super().__init__('servo_can_bridge_exe')
        # ---------- 공통 파라미터 ----------
        self.declare_parameter('mode', 'velocity')                     # 'velocity' | 'position'
        self.declare_parameter('input_topic', '/zero_arm_controller/joint_trajectory')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('setup_on_start', True)
        self.declare_parameter('connect_on_start', True)
        self.declare_parameter('channel', 'can0')
        self.declare_parameter('bitrate', 500000)

        self.declare_parameter('tpdo_timeout_sec', 0.5)
        self.declare_parameter('ready_wait_sec', 2.0)
        self.declare_parameter('homing_ok_forced', True)

        # ---------- 다축 파라미터(길이가 축 수와 동일해야 함) ----------
        # 드라이브 노드 ID들 (예: [1,2,3,4,5])
        self.declare_parameter('node_ids', [1,2,3,4,5])
        # 각 관절 이름 (joint_states.name에 사용)
        self.declare_parameter('joint_names', ['joint1','joint2','joint3','joint4','joint5'])
        # 모터방향 보정(+1/-1)
        self.declare_parameter('signs', [1,1,1,1,1])
        # 기어비(모터축 RPM 변환에 사용)
        self.declare_parameter('gear_ratios', [285.0, 33., 33.,110.25,110.25])

        # 호밍
        self.declare_parameter('homing_on', True) 
        self.declare_parameter('homing_dir', [1, 1, 1, 1, 1]) #1:+ -1:- 방향
        self.declare_parameter('homing_timeout', [15, 15, 15, 15, 15]) #1:+ -1:- 방향
        self.declare_parameter('homing_state', [0, 0, 0, 0, 0]) #0:호밍 미완 1: 호밍 완료
        self.declare_parameter('homing_joint', [1, 1, 1, 1, 1]) 

        # 리밋/스케일
        self.declare_parameter('vel_limit_rpm', [5000.0, 1200.0, 2000.0 ,3000.0, 3000.0])
        self.declare_parameter('acc_limit_rpmps', [300.0]*5)
        self.declare_parameter('dcc_limit_rpmps', [10000.0]*5)
        self.declare_parameter('vel_trip_ratio', [1.2]*5)              # 측정속도 트립 배율
        self.declare_parameter('pos_abs_limit_counts', [0]*5)          # 0=미사용
        self.declare_parameter('joint_vel_limit', [0.5, 0.07, 0.05, 1.0, 1.0]) 

        # 단위 변환 (카운트→라디안, RPM→라디안/초)
        self.declare_parameter('pos_rad_per_count', [1.0e-5]*5)
        self.declare_parameter('vel_radps_per_rpm', [2.0*math.pi/60.0]*5)
        self.declare_parameter('deadband_radps', [0.0]*5)

        # ---------- 파라미터 로드 ----------
        self.mode               = self.get_parameter('mode').value.lower()
        self.input_topic        = self.get_parameter('input_topic').value
        self.publish_rate_hz    = float(self.get_parameter('publish_rate_hz').value)
        self.setup_on_start     = bool(self.get_parameter('setup_on_start').value)
        self.connect_on_start   = bool(self.get_parameter('connect_on_start').value)
        self.channel = self.get_parameter('channel').value
        self.bitrate = self.get_parameter('bitrate').value

        self.tpdo_timeout_sec   = float(self.get_parameter('tpdo_timeout_sec').value)
        self.ready_wait_sec     = float(self.get_parameter('ready_wait_sec').value)
        self.homing_ok_forced   = bool(self.get_parameter('homing_ok_forced').value)

        self.node_ids           = list(self.get_parameter('node_ids').value)
        self.joint_names        = list(self.get_parameter('joint_names').value)
        self.signs              = list(self.get_parameter('signs').value)
        self.gear_ratios        = list(self.get_parameter('gear_ratios').value)

        self.homing_on          = bool(self.get_parameter('homing_on').value)
        self.homing_dir         = list(self.get_parameter('homing_dir').value)
        self.homing_timeout     = list(self.get_parameter('homing_timeout').value)
        self.homing_state       = list(self.get_parameter('homing_state').value)
        self.homing_joint       = list(self.get_parameter('homing_joint').value)

        self.vel_limit_rpm      = list(self.get_parameter('vel_limit_rpm').value)
        self.acc_limit_rpmps    = list(self.get_parameter('acc_limit_rpmps').value)
        self.dcc_limit_rpmps    = list(self.get_parameter('dcc_limit_rpmps').value)
        self.vel_trip_ratio     = list(self.get_parameter('vel_trip_ratio').value)
        self.pos_abs_limit_cnts = list(self.get_parameter('pos_abs_limit_counts').value)
        self.joint_vel_limit    = list(self.get_parameter('joint_vel_limit').value)

        self.pos_rad_per_count  = list(self.get_parameter('pos_rad_per_count').value)
        self.vel_radps_per_rpm  = list(self.get_parameter('vel_radps_per_rpm').value)
        self.deadband_radps     = list(self.get_parameter('deadband_radps').value)


        self.A11 = -1.0; self.A12 = -1.0
        self.A21 = 1.0; self.A22 = -1.0

# j4/j5 캐시 (한 콜백 사이클에서 q4,q5 둘 다 모았을 때만 명령)
        self._wrist_cache = {"q4": None, "q5": None}

        self.num_axes = len(self.node_ids)
        assert all(len(lst) == self.num_axes for lst in [
            self.joint_names, self.signs, self.gear_ratios,
            self.vel_limit_rpm, self.acc_limit_rpmps, self.dcc_limit_rpmps, self.vel_trip_ratio,
            self.pos_abs_limit_cnts, self.pos_rad_per_count, self.vel_radps_per_rpm, self.deadband_radps
        ]), "모든 리스트 파라미터의 길이는 node_ids 길이와 같아야 합니다."

        # ---------- CANopen 연결/초기화 ----------

        try:
            self.get_logger().info("Bringing up slcan interface (can0)...")
            # (이 bringup_slcan 함수는 이 파일 어딘가에 정의되어 있어야 함)
            can_iface.bringup_slcan("/dev/canable0", "can0", 1000000, "s6")
            self.get_logger().info("slcan interface is up.")
        except Exception as e:
            self.get_logger().error(f"Failed to bring up slcan: {e}")
            return # 노드 초기화 실패로 중단
        self.get_logger().info(f"Connecting to {self.channel}...")
        self.net = canopen.Network()
        self.net.connect(bustype="socketcan", channel=self.channel, bitrate=self.bitrate)
        self.nodes = []
        self._mapping_done = [False]*self.num_axes
        self._init_done    = [False]*self.num_axes
        self._last_init_time = [None]*self.num_axes

        if self.connect_on_start:
            for i, nid in enumerate(self.node_ids):
                try:
                    node = drv.add_canopen_node(self.net, nid, heartbeat_ms=500, to_operational=False)
                    self.nodes.append(node)
                    self.get_logger().info(f"[axis{i}] can connection ok")
                except Exception as e:
                    self.get_logger().error(f"[axis{i}] connect_canopen fail (node_id={nid}): {e}")
                    self.nodes.append(None)

        # 초기 설정+매핑
        if self.setup_on_start:
            for i, node in enumerate(self.nodes):
                if node is None: 
                    continue
                try:
                    drv.set_pre_control(node)
                    if self.mode == 'position':
                        node.sdo["ServoType"].raw = 2
                    drv.set_PID(node)
                    drv.mapping(node, 1000./self.publish_rate_hz)  # TPDO1: pos/vel 콜백을 node 내부에 등록
                    self._mapping_done[i] = True
                    self._init_done[i] = True
                    self._last_init_time[i] = time.time()
                    self.get_logger().info(f"[axis{i}] init+mapping OK")
                except Exception as e:
                    self.get_logger().warn(f"[axis{i}] init/mapping skipped/failed: {e}")

        #여기서 노드 5개를 이용하여 관절별 호밍 수행 + 가능하면 각도 offset
        #어떤 방향으로 호밍할지도 설정..
        #blocking
        if self.homing_on == True:
            self.homing(self.homing_dir, self.homing_timeout, self.homing_joint)

        # ---------- TPDO 수신/상태 ----------
        self._lock = threading.Lock()
        # 마지막 TPDO에서 받은 값/시각
        self._last_pos_counts = [0]*self.num_axes
        self._last_vel_rpm    = [0.0]*self.num_axes
        self._last_tpdo_time  = [0.0]*self.num_axes

        # node.tpdo 콜백 연결 (mapping에서 add_callback 한 번 더 호출해도 누적되니, 여기서 보강 연결)
        for i, node in enumerate(self.nodes):
            if node is None:    
                continue
            try:
                node.tpdo[1].add_callback(self._make_tpdo_cb(i))
                self.get_logger().info(f"[axis{i}] TPDO1 callback registered.")
            except Exception as e:
                self.get_logger().warn(f"[axis{i}] TPDO1 callback registration failed: {e}")
        cmd_qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        lifespan=Duration(seconds=0.1),   # 오래된 명령 폐기
        )
        js_qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        )
        
        try:
            for node in self.nodes:
                drv.set_nmt_preop(node)
                node.sdo["ResetEncoder"].raw = 0
                drv.set_nmt_operational(node)
        except Exception:
            pass
        # ---------- ROS I/F ----------
        self.sub_cmd   = self.create_subscription(Float64MultiArray, self.input_topic, self._on_servo_cmd, cmd_qos)
        self.pub_js    = self.create_publisher(JointState, '/joint_states', js_qos)
        self.pub_ok    = self.create_publisher(Bool, '/servo_ok', 5)
        self.pub_state = self.create_publisher(String, '/bridge_state', 5)

        self.timer_js    = self.create_timer(1.0/max(1.0, self.publish_rate_hz), self._publish_joint_state)
        self.timer_check = self.create_timer(0.05, self._health_check_loop)

        self._servo_ok = False
        
        self.get_logger().info(f"[MultiBridge] axes={self.num_axes}, mode={self.mode}, topic={self.input_topic}")

    # ---------- TPDO 콜백 생성 ----------
    def _make_tpdo_cb(self, axis_idx):
        def _cb(pdo):
            try:
                snap = {v.name: v.raw for v in pdo}
                pos = int(snap.get("Read_MotorPos", 0))
                vel_raw = snap.get("Read_MotorVel", 0)
                vel = float(vel_raw) if isinstance(vel_raw, (int,float)) else float(vel_raw)

                with self._lock:
                    self._last_pos_counts[axis_idx] = pos
                    self._last_vel_rpm[axis_idx]    = vel
                    self._last_tpdo_time[axis_idx]  = time.time()
            except Exception as e:
                self.get_logger().warn(f"[axis{axis_idx}] TPDO parse failed: {e}")
        return _cb

    # ---------- Servo 명령(게이트) ----------
    def _on_servo_cmd(self, msg: Float64MultiArray):
        if not self._servo_ok:
            return

        m = min(len(msg.data), self.num_axes)
        if self.mode == 'velocity':
            q_scaling = self.scaling(msg)
        else:
            q_scaling = msg
        for i in range(m):
            # j4/j5만 듀얼 노드로 넘겨서 한번에 명령될 수 있게 함
            if i in (3, 4):
                node = [self.nodes[3], self.nodes[4]]
            else:
                node = self.nodes[i]

            if node is None or (isinstance(node, list) and any(n is None for n in node)):
                continue
            
            q = float(q_scaling.data[i])

            try:
                if self.mode == 'velocity':
                    self._handle_velocity_axis(i, node, q)
                elif self.mode == 'position':
                    self._handle_position_axis(i, node, q)
                else:
                    if i == 0:
                        self.get_logger().warn_once(f"Unknown mode '{self.mode}'")
            except Exception as e:
                self.get_logger().error(f"error in axis{i}")
                self._trip(f"[axis{i}] command error: {e}")
                return

    def  _handle_velocity_axis(self, i, node, qdot_radps: float):
        """
        self에는 wrist_cache만 사용
        나머지는 모두 지역 변수로 처리
        i = 0~4 (joint index)
        node = 단일 노드 or [node4, node5]
        """

        # == 데드밴드 ==
        if abs(qdot_radps) < self.deadband_radps[i]:
            qdot_radps = 0.0

        # ----------------------------
        # 1) 축별 raw rpm 계산 (지역 변수 배열에 적재)
        # ----------------------------
        # 지역 변수 버퍼 (매 호출마다 새로 생성)
        #   ※ servo cmd가 5축 모두 빠르게 연속 호출된다는 전제
        #   ※ J4/J5(3,4)는 i==4에서 둘 다 rpm 계산을 수행
        if not hasattr(self, "_local_buf"):
            # 첫 호출 시 초기화
            self._local_buf = {
                "rpm_raw": [0.0] * 5,     # 0~4축 raw rpm 저장
                "nodes":   [None] * 5,    # 0~4축 해당 노드 저장
                "filled":  [False] * 5,   # 해당 축이 들어왔는지 체크
            }

        buf = self._local_buf  # 지역 버퍼 단축명

        # --- 단일 축 J1 ---
        if i == 0:
            rpm_raw = qdot_radps * RADPS_TO_RPM * 285 * 3
            buf["rpm_raw"][0] = rpm_raw
            buf["nodes"][0]   = node
            buf["filled"][0]  = True
            return  # 전체는 J5(i==4) 때 처리

        # --- J2 ---
        if i == 1:
            rpm_raw = xth.thd2xd_L1(
                qdot_radps,
                self._last_pos_counts[i] * 6.35 / (2.0 * 33.0 * 4.0)
            ) * 33.0 * 60.0 / 6.35
            buf["rpm_raw"][1] = rpm_raw
            buf["nodes"][1]   = node
            buf["filled"][1]  = True
            return

        # --- J3 ---
        if i == 2:
            rpm_raw = xth.thd2xd_L2(
                qdot_radps,
                self._last_pos_counts[i] * 3.175 / (2.0 * 33.0)
            ) * 33.0 * 60.0 / 12.7
            buf["rpm_raw"][2] = rpm_raw
            buf["nodes"][2]   = node
            buf["filled"][2]  = True
            return

        # --- J4: q4만 저장하여 J5에서 계산 ---
        if i == 3:
            self._wrist_cache["q4"] = qdot_radps
            # node4 저장
            if isinstance(node, (list, tuple)) and len(node) > 0:
                buf["nodes"][3] = node[0]
            else:
                buf["nodes"][3] = None
            buf["filled"][3] = True
            return

        # --- J5: 여기에서 J4+J5 rpm 계산 + 모든 축 스케일링/명령 ---
        if i == 4:
            self._wrist_cache["q5"] = qdot_radps
            q4 = self._wrist_cache.get("q4", 0.0)
            q5 = self._wrist_cache.get("q5", 0.0)

            # node5 저장
            buf["nodes"][4] = (node[1] if isinstance(node, (list, tuple)) and len(node) > 1 else None)
            buf["filled"][4] = True

            # 커플링 변환 → J4,J5 raw rpm 계산
            rpm4_raw = (self.A11*q4 + self.A12*q5) * RADPS_TO_RPM * 49 * 2.25
            rpm5_raw = (self.A21*q4 + self.A22*q5) * RADPS_TO_RPM * 49 * 2.25

            buf["rpm_raw"][3] = rpm4_raw
            buf["rpm_raw"][4] = rpm5_raw

            # q4,q5 캐시 초기화
            self._wrist_cache["q4"] = None
            self._wrist_cache["q5"] = None

            # ---------------------------------------------
            # 2) 모든 축 rpm_raw 준비 완료 → 글로벌 스케일링
            # ---------------------------------------------
            if not all(buf["filled"]):
                # 5축 모두 들어오지 않았으면 skip (다음 사이클에서 채울 것)
                return

            # --- 2-1) 가장 많이 limit를 넘는 축 찾기 ---
            max_ratio = 0.0
            for j in range(5):
                limit = float(self.vel_limit_rpm[j])
                rpm_r = buf["rpm_raw"][j]
                if limit <= 0: 
                    continue
                if rpm_r != 0:
                    ratio = abs(rpm_r) / limit
                    if ratio > max_ratio:
                        max_ratio = ratio

            # --- 2-2) 스케일 팩터 결정 ---
            scale = 1.0 if max_ratio <= 1.0 else (1.0 / max_ratio)

            # ---------------------------------------------
            # 3) 스케일 적용 & 실제 speed_control 발행
            # ---------------------------------------------
            for j in range(5):
                nd = buf["nodes"][j]
                if nd is None:
                    continue

                rpm_scaled = int(buf["rpm_raw"][j] * scale)

                # soft clamp
                limit = self.vel_limit_rpm[j]
                rpm_scaled = max(-limit, min(limit, rpm_scaled))

                # 로그
                self.get_logger().info(
                    f"[J{j+1}] raw={int(buf['rpm_raw'][j])}, scaled={rpm_scaled}, scale={scale:.3f}"
                )

                # 계수
                acc = max(abs(int(rpm_scaled // 0.4)), 200)
                dcc = max(abs(int(rpm_scaled // 0.1)), 300)

                drv.speed_control(nd, vel=rpm_scaled, acc=acc, dcc=dcc)

            # ---------------------------------------------
            # 4) 지역 버퍼 리셋 (다음 사이클 준비)
            # ---------------------------------------------
            buf["rpm_raw"] = [0.0] * 5
            buf["nodes"]   = [None] * 5
            buf["filled"]  = [False] * 5

            return

    def _handle_position_axis(self, i, node, q_rad: float):
        """
        위치 제어 모드
        - Forward Kinematics의 역연산 적용
        - 현재 위치와 목표 위치의 차이에 비례한 가변 속도/가속도 적용
        """

        # J4, J5 처리를 위한 캐시 및 버퍼 초기화
        if not hasattr(self, "_pos_local_buf"):
            self._pos_local_buf = {
                "nodes":   [None] * 5,
                "filled":  [False] * 5
            }
        
        buf = self._pos_local_buf

        pulse = [19., 2., 2., 13., 13.]
        vel_t = [0.015, 0.04, 0.12, 0.002, 0.002]
        acc_t = [0.5, 0.5, 0.5, 0.2, 0.2]
        dcc_t = [0.3, 0.3, 0.3, 0.2, 0.2]
        # -----------------------------------------------------------
        # [내부 함수] 거리 비례 속도/가속도 계산
        # -----------------------------------------------------------
        def calc_dynamic_params(axis_idx, target_cnt):
            current_cnt = self._last_pos_counts[axis_idx]
            dist = abs(target_cnt - current_cnt)

            # 거리에 따른 속도 계수 설정 (파라미터 튜닝 가능)
            # 예: 펄스 차이가 10000일 때 맥스 속도, 그보다 작으면 비례해서 줄임
            # 최소 속도는 확보해야 움직임 (max의 5% 등)
            #pos_err(count) => count/4/encoder_pulse = rotation => rpm = rotation/60/t               => vel, acc(rpm, rpm/s)
            #velocity를 count로부터 정하고(그 거리를 몇초동안 갈 것인지)
            #그 속도를 몇초로 낼 것인지를 정하기 
            
            pulse_quad = pulse[axis_idx]*4.0

            vel = dist/pulse_quad*60./vel_t[axis_idx]
            acc = vel/acc_t[axis_idx]
            dcc = vel/dcc_t[axis_idx]
            # 거리 5000 count당 Max속도 도달하도록 설정 (예시)
            
            # 너무 느리면 안되므로 최소 10RPM은 보장
            target_vel = max(400.0, vel)
            if axis_idx in(1,2) :
                target_vel = min(target_vel, 4000) # 리밋 클램핑
            else:
                target_vel = min(target_vel,10000)

            # 가속도는 속도에 비례하거나, 고정값을 사용
            # 여기서는 속도가 빠르면 가속도도 높게 설정
            target_acc = min(max(500.0,  acc),10000)
            target_dcc = min(max(500.0,  dcc), 20000)
            return int(target_vel), int(target_acc), int(target_dcc)

        # -----------------------------------------------------------
        # [Case 1] J1 (Rotational)
        # -----------------------------------------------------------
        if i == 0:
            # Forward: pos = cnt / 43320 * 2pi
            # Inverse: cnt = pos * 43320 / 2pi
            # 상수: 285*3*4*19 = 43320
            gear_ratio_const = 64980.0
            target_counts = int(q_rad * gear_ratio_const / (2.0 * math.pi))
            
            dyn_vel, dyn_acc, dyn_dcc = calc_dynamic_params(i, target_counts)
            self.get_logger().info(f"{i}axis || pos: {target_counts} // vel: {dyn_vel} // acc: {dyn_acc} // dcc: {dyn_dcc}")
            drv.pos_control(node, pos=target_counts, vel=dyn_vel, acc=dyn_acc, dcc=dyn_dcc)
            return

        # -----------------------------------------------------------
        # [Case 2] J2 (L1 - Linear)
        # -----------------------------------------------------------
        if i == 1:
            # Forward: x = cnt * 6.35 / (2*33*4) -> theta = x2th(x)
            # Inverse: theta -> x = th2x(theta) -> cnt = x * (2*33*4) / 6.35
            
            # xth 라이브러리에 역함수가 있어야 함 (없으면 근사치 사용)
            try:
                target_x_mm = xth.th2x_L1(q_rad)
            except AttributeError:
                self.get_logger().error("xth.th2x_L1 function missing!")
                return

            mm_to_cnt = (2.0 * 33.0 * 4.0) / 6.35
            target_counts = int(target_x_mm * mm_to_cnt)

            dyn_vel, dyn_acc, dyn_dcc = calc_dynamic_params(i, target_counts)
            self.get_logger().info(f"{i}axis || pos: {target_counts} // vel: {dyn_vel} // acc: {dyn_acc} // dcc: {dyn_dcc}")
            drv.pos_control(node, pos=target_counts, vel=dyn_vel, acc=dyn_acc, dcc=dyn_dcc)
            return

        # -----------------------------------------------------------
        # [Case 3] J3 (L2 - Linear)
        # -----------------------------------------------------------
        if i == 2:
            # Forward: x = cnt * 3.175 / (2*33) -> theta = x2th(x)
            # Inverse: theta -> x = th2x(theta) -> cnt = x * (2*33) / 3.175
            
            try:
                target_x_mm = xth.th2x_L2(q_rad)
            except AttributeError:
                self.get_logger().error("xth.th2x_L2 function missing!")
                return

            mm_to_cnt = (2.0 * 33.0) / 3.175
            target_counts = int(target_x_mm * mm_to_cnt)

            dyn_vel, dyn_acc, dyn_dcc = calc_dynamic_params(i, target_counts)

            self.get_logger().info(f"{i}axis || pos: {target_counts} // vel: {dyn_vel} // acc: {dyn_acc} // dcc: {dyn_dcc}")
            drv.pos_control(node, pos=target_counts, vel=dyn_vel, acc=dyn_acc, dcc=dyn_dcc)
            return

        # -----------------------------------------------------------
        # [Case 4] J4 (Caching)
        # -----------------------------------------------------------
        if i == 3:
            self._wrist_cache["q4"] = q_rad
            # 리스트 분기 처리
            if isinstance(node, (list, tuple)) and len(node) > 0:
                buf["nodes"][3] = node[0] 
            else:
                buf["nodes"][3] = node
            buf["filled"][3] = True
            return

        # -----------------------------------------------------------
        # [Case 5] J5 (Coupling Calculation)
        # -----------------------------------------------------------
        if i == 4:
            self._wrist_cache["q5"] = q_rad
            q4 = self._wrist_cache.get("q4", 0.0)
            q5 = self._wrist_cache.get("q5", 0.0)

            if isinstance(node, (list, tuple)) and len(node) > 1:
                buf["nodes"][4] = node[1]
            else:
                buf["nodes"][4] = node # 예외상황

            buf["filled"][4] = True

            # --- 역기구학 수식 유도 ---
            # Forward 식:
            # q3 = -(c3 - c4) * K * 2pi
            # q4 = -(c3 + c4) * K * 2pi
            # 여기서 K_inv = 1 / (K * 2pi) 라고 하면
            # -(c3 - c4) = q3 * K_inv
            # -(c3 + c4) = q4 * K_inv
            # 두 식을 더하고 빼서 c3, c4를 구함
            
            # 상수항 (Forward 코드 기준): 2 * 49 * 2.25 * 4 * 13 = 11466
            # Forward 식을 보면 분모가 (2 * 49 * 2.25 * 4 * 13) 임.
            
            gear_total = 2.0 * 49.0 * 2.25 * 4.0 * 13.0 # 11466.0
            k_factor = gear_total / (2.0 * math.pi)

            # 유도된 역기구학 식:
            # c3 = -0.5 * k_factor * (q4 + q5)
            # c4 =  0.5 * k_factor * (q4 - q5)

            cnt3_target = int(-0.5 * k_factor * (q4 + q5))
            cnt4_target = int( 0.5 * k_factor * (q4 - q5))

            # --- 모터 명령 전송 (동적 속도 적용) ---
            node_j4 = buf["nodes"][3]
            node_j5 = buf["nodes"][4]

            # J4 모터 (Axis 3)
            if node_j4 is not None:
                v4, a4, d4 = calc_dynamic_params(3, cnt3_target)
                
                self.get_logger().info(f"3axis | pos: {cnt3_target} // vel: {v4} // acc: {a4} // dcc: {d4}")
            # J5 모터 (Axis 4)
            if node_j5 is not None:
                v5, a5, d5 = calc_dynamic_params(4, cnt4_target)
                self.get_logger().info(f"4axis || pos: {cnt4_target} // vel: {v5} // acc: {a5} // dcc: {d5}")
                drv.pos_control(node[0], pos=cnt3_target, vel=v4, acc=a4, dcc=d4)
                drv.pos_control(node[1], pos=cnt4_target, vel=v5, acc=a5, dcc=d5)
                

            # 초기화
            self._wrist_cache["q4"] = None
            self._wrist_cache["q5"] = None
            buf["filled"][3] = False
            buf["filled"][4] = False
            buf["nodes"][3] = None
            buf["nodes"][4] = None 

    # ---------- joint_states 발행 ----------
    def _publish_joint_state(self):
        if not self._servo_ok:
            return
        with self._lock:
            pos_counts = list(self._last_pos_counts)
        # 배열 변환
        pos = [0.0]*(self.num_axes+1)
        vel = [0.0]*(self.num_axes+1)
        for i in range(self.num_axes+1):
            if i == 0:
                pos[i] = pos_counts[i]/285./3./4./19.*2*math.pi
            elif i == 1:
                #self.get_logger().info(f"{pos_counts[i]*6.35/(2.*33.*4.)}")
                pos[i] = xth.x2th_L1(pos_counts[i]*6.35/(2.*33.*4.))
            elif i == 2:
                pos[i] = xth.x2th_L2(pos_counts[i]*3.175/(2.*33.))
            elif i == 3:
                pass
            elif i == 4:
                pos[3] = -(pos_counts[3]-pos_counts[4])/2./49./2.25/4./13.*2*math.pi
                pos[4] = (-(pos_counts[3]+pos_counts[4])/2./49./2.25/4./13.*2*math.pi)
            elif i == self.num_axes:
                pos[i] = 0.
        
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        js.name.append('finger_joint1')
        js.position = pos
        js.velocity = vel
        js.effort = []
        self.pub_js.publish(js)

    # ---------- 상태 평가 & 게이트 ----------
    def _health_check_loop(self):
        ok, reason = self._evaluate_ready_all()
        ok=1
        if ok and not self._servo_ok:
            self._set_servo_ok(True, "READY")
            return
        if (not ok) and self._servo_ok:
            self._trip(f"Condition lost: {reason or 'unknown'}")
            return

        # 런타임 트립(속도/위치 한계)
        if ok and self._servo_ok:
            if self._should_trip_runtime():
                self.get_logger().error(f"속도/위치 오류")
                return

    def _evaluate_ready_all(self):
        # 모든 축이 조건 만족해야 TRUE
        now = time.time()
        for i, node in enumerate(self.nodes):
            if node is None:
                return False, f"[axis{i}] node missing"

            # 1) NMT
            try:
                state = str(node.nmt.state)
                if state not in ("PRE-OPERATIONAL","OPERATIONAL"):
                    return False, f"[axis{i}] NMT={state}"
            except Exception as e:
                return False, f"[axis{i}] NMT check fail: {e}"

            # 2) init/mapping
            if not (self._init_done[i] and self._mapping_done[i]):
                return False, f"[axis{i}] init/mapping not done"

            # 3) 안정화 대기
            t0 = self._last_init_time[i]
            if t0 is None or (now - t0) < self.ready_wait_sec:
                return False, f"[axis{i}] stabilizing"

            # 4) TPDO 타임아웃
            if (now - self._last_tpdo_time[i]) > self.tpdo_timeout_sec:
                return False, f"[axis{i}] tpdo timeout"

            # 5) homing (강제 True)
            if not self.homing_ok_forced:
                return False, f"[axis{i}] homing not done"

        return True, None

    def _should_trip_runtime(self):
        with self._lock:
            pos_counts = list(self._last_pos_counts) #4체배 엔코더 펄스
            vel_rpm    = list(self._last_vel_rpm) #모터 속도(감속x) rpm

        for i in range(self.num_axes):
            # 속도 초과
            vmax = self.vel_limit_rpm[i] * self.vel_trip_ratio[i]
            if abs(vel_rpm[i]) > vmax:
                self._trip(f"[axis{i}] velocity trip {vel_rpm[i]:.1f} > {vmax:.1f}")
                return True
            # 위치 절대 한계
            lim = self.pos_abs_limit_cnts[i]
            if lim > 0 and abs(pos_counts[i]) > lim:
                self._trip(f"[axis{i}] position trip {pos_counts[i]} > {lim}")
                return True
        return False

    # ---------- 상태/정지 ----------
    def _set_servo_ok(self, val: bool, reason: str):
        self._servo_ok = val
        self.pub_ok.publish(Bool(data=val))
        self.pub_state.publish(String(data=("READY" if val else "NOT_READY") + (f" ({reason})" if reason else "")))
        self.get_logger().info(f"servo_ok={val} {reason}")

    def _trip(self, reason: str):
        # 모든 축 정지 시도
        self.get_logger().info(f"trip trip")
        for i, node in enumerate(self.nodes):
            if node is None: 
                continue
            try:
                drv.stop_motor(node)
            except Exception:
                pass
        self._set_servo_ok(False, reason)
    def homing(
        self,
        dirs=[1]*5,
        timeouts=[15,15,15,15,15],         # 주기
        homing_joints = [1, 1, 1, 1, 1],
    ):
        # 내부 파라미터(필요시 현장 튠)
        stall_window_s = 0.5        # 펄스 안 변하고 rpm==0가 유지되면 스톨
        pulse_tol = 5            # 펄스 정지 판정 허용오차
        rpm_tol = 0                 # 0이면 정확히 0만 정지로 인정
        stable_cycles = 30           # 성공 조건 연속 확인 횟수
        period_s = 0.05
        # 관절 그룹 구성
        joint_groups = []
        for i in range(5):
            if i <3: #0 1 2 
                joint_groups.append([self.nodes[i] if i < len(self.nodes) else None])
            else: # 3 4 일경우 node에 3 4 추가
                joint_groups.append([self.nodes[3], self.nodes[4]]) 

        J = len(joint_groups)
        self.homing_state = [-1] * J
        results = []

        def _rpm_zero(v):  # 허용오차 적용
            return abs(int(v)) <= rpm_tol

        for j in range(J):
            if homing_joints[j] != 1:
                try:
                    self.get_logger().info(f"[HOMING] J{j+1} passed")
                except Exception:
                    pass
                self.homing_state[j] = -1
                results.append({"joint": j+1, "ok": False, "error": "passed"})
                continue

            motors = joint_groups[j] or []
            actives = [m for m in motors if m is not None]

            if len(actives) == 0:
                self.homing_state[j] = -1
                results.append({"joint": j+1, "skipped": True})
                continue

            # 듀얼 필요한데 하나만 살아있으면 정책 결정: 여기선 실패
            if len(motors) >= 2 and len(actives) == 1:
                self.homing_state[j] = 0
                results.append({"joint": j+1, "ok": False, "error": "missing_slave"})
                continue

            direction = int(dirs[j] if j < len(dirs) else 1)
            timeout_s = float(timeouts[j] if j < len(timeouts) else 10.0)

            try:
                # 리미트 보호 on
                for node in actives:
                    node.sdo["LsEnable"].raw = 1 # active

                # 구동 시작(동시) 일단 100으로 줌
                if j == 0:
                    target = 1000 * direction
                    for node in actives:
                        drv.speed_control(node, target, int(abs(target/1.)), int(abs(target/0.2)))
                elif j in(1, 2): #2, 3은 리밋스위치 사용하지 않음=> 모터 속도, pos로만 측정
                    node.sdo["LsEnable"].raw = 0
                    target = -1000
                    for node in actives:
                        drv.speed_control(node, target, int(abs(target/1.)), int(abs(target/0.2)))
                elif j==3: #pitch
                    target = 400 * direction #dir:1 = +방향 , dir:-1 = -방향 
                    drv.speed_control(actives[0], -target, int(abs(target/1.)), int(abs(target/0.3))) 
                    drv.speed_control(actives[1], target, int(abs(target/1.)), int(abs(target/0.3)))

                elif j==4: #roll
                    target = 400 * direction
                    drv.speed_control(actives[0], -target, int(abs(target/1.)), int(abs(target/0.3)))
                    drv.speed_control(actives[1], -target, int(abs(target/1.)), int(abs(target/0.3)))

                t0 = time.monotonic()
                last_move_t = []
                prev_pulse  = []
                for node in actives:
                    p = node.sdo["Read_MotorPos"].raw
                    prev_pulse.append(p)
                    last_move_t.append(t0)

                ok_stable = 0

                while True:
                    now = time.monotonic()

                    # 상태 읽기(모든 모터)
                    rpms = []
                    pulses = []
                    for i, node in enumerate(actives):
                        r = node.sdo["Read_MotorVel"].raw
                        p = node.sdo["Read_MotorPos"].raw
                        rpms.append(r)
                        pulses.append(p)
                        if p != prev_pulse[i]: # 다를 경우 갱신
                            last_move_t[i] = now
                            prev_pulse[i] = p

                    # 리미트 읽기
                    ls_flags = [node.sdo["Read_MotorLsState"].raw for node in actives]
                    
                    
                    if j in (1, 2):
                        limit_on = 1
                    else:
                        limit_on = all(ls == 3 for ls in ls_flags)

                    # 성공 조건: 리미트 ON + 모든 모터 정지(rpm, pulse)
                    all_rpm_zero = all(_rpm_zero(r) for r in rpms)
                    all_pulse_still = all(abs(pulses[i] - prev_pulse[i]) <= pulse_tol for i in range(len(actives)))
                    if limit_on and all_rpm_zero and all_pulse_still:
                        ok_stable += 1
                        if ok_stable >= stable_cycles:
                            for node in actives:
                                drv.stop_motor(node)
                            for node in actives:
                                node.sdo["LsEnable"].raw = 0
                                drv.reset_encoder(node)

                                
                            self.homing_state[j] = 1
                            results.append({"joint": j+1, "ok": True, "motors": len(actives)})
                            break
                    else:
                        ok_stable = 0

                    # 스톨(어느 모터든)
                    any_stall = (limit_on == 0) and any(((now - last_move_t[i]) > stall_window_s) 
                                                        and _rpm_zero(rpms[i]) 
                                                        and (abs(pulses[i] - prev_pulse[i]) <= pulse_tol) for i in range(len(actives)))
                    if any_stall:
                        for node in actives:
                            drv.stop_motor(node)
                            node.sdo["LsEnable"].raw = 0
                        self.homing_state[j] = 0
                        results.append({"joint": j+1, "ok": False, "error": "stall"})
                        break

                    # 타임아웃
                    if (now - t0) > timeout_s:
                        for node in actives:
                            drv.stop_motor(node)
                            node.sdo["LsEnable"].raw = 0
                        self.homing_state[j] = 0
                        results.append({"joint": j+1, "ok": False, "error": "timeout"})
                        break

                    time.sleep(period_s)

            except Exception as e:
                for node in actives:
                    try:
                        drv.stop_motor(node)
                        node.sdo["LsEnable"].raw = 0
                    except Exception:
                        pass
                self.homing_state[j] = 0
                try:
                    self.get_logger().error(f"[HOMING] J{j+1} exception: {e}")
                except Exception as e:
                    pass
                results.append({"joint": j+1, "ok": False, "error": "exception", "detail": str(e)})
        for r in results:
            if r["ok"]:
                self.get_logger().info(f"[Homing Result] J{r['joint']}: OK")
            else:
                self.get_logger().warn(
                    f"[Homing Result] J{r['joint']}: FAIL "
                    f"(error={r['error']})"
                )
        try:
            for node in self.nodes:
                node.sdo["ResetEncoder"].raw = 0
        except Exception:
            pass


        return results

            #J2 리니어모터는 무조건 음의방향으로 맞추고 리밋스위치가 필요없으니까 모터속도나 펄스 변화 그런 거로 완료됐는지 확인.
                #그러고 reset encoder 해주기
            #J3
                #J2와 마찬가지
            #J4
                #roll방향과 pitch 방향 따로해주기, 그때 node[3], node[4] 에 각각 맞는 모터 속도 주기
            #J5

        # ---------- 종료 ----------
    def destroy_node(self):
        try:
            for node in self.nodes:
                if node is not None:
                    drv.stop_motor(node)
        except Exception:
            pass
        super().destroy_node()
    def scaling(self, data: Float64MultiArray) -> Float64MultiArray:
        """
        Servo가 준 joint 속도 명령(data.data)을
        각 관절 최대속도(self.joint_vel_limit) 기준으로 스케일링한다.

        - ratio_i = |cmd_i| / vmax_i 를 구해서
        - 그 중 가장 큰 ratio_max를 찾고
        - ratio_max > 1이면 전체 벡터를 1/ratio_max 만큼 줄인다.
        """
        cmds = list(data.data)  # 복사해서 작업
        n = min(len(cmds), len(self.joint_vel_limit))

        max_ratio = 0.0

        # 1) 최대 ratio 찾기
        for i in range(n):
            cmd = cmds[i]
            vmax = self.joint_vel_limit[i]

            ratio = abs(cmd) / vmax
            if ratio > max_ratio:
                max_ratio = ratio

        # 2) ratio가 1보다 크면 전체 스케일링
        if max_ratio > 1.0:
            scale = 1.0 / max_ratio
            for i in range(len(cmds)):
                cmd = cmds[i]
                cmds[i] = cmd * scale
        # ratio <= 1이면 그대로 통과

        out = Float64MultiArray()
        out.data = cmds
        return out


def main():
    rclpy.init()
    node = SERVO_CAN_BRIDGE()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
