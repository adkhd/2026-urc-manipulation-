import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
import time
import math
import sys
from rmd_control.can_interface import bringup_can_interface
from rmd_control.rmd_driver_4_3 import RMDMotorDriver as rmd43
from rmd_control.rmd_driver_4_01 import RMDMotorDriver as rmd401
from rmd_control.kinematics_angle import calculate_kinematics as fk, calculate_ik as ik
from rmd_control.kinematics_vel import KinematicsSolver
from rclpy.executors import MultiThreadedExecutor
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class RMDControlNode(Node):
    def __init__(self):
        super().__init__('rmd_control_node')
        self.last_command_time = self.get_clock().now()
        self.COMM_TIMEOUT_SEC = 0.5  # 0.5초 이상 명령 없으면 통신 두절로 간주
        # 1. 설정 변수 초기화
        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint7']
        self.motor_ids = [1, 2, 3, 4, 5]  # CAN ID 매핑
        self.num_joints = 5
        self.consecutive_error_count = 0
        # 상태 변수 (Radian 단위)

        #threading
        self.pub_thread = threading.Thread(target=self.thread_pub_loop)
        self.pub_thread.daemon = True # 노드 종료 시 같이 종료되도록 설정
        
        self.solver = KinematicsSolver()
        
        self.current_motor_degrees = [0.0] * 5

        self.current_joint_pos = [0.0] * self.num_joints
        self.target_joint_pos = [0.0] * self.num_joints
        
        self.is_hardware_ready = False
        self.drivers = {} # {id: RMDMotorDriver객체}
        self.control_Hz = 150

        self.MAX_ERROR_THRESHOLD = 10
        # 2. ROS 통신 설정
        # MoveIt Servo 명령 구독 (토픽명은 실제 MoveIt 설정에 맞게 변경)
        high_performance_qos = QoSProfile(
            depth=1,                                   # 대기열 크기 1 (핵심)
            history=HistoryPolicy.KEEP_LAST,           # 마지막 데이터만 유지
            reliability=ReliabilityPolicy.RELIABLE     # 데이터 도착 보장 (호환성 유지)
        )
        self.sub_command = self.create_subscription(
            JointTrajectory,
            '/arm_controller/commands',  # 혹은 '/move_group/display_planned_path' 등 실제 토픽명
            self.command_callback,
            qos_profile=high_performance_qos
        )

        # JointState 발행
        self.pub_joint_state = self.create_publisher(
            JointState,
            '/joint_states',
            qos_profile=high_performance_qos
        )
        
        # 3. 초기 시퀀스 실행
        if self.startup_sequence():
            # [수정됨] 제어 루프 대신 상태 발행 루프(pub_joint_state)를 연결
            self.pub_thread.start()
        else:
            self.get_logger().error("시스템 초기화 실패. 노드를 종료합니다.")
            sys.exit(1)
    def thread_pub_loop(self):
        rate = 1.0 / self.control_Hz
        while rclpy.ok():
            start_time = time.time()
            
            # 기존 publish_callback 내용을 여기에 직접 실행
            if self.is_hardware_ready:
                try:
                    # 1. Read & FK
                    fine_angles = self.cal_fine_motor_angles()
                    self.current_motor_degrees = fine_angles
                    self.current_joint_pos = self.calc_all_fk(fine_angles)
                    
                    # 2. Publish
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = self.joint_names
                    msg.position = self.current_joint_pos + [0.0]
                    self.pub_joint_state.publish(msg)
                    
                except Exception as e:
                    pass # 에러 처리는 기존 로직 참고

            # 주기 맞추기 (Sleep)
            elapsed = time.time() - start_time
            sleep_time = rate - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    def startup_sequence(self):
        """
        [초기화 시퀀스]
        CAN 설정 -> 드라이버 연결 -> 에러 체크 -> 초기 위치 동기화
        """
        self.get_logger().info("--- 초기화 시퀀스 시작 ---")

        # 단계 1: CAN 인터페이스 활성화
        if not bringup_can_interface(tty="/dev/canable0", ifname="can0"):
            self.get_logger().error("CAN 인터페이스 설정 실패")
            return False

        try:
            # 단계 2: 모터 드라이버 인스턴스 생성 및 연결
            for idx, mid in enumerate(self.motor_ids):
                self.get_logger().info(f"모터 ID {mid} 연결 시도 중...")
                if mid in [1, 4]:
                    driver = rmd401(channel='can0', motor_id=mid)
                else:
                    driver = rmd43(channel='can0', motor_id=mid)
                self.drivers[mid] = driver
                
                # 통신 확인을 위해 상태 읽기 요청
                driver.read_motor_status_1()
                time.sleep(0.1) # 응답 대기 (비동기 처리 때문)

                # 연결 확인 (State가 업데이트 되었는지 확인 필요, 여기서는 에러코드로 간접 확인)
                if driver.state['error_code'] != 0:
                     # 에러 발생 시 처리
                    err_msg = driver.state['error_string']
                    self.get_logger().warn(f"모터 {mid} 초기 에러 감지: {err_msg}")
                    self.get_logger().error("원인 파악 및 해결 후 노드를 재실행하십시오.")
                    return False

            # homing_motors = [4]

            # for mid in homing_motors:
            #     self.get_logger().info(f"모터 {mid} 호밍 시작...")
            #     homing_speed = 20
            #     homing_curr = 0.6
            #     timeout_sec = 18
            
            #     if not self.drivers[mid].perform_homing_task(homing_speed, homing_curr, timeout_sec):
            #         self.get_logger().error(f"모터 {mid} 호밍 실패!")
            #         return False
            #     self.get_logger().info(f"모터 {mid} 호밍 완료 및 영점 설정됨.")

            # 4. Active Reply (0xB6) 설정
            # 제어 주기(0.02s = 20ms)에 맞춰서 모터가 스스로 상태(0x9C)를 보내게 함
            self.get_logger().info("모터 Active Reply(20ms) 설정 중...")
            for mid in self.motor_ids:
                driver = self.drivers[mid]
                # 0x92 명령 멀티턴 각도(0.01도의 resolution)을 20ms마다 수신
                # 0x9c 전류/각속도/각도
                driver.set_active_reply(cmd_byte=0x9C, enable=1, interval_ms=100) #전류, speed 정보를 얻기 위함 state['current']  state['speed']
                driver.set_active_reply(cmd_byte=0x92, enable=1, interval_ms=1000./self.control_Hz) # state['multi_angle']

            # 5. 데이터 수신 대기 (Active Reply가 들어올 때까지 잠시 대기)
            time.sleep(1.0)
            
            # 단계 3: 초기 위치 읽고 모터 각도 정렬

            self.get_logger().info("초기 위치 읽기 중...")
            fine_motor_positions = self.cal_fine_motor_angles()
            self.drivers[4].angle_offset = -130.65 + self.drivers[4].state['multi_angle']
            print(f"4번 모터 초기각도 보정값: {self.drivers[4].angle_offset}")
            time.sleep(1)
            fine_motor_positions = self.cal_fine_motor_angles()
            #초기 작동시 모터각도 범위 확인
            if not self.check_motor_limits(fine_motor_positions):
                self.get_logger().error("모터 허용각도 벗어남")
                return False

            #초기 위치 세팅
            if not self.set_ready_pos():
                self.get_logger().error("초기 pos 위치 실패")
                return False
            
            # 초기 위치 설정 후 fine_angle => calc fk
            # motor_pos 업데이트
            self.current_joint_pos = self.calc_all_fk(self.cal_fine_motor_angles()) #joint pos 단위
            self.target_joint_pos = list(self.current_joint_pos)  #joint pos
            self.get_logger().info(f"목표 조인트 포즈 {self.target_joint_pos}")
            # 단계 5: 안전 범위 체크
            if not self.check_Joint_limits(self.current_joint_pos):
                 self.get_logger().error("관절 허용 범위를 벗어남")
                 return False

            self.is_hardware_ready = True
            self.get_logger().info("--- 하드웨어 준비 완료. 제어 루프 대기 ---")
            self.last_command_time = self.get_clock().now()
            return True

        except Exception as e:
            self.get_logger().error(f"초기화 중 예외 발생: {e}")
            return False

    def command_callback(self, msg):
            """MoveIt Servo로부터 목표값 수신 -> 안전 체크 후 모터 제어"""
            if not self.is_hardware_ready:
                return
            
            if not msg.points:
                return
            point = msg.points[0]

            if len(point.positions) < self.num_joints:
                return
                
            self.last_command_time = self.get_clock().now()
            
            # --- [핵심 수정: 급발진 방지 안전장치] ---
            # MoveIt이 보낸 목표값과 현재 로봇 위치의 차이(Error)를 계산합니다.
            # 만약 차이가 너무 크면(예: 10도 이상), MoveIt이 잘못된(튀는) 명령을 보낸 것이므로 무시합니다.
            target_pos_list = list(point.positions[:self.num_joints])
            target_vel_list = list(point.velocities[:self.num_joints])
            SAFE_GOAL_TOLERANCE_RAD = 0.4  # 약 17도 (상황에 따라 0.1~0.5 조절)
            
            is_safe = True
            for i, target_pos in enumerate(target_pos_list):
                current_pos = self.current_joint_pos[i]
                diff = abs(target_pos - current_pos)
                
                if diff > SAFE_GOAL_TOLERANCE_RAD:
                    self.get_logger().warn(
                        f"[Safety] 급격한 명령 감지! J{i+1} "
                        f"현재: {current_pos:.2f}, 명령: {target_pos:.2f}, 차이: {diff:.2f}"
                    )
                    is_safe = False
                    break
            
            if not is_safe:
                return
            # ------------------------------------------

            self.target_joint_pos = target_pos_list

            if not self.check_Joint_limits(self.target_joint_pos):
                self.get_logger().warn("목표 관절 범위 벗어남. 명령 무시.")
                return 

            # IK 계산 (Joint Angle -> Motor Angle)
            target_motor_degrees = self.calc_all_ik(self.target_joint_pos)
            
            # [수정됨] 위치와 속도 리스트를 함께 전달
            self.all_pos_control(target_motor_degrees, target_vel_list)
        
    def publish_callback(self):
        """
        [상태 발행 루프] Hz
        READ -> FK -> PUB (No Control)
        """
        if not self.is_hardware_ready:
            return
        time_diff = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if time_diff > self.COMM_TIMEOUT_SEC:
            # 1. 경고 메시지 (너무 자주 뜨지 않게 throttle 조절 필요)
            # 2. 필요 시 비상 정지 로직 (현재 위치 유지)
            # self.stop_all_motor() # 혹은 현재 위치를 타겟으로 덮어쓰기
            #self.stop_all_motor()
            self.get_logger().warn(f"Timeout: 0.5초 이상 명령 없음! (Diff: {time_diff:.2f})", throttle_duration_sec=1.0)
        try:
            # --- [1. READ & FK] ---
            # Active Reply로 갱신된 값 읽기
            fine_angles = self.cal_fine_motor_angles() 
            self.current_motor_degrees = fine_angles 
            self.current_joint_pos = self.calc_all_fk(fine_angles)
            gripper_pos = 0.0

            full_joint_pos = self.current_joint_pos + [gripper_pos]
            # --- [2. PUB] ---
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = full_joint_pos
            self.pub_joint_state.publish(msg)

            # 통신 성공으로 간주 (읽기 성공 시)
            self.consecutive_error_count = 0 

        except Exception as e:
            self.consecutive_error_count += 1
            # 에러 로그가 너무 많이 뜨면 주석 처리
            # self.get_logger().warn(f"상태 읽기 에러: {e}")

            if self.consecutive_error_count >= self.MAX_ERROR_THRESHOLD:
                self.get_logger().error("통신 에러 임계치 초과! 비상 정지.")
                self.stop_all_motor()
                self.is_hardware_ready = False
                sys.exit(1)
    
    def calc_all_fk(self, motor_degrees):
        """
        모터 5개의 fine 각도(deg) 리스트를 받아 관절 5개의 각도(rad) 리스트 반환
        """
        joint_pos = []
        for mid in [1, 2, 3, 4, 5]:
            if mid == 1:
                joint_pos.append(motor_degrees[mid-1]/360.*0.01) # m
            elif mid == 2:
                joint_pos.append(math.radians(motor_degrees[mid-1])) #rad
            elif mid == 3:
                joint_pos.append(fk(motor_degrees[mid-1]-motor_degrees[mid-2])) #rad
            elif mid == 4:
                joint_pos.append(math.radians(motor_degrees[mid-1])) #rad
            elif mid == 5:
                joint_pos.append(math.radians(motor_degrees[mid-1])) #rad
        return joint_pos

    def calc_all_ik(self, joint_pos):
        """
        관절 5개의 목표 각도(rad)를 받아 모터 5개의 목표 각도(deg) 반환
        """
        motor_fine_pos = []
        
        for mid in [1, 2, 3, 4, 5]:
            if mid ==1: #m => rad
                motor_fine_pos.append(joint_pos[mid-1]/0.01 * 2*math.pi)
            elif mid == 2: #rad
                motor_fine_pos.append(joint_pos[mid-1])
            elif mid == 3: #rad
                motor_fine_pos.append(joint_pos[mid-2]+ik(joint_pos[mid-1]))
            elif mid == 4: #rad
                motor_fine_pos.append(joint_pos[mid-1])
            elif mid == 5: #rad
                motor_fine_pos.append(joint_pos[mid-1])
        motor_fine_pos_degrees = [math.degrees(rad) for rad in motor_fine_pos]
        return motor_fine_pos_degrees
    def cal_fine_motor_angles(self): 
        #5개의 fine motor angle 리스트 반환
        fine_motor_angle = []
        motor_directions = [1, 1, -1, 1, 1]
        for mid in self.motor_ids:
                # Active Reply 덕분에 driver.state['angle']이 최신값임
                if mid in [2, 3]:
                    angle = self.drivers[mid].state['multi_angle'] * motor_directions[mid-1] + 90 # J1모터 10도 = 관절 100도 = 10 + 90 #J3 모터 20도 = 관절각 70도 = -20+90
                else:
                    angle = self.drivers[mid].state['multi_angle'] * motor_directions[mid-1] 
                fine_motor_angle.append(angle)
        return fine_motor_angle

    def check_motor_limits(self, motor_degrees):
            """
            IK 계산 후 나온 모터 각도(deg)가 하드웨어 허용 범위인지 체크
            입력: 5개의 모터 각도 리스트 [deg1, deg2, deg3, deg4, deg5]
            """
            # [Min, Max] (사용자 주석 기반 설정)
            # J1: 범위가 특이하여 확인 필요, 일단 넉넉하게 잡거나 주석대로 설정
            # J5: limitless라고 하셨으므로 -inf, inf
            limits = [
                [-180, 10800],  # J1 (주석이 모호하여 넓게 잡음, 필요시 수정)
                [-40, 270],       # J2
                [-90, 270],         # J3
                [-135, 135],      # J4
                [-float('inf'), float('inf')] # J5 (Limitless)
            ]

            for i, deg in enumerate(motor_degrees):
                min_val, max_val = limits[i]
                if not (min_val <= deg <= max_val):
                    self.get_logger().warn(f"[Limit] 모터 {i+1}번 각도 {deg:.2f}가 범위({min_val}~{max_val})를 벗어남")
                    return False
            return True

    def check_Joint_limits(self, joint_positions):
        # 관절 한계값 체크 로직
        # joint_positions = 다음 타겟 관절 pos 크기 5의 리스트
        # 관절 범위를 벗어나는 명령일 경우 false 리턴
        # J1 : >= 0, J2 : -20 ~ 200, J3 : -137 ~ -22, J4: -150 ~ 150, J5: limit less 
        #실제 조인트 리밋을 확인해보고, 하는 것이 좋아보임.
        limits_rad = [
            [0.000, 0.255], # J1 m단위
            [-0.3, 3.141],   # J2 (약 -28 ~ 180도) 계산해보기
            [-2.6564, -0.615],    # J3 (-137 ~ 22도)(수정 필요) 계산해보기
            [-2.25, 2.25],   # J4 -130~ 130
            [-100, 100]  # J5
        ]

        for i, pos in enumerate(joint_positions):
            min_val, max_val = limits_rad[i]
            if not (min_val <= pos <= max_val):
                self.get_logger().warn(f"[Limit] 관절 {i+1}번 {pos:.2f}rad 범위 이탈")
                return False
        return True

    def set_ready_pos(self):
        #ready pos는 직접 정함
        #조인트별로 순차적으로 pos control을 함
        # 한 조인트 pos위치가 성공했으면(0.5도 내) 해당 motor stop하고 다음 모터로 이동
        # 만약 (vel = 0이고 angle이 일치하지 않거나)or(타임아웃일 경우) false 내보내기
        # J1 = 0 => J2 = 110 => J3 = 140 => J4 = 0 => J5 = 0
        """
        [초기 자세 순차 이동]
        J1 -> J2 -> J3 -> J4 -> J5 순서로 이동
        목표: [0, 110, 140, 0, 0]
        """
        self.get_logger().info("Ready Pose 순차 이동 시작...")
        
        # 목표 각도 (Fine Angle 기준)
        target_ready_angles = [360, 150, 180, -30, 0]
        
        # 각 관절별 허용 오차 및 타임아웃 설정
        ERROR_TOLERANCE = 0.3  # 도
        TIMEOUT_SEC = 8.0      # 관절당 최대 대기 시간
        
        for i, target in enumerate(target_ready_angles):
            mid = self.motor_ids[i]
            self.get_logger().info(f"모터 {mid}번 이동 시작 -> 목표: {target}도")
            
            # 1. 이동 명령 전송
            self._pos_control(mid, target)
            
            start_time = time.time()
            success = False
            
            # 2. 도달 확인 루프
            while (time.time() - start_time) < TIMEOUT_SEC:
                # 상태 갱신 (Active Reply가 오고 있더라도 명시적으로 최신값 확인 권장)
                # 루프 속도 조절
                time.sleep(0.05) 
                try:
                    fine_angles = self.cal_fine_motor_angles()
                    curr_fk = self.calc_all_fk(fine_angles)
                    gripper = 0.0
                    
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = self.joint_names
                    msg.position = curr_fk + [gripper]
                    self.pub_joint_state.publish(msg) # 강제 발행
                except Exception:
                    pass
                # 현재 모든 모터의 Fine Angle 계산 (현재 루프의 모터값만 쓰면 됨)
                current_fine_angles = self.cal_fine_motor_angles()
                current_pos = current_fine_angles[i]
                
                # 현재 속도 확인 (Stall 체크용) - state['speed']는 Active Reply로 갱신됨
                current_vel = abs(self.drivers[mid].state['speed'])
                
                # 오차 계산
                error = abs(target - current_pos)
                
                # [조건 A] 성공: 오차가 허용범위 이내
                if error <= ERROR_TOLERANCE:
                    self.get_logger().info(f"모터 {mid}번 도달 성공. (오차: {error:.2f})")
                    success = True
                    self.drivers[mid].stop_motor()
                    break
                
                # [조건 B] 실패: 속도는 0인데 각도가 맞지 않음 (Stall/끼임)
                # 속도 5dps 이하이고, 오차가 2도 이상이면 문제로 간주
                if current_vel < 5 and error > 2.0:
                    # 잠깐 멈칫한 것일 수 있으니 시간을 좀 더 두고 판단해야 하지만,
                    # 여기선 단순화를 위해 타임아웃까지 기다리거나 바로 경고
                    pass # 루프 계속 돌면서 타임아웃 체크

            if not success:
                self.get_logger().error(f"모터 {mid}번 Ready Pose 이동 실패! (Timeout or Stall)")
                self.get_logger().error(f"현재각: {current_pos:.2f}, 목표각: {target}, 속도: {current_vel}")
                # 실패 시 전체 정지 후 종료
                self.stop_all_motor()
                return False
                
            # 성공 시, 해당 모터는 Position Control 상태로 홀딩(Hold) 유지
            # 다음 모터로 진행

        self.get_logger().info("모든 모터 Ready Pose 도달 완료.")
        return True
    def all_pos_control(self, target_pos, target_vels):
        #보내지는 건 모터 엔코더값이 되기 전인 fine angle
        # fine angle => raw angle
        #target pos 는 크기 5의 리스트
        #변환식은 J1, J4, J5는 그대로
        #J2는 raw_angle + 90 = fine angle
        #J3는 (-raw_angle + 90) = fine angle
        #모터를 굴릴 땐 self.drivers[mid].pos_control(angle_raw, max_dps) 와 같은 식으로 줌
        # 일단 max_dps를 일정한 값으로 설정을 해두고(관절별) 나중에 제어방식을 변경해보기
        """
        [전체 모터 위치 제어]
        입력: target_pos (Fine Angle 리스트, 단위: Degree)
        기능: Fine Angle -> Raw Angle 변환 후 모터 드라이버에 명령 전송
        """
        # 안전을 위한 최대 속도 제한 (관절별로 다르게 설정 가능하나 일단 통일)
        
        for i, fine_angle in enumerate(target_pos):
            mid = self.motor_ids[i]
            desired_joint_vel_rad = target_vels[i] # J1 = m/s, ~ rad/s
            raw_angle = 0.0
            calc_dps = 0.0
            if mid == 1:
                # FK 공식: 360 deg = 0.01 m
                # 즉, 1 m = 36000 deg
                # 속도(deg/s) = 속도(m/s) * 36000
                calc_dps = abs(desired_joint_vel_rad) * 36000.0
            elif mid == 3:
                # [J3 특수 처리]
                # 현재 J3 관절 각도 (rad)
                current_j3_angle = self.current_joint_pos[2]
                
                # solver의 kinematic_vel 함수 사용
                # 입력: 현재 J3- 각도, 목표 J3 속도
                # 출력: 실제 모터 구동 속도 (rad/s라 가정하고 deg/s로 변환)
                vel_j2_rad = target_vels[1]
                
                motor_vel_rad = vel_j2_rad + self.solver.calculate_safe_velocity(ik(current_j3_angle), desired_joint_vel_rad)
                calc_dps = math.degrees(abs(motor_vel_rad))
                calc_dps=calc_dps
            else:
                # [나머지 관절]
                # Joint Velocity(rad/s) -> Motor Velocity(deg/s)
                # J1의 경우 감속비 등을 고려해야 한다면 여기서 비율을 곱해야 함 (현재는 1:1 가정)
                calc_dps = math.degrees(abs(desired_joint_vel_rad))
                calc_dps=calc_dps
            
            
            # 모터 스펙에 따른 최대 속도 제한 (예: 3000dps)
            if calc_dps > 5000.0: calc_dps = 5000.0

            send_speed_dps = int(calc_dps)
            # --- [변환 로직] Fine -> Raw ---
            if mid == 2:
                # 공식: Fine = Raw + 90  =>  Raw = Fine - 90
                raw_angle = fine_angle - 90.0
            elif mid == 3:
                # 공식: Fine = -Raw + 90 =>  Raw = 90 - Fine
                raw_angle = 90.0 - fine_angle
            else:
                # J1, J4, J5 (변환 없음)
                raw_angle = fine_angle

            # --- [명령 전송] ---
            # 드라이버의 위치 제어 함수 호출 (Closed Loop Control)
            # 0xA4 명령 (Angle, MaxSpeed) 사용 가정
            self.drivers[mid].pos_control(raw_angle, send_speed_dps)

    def _pos_control(self, mid, target_pos):
        #한 관절의 모터만 pos_control
        #all pos control과 비슷하게 진행함. 하지만 모터id를 받아서 target_pos를 수행하는 것 외엔 다름이 없음.
        raw_angle = 0.0
        # 초기 거동용 속도 (조금 천천히 설정)
        READY_SPEED_DPS = [120, 30, 30, 30, 30]

        # --- [변환 로직] ---
        if mid == 2:
            raw_angle = target_pos - 90.0
        elif mid == 3:
            raw_angle = 90.0 - target_pos
        else:
            raw_angle = target_pos

        # 명령 전송
        self.drivers[mid].pos_control(raw_angle, READY_SPEED_DPS[mid-1])
    def stop_all_motor(self):
        for mid in self.motor_ids:
            self.drivers[mid].stop_motor()
        #모든 모터를 종료함. 브레이킹 힘은 유지
    def shutdown_all_motor(self):
        for mid in self.motor_ids:
            self.drivers[mid].shutdown_motor()
        #모든 모터를 종료함. 브레이킹 힘은 유지
def main(args=None):
    rclpy.init(args=args)
    node = RMDControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("종료 요청")
    finally:
        # 종료 처리
        node.stop_all_motor()
        # node.drivers 정리...
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()