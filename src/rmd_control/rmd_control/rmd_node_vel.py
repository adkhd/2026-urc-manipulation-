import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_srvs.srv import Trigger
import time
import math
import sys
import threading
import numpy as np
from rmd_control.kinematics_vel import KinematicsSolver 
from rmd_control.kinematics_angle import calculate_kinematics as fk, calculate_ik as ik
from rmd_control.can_interface import bringup_can_interface
from rmd_control.rmd_driver_4_3 import RMDMotorDriver as rmd43
from rmd_control.rmd_driver_4_01 import RMDMotorDriver as rmd401
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
class LowPassFilter:
    def __init__(self, alpha, num_joints):
        """
        간단한 알파값 기반 LPF
        alpha: 0~1 사이 값. 1에 가까울수록 필터링 약함(반응 빠름)
        """
        self.num_joints = num_joints
        self.alpha = alpha
        self.prev_val = np.zeros(num_joints)
        self.is_initialized = False
        

    def filter(self, new_val):
        new_val = np.array(new_val)
        if not self.is_initialized:
            self.prev_val = new_val
            self.is_initialized = True
            return new_val
        
        filtered_val = self.alpha * new_val + (1.0 - self.alpha) * self.prev_val
        self.prev_val = filtered_val
        return filtered_val.tolist()

class RMDControlNode(Node):
    def __init__(self):
        super().__init__('rmd_control_node_vel')
        self.last_command_time = self.get_clock().now()
        self.COMM_TIMEOUT_SEC = 0.5
        
        # 1. 설정 변수
        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint7']
        self.motor_ids = [1, 2, 3, 4, 5]
        self.num_joints = 5
        self.control_Hz = 150.0  # 제어 주기
        self.solver = KinematicsSolver()
        # [수정] Velocity Limits (J1은 0.03m/s로 제한)
        self.vel_limits = [0.1, 1.0, 1.0, 1.0, 2.0]  # [m/s, rad/s, rad/s, rad/s, rad/s]

        # [수정] LPF 설정 (알파값 사용: 0.2 = 강한 필터링)
        self.lpf = LowPassFilter(alpha=0.2, num_joints=self.num_joints)

        #0.03m/s = 180 rpm
        self.j1_max_rpm = 180.0

        self.drivers = {}
        self.current_joint_pos = [0.0] * self.num_joints
        self.current_joint_vel = [0.0] * self.num_joints  # [추가] 속도 저장
        self.target_joint_pos = [0.0] * self.num_joints
        self.is_hardware_ready = False
        
        # Threading
        self.pub_thread = threading.Thread(target=self.thread_pub_loop, daemon=True)
        
        # 2. ROS 통신
        high_performance_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # MoveIt Servo 명령 구독
        self.sub_command = self.create_subscription(
            JointTrajectory,
            '/arm_controller/commands',  # [수정] 올바른 토픽명
            self.command_callback,
            qos_profile=high_performance_qos
        )

        self.pub_joint_state = self.create_publisher(
            JointState,
            '/joint_states',
            qos_profile=high_performance_qos
        )
        self.servo_start_client = self.create_client(Trigger, '/servo_node/start_servo')
        # 3. 초기화 실행
        if self.startup_sequence():
            self.pub_thread.start()
        else:
            self.get_logger().error("초기화 실패. 종료합니다.")
            sys.exit(1)

    def startup_sequence(self):
        self.get_logger().info("--- 초기화 시퀀스 시작 ---")
        if not bringup_can_interface(tty="/dev/canable0", ifname="can0"):
            return False

        try:
            for mid in self.motor_ids:
                if mid in [1, 4]:
                    driver = rmd401(channel='can0', motor_id=mid)
                else:
                    driver = rmd43(channel='can0', motor_id=mid)
                self.drivers[mid] = driver
                time.sleep(0.05)
                
                status = driver.read_motor_status_1()
                time.sleep(0.1)
                if status is None:
                    self.get_logger().warn(f"모터 {mid} 상태 읽기 실패")
                    return False
                
                if driver.state['error_code'] != 0:
                    self.get_logger().warn(f"모터 {mid} 에러: {driver.state['error_string']}")
                    return False

            # Active Reply 설정
            for mid in self.motor_ids:
                self.drivers[mid].set_active_reply(cmd_byte=0x9C, enable=True, interval_ms=500) 
                self.drivers[mid].set_active_reply(cmd_byte=0x92, enable=True, interval_ms=int(1000/self.control_Hz))
            
            time.sleep(1.0)
            
            # [추가] 4번 모터 오프셋 설정
            self.drivers[4].angle_offset = -130.65 + self.drivers[4].state['multi_angle']
            self.get_logger().info(f"4번 모터 오프셋: {self.drivers[4].angle_offset}")
            time.sleep(1.0)
            
            # 초기 위치 읽기
            for mid in self.motor_ids:
                angle = self.drivers[mid].read_multi_turn_angle()
                if angle is None:
                    self.get_logger().warn(f"모터 {mid} 각도 읽기 실패")
            
            fine_angles = self.cal_fine_motor_angles()
            
            # [추가] 모터 리미트 체크
            if not self.check_motor_limits(fine_angles):
                self.get_logger().error("모터 허용각도 벗어남")
                return False
            
            # [추가] Ready Position 이동
            if not self.set_ready_pos():
                self.get_logger().error("초기 pos 위치 실패")
                return False
            
            # FK 계산
            self.current_joint_pos = self.calc_all_fk(self.cal_fine_motor_angles())
            
            # [추가] 관절 리미트 체크
            if not self.check_Joint_limits(self.current_joint_pos):
                self.get_logger().error("관절 허용 범위를 벗어남")
                return False
            
            self.is_hardware_ready = True
            self.get_logger().info("--- 하드웨어 준비 완료 (Velocity Control Mode) ---")

            self.get_logger().info("MoveIt Servo 시작 요청 중...")
            
            # 서비스가 뜰 때까지 최대 2초 기다림 (보통 Launch가 동시에 켜지므로 금방 뜸)
            if self.servo_start_client.wait_for_service(timeout_sec=2.0):
                req = Trigger.Request()
                # 비동기 호출 (응답을 기다리느라 멈추지 않게 함)
                future = self.servo_start_client.call_async(req)
                # 콜백을 등록하여 성공 여부 로그 출력 (선택 사항)
                future.add_done_callback(self.servo_start_response_callback)
            else:
                self.get_logger().warn("경고: Servo 서비스를 찾을 수 없어 자동 시작에 실패했습니다.")
                self.get_logger().warn("수동으로 'ros2 service call /servo_node/start_servo ...'를 실행해주세요.")
            self.last_command_time = self.get_clock().now()
            return True

        except Exception as e:
            self.get_logger().error(f"초기화 예외: {e}")
            import traceback
            traceback.print_exc()
            return False
    def command_callback(self, msg):
        """JointTrajectory 메시지 수신 (velocities 사용)"""
        if not self.is_hardware_ready:
            return

        if not msg.points:
            self.get_logger().warn("Empty trajectory points received")
            return
            
        self.last_command_time = self.get_clock().now()
        
        # 1. 목표 속도 추출
        if len(msg.points[0].velocities) < self.num_joints:
            self.get_logger().warn(f"Insufficient velocities: {len(msg.points[0].velocities)}")
            return
        if len(msg.points[0].positions) < self.num_joints:
            self.get_logger().warn(f"Insufficient velocities: {len(msg.points[0].velocities)}")
            return
        target_vels = list(msg.points[0].velocities)
        target_pos = list(msg.points[0].positions)

        # 2. Low Pass Filter 적용
        filtered_vels = self.lpf.filter(target_vels)
        if not self.check_Joint_limits(target_pos):
            self.stop_all_motor()
            self.get_logger().warn(f"target pos 벗어남")
            return
        
        # 3. Joint Limits (Velocity) 적용 및 모터 구동
        self.process_velocity_command(filtered_vels, target_pos)
    def servo_start_response_callback(self, future):
        """서비스 호출 결과를 비동기로 확인"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Servo 시작 성공: {response.message}")
            else:
                self.get_logger().warn(f"Servo 시작 실패: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Servo 서비스 호출 중 오류 발생: {e}")

    # ... (나머지 command_callback, process_velocity_command 등은 기존과 동일) ...
    # ... (thread_pub_loop, cal_fine_motor_angles, calc_all_fk, stop_all_motor 등 동일) ...
    def process_velocity_command(self, joint_vels, target_pos):
        """
        관절 속도(rad/s 또는 m/s) -> 모터 속도(dps) 변환 및 전송
        J3 Coupling 처리 포함
        """
        for i, vel in enumerate(joint_vels):
            # 1. Velocity Limit Clipping
            limit = self.vel_limits[i]
            vel = max(-limit, min(limit, vel))
            
            mid = self.motor_ids[i]
            motor_dps = 0.0

            # 2. Kinematics & Coupling (Joint Vel -> Motor Vel)
            if mid == 1:
                #조이스틱에서 리니어1을 움직이는 명령은 최대 0.03으로 제한둠 다른 건 0.1 m/s
                # [수정] J1 Linear Joint: m/s -> dps
                # 가정: 1회전(360도) = 0.01m (1cm) 피치
                # vel_m_s * (360도 / 0.01m) = vel_m_s * 36000 deg/s
                motor_dps = vel * 36000.0
                
                # [추가] RPM 제한 (150 RPM = 900 deg/s)
                max_dps = self.j1_max_rpm * 6.0  # 150 RPM * 6 = 900 deg/s
                motor_dps = max(-max_dps, min(max_dps, motor_dps))
            
            elif mid == 2:
                # Joint 2: rad/s -> dps (방향: 1)
                motor_dps = math.degrees(vel)
            
            elif mid == 3:
                # [중요] J3 Coupling 처리
                # J3 모터 = J2 속도 보상 + J3 고유 속도
                j2_vel = joint_vels[1]  # J2 관절 속도 (rad/s)
                j3_vel = vel            # J3 관절 속도 (rad/s)
                
                # Coupling: M3 = M2 + J3 (링키지 보상)
                coupled_vel = j2_vel + self.solver.calculate_safe_velocity(ik(target_pos[2]),j3_vel)
                
                # 모터 방향(-1) 적용
                motor_dps = math.degrees(coupled_vel) * -1.0

            elif mid == 4:
                # Joint 4: rad/s -> dps (방향: 1)
                motor_dps = math.degrees(vel)
                
            elif mid == 5:
                # Joint 5: rad/s -> dps (방향: 1)
                motor_dps = math.degrees(vel)

            # 3. 명령 전송 (0xA2 Velocity Control)
            try:
                self.drivers[mid].vel_control(speed_dps=motor_dps)
            except Exception as e:
                self.get_logger().error(f"Motor {mid} vel_control failed: {e}")

    def thread_pub_loop(self):
        """상태 모니터링 및 발행 루프 (150Hz)"""
        rate = 1.0 / self.control_Hz
        while rclpy.ok():
            start_time = time.time()
            
            if self.is_hardware_ready:
                # 통신 타임아웃 체크
                time_diff = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
                if time_diff > self.COMM_TIMEOUT_SEC:
                    # 타임아웃 시 정지
                    self.get_logger().warn(f"Motor time out")
                    self.stop_all_motor()
                
                try:
                    # 1. 모터 각도 읽기 (Active Reply 버퍼값 사용)
                    fine_angles = self.cal_fine_motor_angles()
                    
                    # 2. FK 계산
                    self.current_joint_pos = self.calc_all_fk(fine_angles)

                    # 4. JointState 발행
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = self.joint_names
                    msg.position = self.current_joint_pos + [0.0]
                    self.pub_joint_state.publish(msg) # 강제 발행
                    
                    self.pub_joint_state.publish(msg)
                    
                except Exception as e:
                    self.get_logger().error(f"Pub loop error: {e}")

            elapsed = time.time() - start_time
            sleep_time = rate - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def cal_fine_motor_angles(self): 
        """모터 각도 -> Fine 각도 변환"""
        fine_motor_angle = []
        motor_directions = [1, 1, -1, 1, 1]
        
        for i, mid in enumerate(self.motor_ids):
            angle = self.drivers[mid].state['multi_angle'] * motor_directions[i]
            
            # J2, J3는 +90도 오프셋
            if mid in [2, 3]:
                angle += 90.0
            
            fine_motor_angle.append(angle)
        
        return fine_motor_angle

    def calc_all_fk(self, motor_degrees):
        """
        모터 각도 -> 관절 각도 변환 (FK)
        [수정] J3 커플링 고려
        """
        joint_pos = []
        
        # J1: Linear (degrees -> meters)
        # 360도 = 0.01m
        joint_pos.append(motor_degrees[0] / 36000.0)
        
        # J2: Rotational
        joint_pos.append(math.radians(motor_degrees[1]))
        
        # J3: Coupled (M3 - M2)
        # J3 관절각 = M3각도 - M2각도 (커플링 보정)
        j3_motor_angle = motor_degrees[2]
        j2_motor_angle = motor_degrees[1]
        j3_joint_angle = fk(j3_motor_angle - j2_motor_angle) # deg => rad
        joint_pos.append(j3_joint_angle)
        
        # J4, J5: Rotational
        joint_pos.append(math.radians(motor_degrees[3]))
        joint_pos.append(math.radians(motor_degrees[4]))
        
        return joint_pos
    def check_motor_limits(self, motor_degrees):
            """
            IK 계산 후 나온 모터 각도(deg)가 하드웨어 허용 범위인지 체크
            입력: 5개의 모터 각도 리스트 [deg1, deg2, deg3, deg4, deg5]
            """
            # [Min, Max] (사용자 주석 기반 설정)
            # J1: 범위가 특이하여 확인 필요, 일단 넉넉하게 잡거나 주석대로 설정
            # J5: limitless라고 하셨으므로 -inf, inf
            limits = [
                [-180, 9180],  # J1 (주석이 모호하여 넓게 잡음, 필요시 수정)
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
            [-0.005, 0.255], # J1 m단위
            [-0.3, 3.241],   # J2 (약 -28 ~ 180도) 계산해보기
            [-2.4564, -0.815],    # J3 (-137 ~ 22도)(수정 필요) 계산해보기
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
        """안전을 위해 0 속도 명령 전송"""
        for mid in self.motor_ids:
            try:
                self.drivers[mid].stop_motor()
            except:
                pass

    def shutdown_all_motor(self):
        """모터 셧다운"""
        for mid in self.motor_ids:
            try:
                self.drivers[mid].shutdown_motor()
            except:
                pass

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
        node.stop_all_motor()
        time.sleep(0.1)
        node.stop_all_motor()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()