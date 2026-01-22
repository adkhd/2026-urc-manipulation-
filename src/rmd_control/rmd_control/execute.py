"""
Trajectory Execute Node
- Receives planned trajectory from /trajectory_for_execute
- Executes with velocity control + position PID
- No CAN initialization (uses existing rmd_node drivers)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Bool, String
import time
import math
import math
import threading
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rmd_control.kinematics_vel import KinematicsSolver
from rmd_control.kinematics_angle import calculate_kinematics as fk, calculate_ik as ik
from rmd_control.rmd_driver_4_3 import RMDMotorDriver as rmd43
from rmd_control.rmd_driver_4_01 import RMDMotorDriver as rmd401


class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')
        
        # ========================
        # 1. 기본 설정
        # ========================
        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5']
        self.motor_ids = [1, 2, 3, 4, 5]
        self.num_joints = 5
        self.control_Hz = 150.0
        self.solver = KinematicsSolver()
        
        # 속도 제한 (Joint 레벨)
        self.vel_limits = [0.1, 1.0, 1.0, 1.0, 2.0]  # [m/s, rad/s, rad/s, rad/s, rad/s]
        
        # PID 게인 (관절별 위치 오차 보정용)
        # [J1, J2, J3, J4, J5]
        self.position_kp = [50.0, 1.5, 1.5, 1.0, 0.8]  # P gain
        self.position_ki = [0.0, 0.0, 0.0, 0.0, 0.0]   # I gain (optional)
        self.position_kd = [0.0, 0.0, 0.0, 0.0, 0.0]   # D gain (optional)
        
        # 적분 오차 및 이전 오차
        self.integral_error = [0.0] * self.num_joints
        self.prev_error = [0.0] * self.num_joints
        
        # 모터 드라이버 (외부에서 공유받을 예정)
        self.drivers = {}
        
        # 현재 상태
        self.current_joint_pos = [0.0] * self.num_joints
        self.is_executing = False
        self.emergency_stop = False
        
        # Trajectory 데이터
        self.trajectory = None
        self.traj_lock = threading.Lock()
        
        # ========================
        # 2. ROS 통신
        # ========================
        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # Trajectory 수신 (Planning 노드로부터)
        self.sub_trajectory = self.create_subscription(
            JointTrajectory,
            '/trajectory_for_execute',
            self.trajectory_callback,
            qos
        )
        
        # Joint States 수신 (rmd_node로부터)
        self.sub_joint_state = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos
        )
        
        # 완료 상태 발행
        self.pub_status = self.create_publisher(String, '/execute_status', 10)
        self.pub_complete = self.create_publisher(Bool, '/execute_complete', 10)
        
        # ========================
        # 3. 모터 드라이버 연결 (기존 CAN 사용)
        # ========================
        self.connect_to_motors()
        
        self.get_logger().info("✅ Trajectory Executor Ready")
        self.get_logger().info("   Waiting for trajectory on /trajectory_for_execute")

    def connect_to_motors(self):
        """기존 CAN 인터페이스에 연결 (초기화 X)"""
        try:
            for mid in self.motor_ids:
                if mid in [1, 4]:
                    driver = rmd401(channel='can0', motor_id=mid)
                else:
                    driver = rmd43(channel='can0', motor_id=mid)
                self.drivers[mid] = driver
                
            self.get_logger().info("✅ Connected to existing CAN motors")
        except Exception as e:
            self.get_logger().error(f"❌ Motor connection failed: {e}")
            import traceback
            traceback.print_exc()

    def joint_state_callback(self, msg: JointState):
        """현재 관절 위치 업데이트"""
        if len(msg.position) >= self.num_joints:
            self.current_joint_pos = list(msg.position[:self.num_joints])

    def trajectory_callback(self, msg: JointTrajectory):
        """Trajectory 수신 및 실행 시작"""
        if self.is_executing:
            self.get_logger().warn("⚠️  Already executing. Ignoring new trajectory.")
            return
        
        with self.traj_lock:
            self.trajectory = msg
        
        self.get_logger().info("="*60)
        self.get_logger().info(f"📥 Trajectory Received")
        self.get_logger().info(f"   Waypoints: {len(msg.points)}")
        
        if msg.points:
            duration = msg.points[-1].time_from_start.sec + \
                      msg.points[-1].time_from_start.nanosec * 1e-9
            self.get_logger().info(f"   Duration: {duration:.2f}s")
        
        # 실행 시작
        exec_thread = threading.Thread(target=self.execute_trajectory, daemon=True)
        exec_thread.start()

    def execute_trajectory(self):
        """Trajectory 실행 메인 루프"""
        with self.traj_lock:
            traj = self.trajectory
        
        # ========================
        # 궤적 유효성 검증
        # ========================
        if not self.validate_trajectory(traj):
            self.publish_status("FAILED_INVALID_TRAJECTORY")
            return
        
        self.is_executing = True
        self.emergency_stop = False
        self.publish_status("EXECUTING")
        
        try:
            # ========================
            # 0. 시작점 검증
            # ========================
            if not self.validate_start_position(traj.points[0].positions):
                self.get_logger().error("❌ Start position mismatch. Aborting.")
                self.publish_status("FAILED_START_MISMATCH")
                return
            
            # ========================
            # 1. Trajectory 파라미터 추출
            # ========================
            waypoints = traj.points
            total_time = waypoints[-1].time_from_start.sec + \
                        waypoints[-1].time_from_start.nanosec * 1e-9
            
            final_target_pos = list(waypoints[-1].positions[:self.num_joints])
            
            self.get_logger().info(f"🚀 Execution Start")
            self.get_logger().info(f"   Total Time: {total_time:.2f}s")
            self.get_logger().info(f"   Control Rate: {self.control_Hz} Hz")
            self.get_logger().info(f"   Target: {[f'{p:.3f}' for p in final_target_pos]}")
            
            # ========================
            # 2. PID 초기화
            # ========================
            self.reset_pid()
            
            # ========================
            # 3. 메인 실행 루프 (궤적 추종)
            # ========================
            start_time = time.time()
            control_period = 1.0 / self.control_Hz
            
            while not self.emergency_stop:
                loop_start = time.time()
                elapsed = loop_start - start_time
                
                # 시간 초과 체크
                if elapsed > total_time:
                    self.get_logger().info("⏱️  Trajectory time completed")
                    break
                
                # 현재 시간에 해당하는 목표 위치/속도 보간
                target_pos, target_vel = self.interpolate_trajectory(waypoints, elapsed)
                
                if target_pos is None:
                    self.get_logger().error("❌ Interpolation failed")
                    break
                
                # 위치 오차 계산 및 PID 제어
                position_error = [target_pos[i] - self.current_joint_pos[i] 
                                 for i in range(self.num_joints)]
                
                pid_correction = self.compute_pid(position_error, control_period)
                
                # 최종 속도 명령 = Feedforward + PID
                final_velocities = [target_vel[i] + pid_correction[i] 
                                   for i in range(self.num_joints)]
                
                # 모터 속도 명령 전송
                self.send_velocity_command(final_velocities, target_pos)
                
                # 주기 맞추기
                elapsed_loop = time.time() - loop_start
                sleep_time = control_period - elapsed_loop
                if sleep_time > 0:
                    time.sleep(sleep_time)
            
            # ========================
            # 4. 긴급 정지 체크
            # ========================
            if self.emergency_stop:
                self.get_logger().warn("⛔ Emergency Stop!")
                self.stop_all_motors()
                self.publish_status("STOPPED")
                return
            
            # ========================
            # 5. 최종 위치 도달 (PID 제어)
            # ========================
            self.get_logger().info("🎯 Final positioning with PID...")
            
            if not self.reach_final_position(final_target_pos):
                self.get_logger().error("❌ Failed to reach final position")
                self.publish_status("FAILED_POSITIONING")
                return
            
            # ========================
            # 6. 성공 완료
            # ========================
            self.get_logger().info("✅ Trajectory Execution Complete")
            self.publish_status("COMPLETED")
            
            # 완료 신호
            complete_msg = Bool()
            complete_msg.data = True
            self.pub_complete.publish(complete_msg)
        
        except Exception as e:
            self.get_logger().error(f"❌ Execution error: {e}")
            import traceback
            traceback.print_exc()
            self.stop_all_motors()
            self.publish_status("ERROR")
        
        finally:
            self.is_executing = False
            self.get_logger().info("="*60)

    def validate_start_position(self, start_pos):
        """시작 위치가 현재 위치와 일치하는지 확인"""
        # 관절별 맞춤 임계값
        # [J1(m), J2(rad), J3(rad), J4(rad), J5(rad)]
        thresholds = [
            0.005,  # J1: 5mm (리니어 관절은 더 엄격)
            0.05,   # J2: 2.86° (회전 관절)
            0.05,   # J3: 2.86°
            0.05,   # J4: 2.86°
            0.05    # J5: 2.86°
        ]
        
        for i in range(self.num_joints):
            error = abs(start_pos[i] - self.current_joint_pos[i])
            threshold = thresholds[i]
            
            if error > threshold:
                # 단위 표시
                unit = "m" if i == 0 else "rad"
                unit_mm_deg = f"{error*1000:.1f}mm" if i == 0 else f"{math.degrees(error):.2f}°"
                
                self.get_logger().error(
                    f"Joint {i+1}: Start={start_pos[i]:.4f}{unit}, "
                    f"Current={self.current_joint_pos[i]:.4f}{unit}, "
                    f"Error={error:.4f}{unit} ({unit_mm_deg}), "
                    f"Threshold={threshold:.4f}{unit}"
                )
                return False
        
        self.get_logger().info("✅ Start position validated")
        return True

    def validate_trajectory(self, traj):
        """궤적 유효성 종합 검증"""
        log = self.get_logger()
        
        # ========================
        # 1. 기본 존재 여부
        # ========================
        if traj is None:
            log.error("❌ Trajectory is None")
            return False
        
        if not traj.points:
            log.error("❌ Trajectory has no points")
            return False
        
        # ========================
        # 2. Waypoint 개수 검증
        # ========================
        MIN_WAYPOINTS = 2  # 최소 시작점 + 종료점
        if len(traj.points) < MIN_WAYPOINTS:
            log.error(f"❌ Too few waypoints: {len(traj.points)} (min: {MIN_WAYPOINTS})")
            log.error("   Planning 결과가 비정상적으로 짧습니다.")
            return False
        
        # ========================
        # 3. 실행 시간 검증
        # ========================
        MIN_DURATION = 0.1  # 최소 0.1초
        MAX_DURATION = 60.0  # 최대 60초
        
        duration = (traj.points[-1].time_from_start.sec + 
                   traj.points[-1].time_from_start.nanosec * 1e-9)
        
        if duration < MIN_DURATION:
            log.error(f"❌ Duration too short: {duration:.3f}s (min: {MIN_DURATION}s)")
            log.error("   궤적이 너무 짧아 안전하게 실행할 수 없습니다.")
            return False
        
        if duration > MAX_DURATION:
            log.warn(f"⚠️  Duration very long: {duration:.1f}s (max recommended: {MAX_DURATION}s)")
            log.warn("   실행 시간이 매우 깁니다. 계속 진행합니다.")
        
        # ========================
        # 4. 시간 순서 검증
        # ========================
        prev_time = 0.0
        for i, point in enumerate(traj.points):
            curr_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            
            if curr_time < prev_time:
                log.error(f"❌ Time not monotonic at point {i}: "
                         f"{prev_time:.3f}s → {curr_time:.3f}s")
                return False
            
            if i > 0 and curr_time == prev_time:
                log.warn(f"⚠️  Duplicate timestamp at point {i}: {curr_time:.3f}s")
            
            prev_time = curr_time
        
        # ========================
        # 5. 관절 개수 검증
        # ========================
        if traj.joint_names and len(traj.joint_names) != self.num_joints:
            log.error(f"❌ Joint count mismatch: "
                     f"Expected {self.num_joints}, Got {len(traj.joint_names)}")
            return False
        
        for i, point in enumerate(traj.points):
            if len(point.positions) < self.num_joints:
                log.error(f"❌ Point {i} has {len(point.positions)} positions "
                         f"(need {self.num_joints})")
                return False
        
        # ========================
        # 6. 데이터 무결성 검증
        # ========================
        for i, point in enumerate(traj.points):
            # NaN/Inf 체크
            for j, pos in enumerate(point.positions[:self.num_joints]):
                if not math.isfinite(pos):
                    log.error(f"❌ Invalid position at point {i}, joint {j}: {pos}")
                    return False
            
            # 속도 데이터가 있다면 검증
            if point.velocities:
                for j, vel in enumerate(point.velocities[:self.num_joints]):
                    if not math.isfinite(vel):
                        log.error(f"❌ Invalid velocity at point {i}, joint {j}: {vel}")
                        return False
        
        # ========================
        # ✅ 모든 검증 통과
        # ========================
        log.info("✅ Trajectory validation passed")
        log.info(f"   Waypoints: {len(traj.points)}")
        log.info(f"   Duration: {duration:.2f}s")
        
        return True

    def interpolate_trajectory(self, waypoints, current_time):
        """현재 시간에 해당하는 목표 위치/속도 선형 보간"""
        # 현재 시간이 속한 구간 찾기
        for i in range(len(waypoints) - 1):
            t0 = waypoints[i].time_from_start.sec + \
                 waypoints[i].time_from_start.nanosec * 1e-9
            t1 = waypoints[i+1].time_from_start.sec + \
                 waypoints[i+1].time_from_start.nanosec * 1e-9
            
            if t0 <= current_time <= t1:
                # 선형 보간
                alpha = (current_time - t0) / (t1 - t0) if (t1 - t0) > 0 else 0.0
                
                pos0 = waypoints[i].positions[:self.num_joints]
                pos1 = waypoints[i+1].positions[:self.num_joints]
                target_pos = [pos0[j] + alpha * (pos1[j] - pos0[j]) 
                             for j in range(self.num_joints)]
                
                # 속도 보간 (있는 경우)
                if waypoints[i].velocities and waypoints[i+1].velocities:
                    vel0 = waypoints[i].velocities[:self.num_joints]
                    vel1 = waypoints[i+1].velocities[:self.num_joints]
                    target_vel = [vel0[j] + alpha * (vel1[j] - vel0[j]) 
                                 for j in range(self.num_joints)]
                else:
                    # 속도 정보 없으면 차분으로 계산
                    dt = t1 - t0
                    target_vel = [(pos1[j] - pos0[j]) / dt if dt > 0 else 0.0
                                 for j in range(self.num_joints)]
                
                return target_pos, target_vel
        
        # 마지막 waypoint
        if current_time >= waypoints[-1].time_from_start.sec + \
                          waypoints[-1].time_from_start.nanosec * 1e-9:
            return list(waypoints[-1].positions[:self.num_joints]), \
                   [0.0] * self.num_joints
        
        return None, None

    def compute_pid(self, error, dt):
        """PID 제어 계산"""
        correction = []
        
        for i in range(self.num_joints):
            # P term
            p_term = self.position_kp[i] * error[i]
            
            # I term
            self.integral_error[i] += error[i] * dt
            i_term = self.position_ki[i] * self.integral_error[i]
            
            # D term
            d_term = self.position_kd[i] * (error[i] - self.prev_error[i]) / dt if dt > 0 else 0.0
            self.prev_error[i] = error[i]
            
            # 총 보정량
            correction.append(p_term + i_term + d_term)
        
        return correction

    def reset_pid(self):
        """PID 상태 초기화"""
        self.integral_error = [0.0] * self.num_joints
        self.prev_error = [0.0] * self.num_joints

    def send_velocity_command(self, joint_vels, target_pos):
        """
        관절 속도 -> 모터 속도 변환 및 전송 (속도 제한 포함)
        
        Args:
            joint_vels: 관절 속도 리스트 [J1(m/s), J2(rad/s), J3, J4, J5]
            target_pos: 목표 관절 위치 (J3 커플링 계산용)
        """
        for i, vel in enumerate(joint_vels):
            mid = self.motor_ids[i]
            motor_dps = 0.0
            
            # ========================
            # 1. 관절 속도 제한 (Clamping)
            # ========================
            vel_limit = self.vel_limits[i]
            vel_clamped = max(-vel_limit, min(vel_limit, vel))
            
            # 클리핑 발생 시 경고
            if abs(vel) > vel_limit:
                unit = "m/s" if i == 0 else "rad/s"
                self.get_logger().warn(
                    f"⚠️  J{i+1} velocity clamped: {vel:.3f}{unit} → {vel_clamped:.3f}{unit}"
                )
            
            # ========================
            # 2. 관절 속도 -> 모터 속도 변환
            # ========================
            if mid == 1:
                # J1 Linear: m/s -> dps
                motor_dps = vel_clamped * 36000.0
                
                # 모터 RPM 제한 (이중 안전장치)
                max_dps = 180.0 * 6.0  # 180 RPM = 1080 deg/s
                motor_dps = max(-max_dps, min(max_dps, motor_dps))
            
            elif mid == 2:
                # J2: rad/s -> dps
                motor_dps = math.degrees(vel_clamped)
                
                # 모터 속도 제한 (deg/s)
                max_dps = math.degrees(self.vel_limits[i])  # 1.0 rad/s = 57.3 deg/s
                motor_dps = max(-max_dps, min(max_dps, motor_dps))
            
            elif mid == 3:
                # J3 Coupling: M3 = M2 + J3
                j2_vel = joint_vels[1]
                j3_vel = vel_clamped
                
                # 커플링 계산
                coupled_vel = j2_vel + self.solver.calculate_safe_velocity(
                    ik(target_pos[2]), j3_vel
                )
                
                motor_dps = math.degrees(coupled_vel) * -1.0
                
                # 모터 속도 제한
                max_dps = math.degrees(self.vel_limits[i])  # 1.0 rad/s = 57.3 deg/s
                motor_dps = max(-max_dps, min(max_dps, motor_dps))
            
            elif mid == 4:
                # J4: rad/s -> dps
                motor_dps = math.degrees(vel_clamped)
                
                # 모터 속도 제한
                max_dps = math.degrees(self.vel_limits[i])  # 1.0 rad/s = 57.3 deg/s
                motor_dps = max(-max_dps, min(max_dps, motor_dps))
            
            elif mid == 5:
                # J5: rad/s -> dps
                motor_dps = math.degrees(vel_clamped)
                
                # 모터 속도 제한
                max_dps = math.degrees(self.vel_limits[i])  # 2.0 rad/s = 114.6 deg/s
                motor_dps = max(-max_dps, min(max_dps, motor_dps))
            
            # ========================
            # 3. 모터 명령 전송
            # ========================
            try:
                self.drivers[mid].vel_control(speed_dps=motor_dps)
            except Exception as e:
                self.get_logger().error(f"Motor {mid} vel_control failed: {e}")

    def reach_final_position(self, target_pos):
        """
        최종 목표 위치에 도달할 때까지 PID 제어
        
        Args:
            target_pos: 목표 관절 위치 리스트 [J1, J2, J3, J4, J5]
        
        Returns:
            bool: 성공 여부
        """
        log = self.get_logger()
        
        # 허용 오차 설정
        position_thresholds = [
            0.002,  # J1: 2mm
            0.01,   # J2: 1.15°
            0.01,   # J3: 1.15°
            0.01,   # J4: 1.15°
            0.01    # J5: 1.72° (그리퍼 회전은 덜 중요)
        ]
        
        # 타임아웃 설정
        TIMEOUT_SEC = 2.0
        CHECK_INTERVAL = 0.05  # 50ms마다 체크
        
        # PID 재초기화
        self.reset_pid()
        
        log.info(f"   Target: {[f'{p:.3f}' for p in target_pos]}")
        
        start_time = time.time()
        control_period = 1.0 / self.control_Hz
        
        while not self.emergency_stop:
            loop_start = time.time()
            elapsed = loop_start - start_time
            
            # 타임아웃 체크
            if elapsed > TIMEOUT_SEC:
                log.error(f"❌ Timeout after {TIMEOUT_SEC}s")
                
                # 최종 오차 출력
                for i in range(self.num_joints):
                    error = abs(target_pos[i] - self.current_joint_pos[i])
                    unit = "m" if i == 0 else "rad"
                    unit_display = f"{error*1000:.1f}mm" if i == 0 else f"{math.degrees(error):.2f}°"
                    
                    log.error(
                        f"   J{i+1}: Target={target_pos[i]:.4f}{unit}, "
                        f"Current={self.current_joint_pos[i]:.4f}{unit}, "
                        f"Error={unit_display}"
                    )
                
                self.stop_all_motors()
                return False
            
            # 현재 위치 오차 계산
            position_error = [target_pos[i] - self.current_joint_pos[i] 
                             for i in range(self.num_joints)]
            
            # 모든 관절이 허용 오차 내에 있는지 체크
            all_reached = True
            max_error_info = {"joint": 0, "error": 0.0}
            
            for i in range(self.num_joints):
                error = abs(position_error[i])
                
                if error > position_thresholds[i]:
                    all_reached = False
                
                if error > max_error_info["error"]:
                    max_error_info = {"joint": i+1, "error": error}
            
            # 도달 성공
            if all_reached:
                log.info(f"✅ Final position reached in {elapsed:.2f}s")
                
                # 최종 위치 출력
                for i in range(self.num_joints):
                    error = abs(position_error[i])
                    unit_display = f"{error*1000:.2f}mm" if i == 0 else f"{math.degrees(error):.3f}°"
                    log.info(f"   J{i+1}: {unit_display} error")
                
                # 속도 0으로 정지
                self.send_velocity_command([0.0]*self.num_joints, target_pos)
                time.sleep(0.1)
                self.stop_all_motors()
                
                return True
            
            # PID 제어 계산
            pid_correction = self.compute_pid(position_error, control_period)
            
            # 속도 명령 = PID만 (Feedforward 없음)
            final_velocities = pid_correction
            
            # 모터 제어
            self.send_velocity_command(final_velocities, target_pos)
            
            # 주기적 로그 출력 (0.2초마다)
            if int(elapsed / CHECK_INTERVAL) != int((elapsed - control_period) / CHECK_INTERVAL):
                unit_display = (f"{max_error_info['error']*1000:.1f}mm" 
                               if max_error_info['joint'] == 1 
                               else f"{math.degrees(max_error_info['error']):.2f}°")
                
                log.info(
                    f"   [{elapsed:.1f}s] Max error: J{max_error_info['joint']} "
                    f"{unit_display}"
                )
            
            # 주기 맞추기
            elapsed_loop = time.time() - loop_start
            sleep_time = control_period - elapsed_loop
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # 긴급 정지
        log.warn("⛔ Emergency stop during final positioning")
        self.stop_all_motors()
        return False

    def stop_all_motors(self):
        """모든 모터 정지"""
        for mid in self.motor_ids:
            try:
                self.drivers[mid].stop_motor()
            except:
                pass

    def publish_status(self, status: str):
        """상태 발행"""
        msg = String()
        msg.data = status
        self.pub_status.publish(msg)
        self.get_logger().info(f"📢 Status: {status}")


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*60)
    print("🎮 Trajectory Executor Node")
    print("  - Subscribe: /trajectory_for_execute")
    print("  - Subscribe: /joint_states")
    print("  - Publish: /execute_status, /execute_complete")
    print("  - Control Rate: 150 Hz")
    print("="*60 + "\n")
    
    node = TrajectoryExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down executor")
    finally:
        node.stop_all_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()