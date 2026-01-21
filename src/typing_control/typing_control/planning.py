#!/usr/bin/env python3
"""
MoveIt Planning-Only Node (Complete Version)
"""
import rclpy
import time
from rclpy.node import Node
from rclpy.duration import Duration
from moveit.planning import MoveItPy, PlanRequestParameters
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration as ROSDuration
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('simple_planner')
        
        # 1. MoveIt 초기화
        self.moveit = MoveItPy(node_name="moveit_py_planner")
        self.arm = self.moveit.get_planning_component("arm")
        
        # 2. TF 초기화
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 3. 데이터 구독
        self.joint_state = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(Point, 'target_input_point', self.plan_cb, 10)
        
        # 4. 시각화 퍼블리셔
        self.viz_pub = self.create_publisher(DisplayTrajectory, '/display_planned_path', 10)
        
        self.get_logger().info("SimplePlanner Ready (Complete Version)")

    def joint_state_cb(self, msg):
        self.joint_state = msg

    def plan_cb(self, msg: Point):
        log = self.get_logger()
        log.info("="*60)
        log.info(f"Target Input: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})")
        
        if self.joint_state is None:
            log.error("Wait for joint_states...")
            return
        
        # 1. 목표 계산
        goal = self.compute_goal(msg)
        if goal is None: return
        
        # 2. 플래닝 실행
        trajectory = self.plan(goal)
        
        # 3. 결과 처리
        if trajectory:
            log.info("✓ PLANNING SUCCESS")
            self.print_trajectory_info(trajectory)
            self.visualize(trajectory)
        else:
            log.error("✗ PLANNING FAILED")
        
        log.info("="*60)

    def compute_goal(self, point_in_link8: Point) -> PoseStamped:
        """link8 좌표 → link0 변환 및 목표 자세 설정"""
        try:
            # 변환할 포인트 설정
            ps = PointStamped()
            ps.header.frame_id = "link8"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.point = point_in_link8
            
            # TF 조회 (link0 <-> link8)
            transform = self.tf_buffer.lookup_transform(
                "link0", "link8", rclpy.time.Time(), timeout=Duration(seconds=1.0))
            
            # 좌표 변환 실행
            target_in_base = do_transform_point(ps, transform)
            
            # 최종 목표 Pose 생성
            goal = PoseStamped()
            goal.header.frame_id = "link0"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position = target_in_base.point
            
            # 목표 자세: RPY(-90, 90, 0)에 해당하는 쿼터니언
            goal.pose.orientation.x = -0.5
            goal.pose.orientation.y = 0.5
            goal.pose.orientation.z = -0.5
            goal.pose.orientation.w = 0.5
            
            self.get_logger().info(f"Goal Pose (link0): {goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}, {goal.pose.position.z:.3f}")
            return goal
            
        except Exception as e:
            self.get_logger().error(f"Compute Goal Error: {e}")
            return None
    def plan(self, goal: PoseStamped):
        """Planning 수행"""
        try:
            self.arm.set_start_state_to_current_state()
            
            # [삭제] 아래 두 줄은 moveit_py에서 지원하지 않으므로 삭제합니다.
            # self.arm.set_goal_position_tolerance(0.001)
            # self.arm.set_goal_orientation_tolerance(0.01)
            
            # 목표 설정
            self.arm.set_goal_state(pose_stamped_msg=goal, pose_link="link7")
            
            # 플래너 파라미터
            params = PlanRequestParameters(self.moveit)
            params.planning_pipeline = "ompl"
            params.planner_id = "RRTConnect"
            params.planning_time = 5.0
            params.max_velocity_scaling_factor = 0.5
            params.max_acceleration_scaling_factor = 0.5
            
            # 플래닝
            self.get_logger().info("Planning started...")
            result = self.arm.plan(parameters=params)
            
            if result and result.trajectory:
                return result.trajectory
            return None
                
        except Exception as e:
            self.get_logger().error(f"Planning Logic Error: {e}")
            import traceback
            traceback.print_exc()
            return None
    def visualize(self, trajectory):
        """
        [핵심] 궤적 데이터를 RViz로 안전하게 전송
        C++ 객체와 Python 객체 간의 충돌을 방지하기 위해 데이터를 '깊은 복사'합니다.
        """
        try:
            log = self.get_logger()
            msg = DisplayTrajectory()
            msg.model_id = "arm_v3"
            
            # 시작 상태 설정
            if self.joint_state:
                msg.trajectory_start.joint_state = self.joint_state
            
            # 궤적 데이터 추출 (버전에 따라 getMessage() 유무가 다름)
            if hasattr(trajectory, 'getMessage'):
                raw_traj = trajectory.getMessage()
            else:
                # getMessage가 없으면 속성으로 접근 시도
                raw_traj = trajectory

            # RobotTrajectory 빈 객체 생성
            clean_traj = RobotTrajectory()
            
            # JointTrajectory 데이터가 있는지 확인
            if hasattr(raw_traj, 'joint_trajectory'):
                src_traj = raw_traj.joint_trajectory
                new_traj = JointTrajectory()
                
                # 헤더 복사
                new_traj.header = Header()
                new_traj.header.frame_id = src_traj.header.frame_id if src_traj.header.frame_id else "link0"
                new_traj.header.stamp = self.get_clock().now().to_msg()
                new_traj.joint_names = src_traj.joint_names
                
                # 포인트 데이터 한땀한땀 복사 (이 부분이 충돌 방지의 핵심)
                for pt in src_traj.points:
                    new_pt = JointTrajectoryPoint()
                    new_pt.positions = list(pt.positions)
                    new_pt.velocities = list(pt.velocities)
                    new_pt.accelerations = list(pt.accelerations)
                    new_pt.effort = list(pt.effort)
                    
                    # 시간 정보 복사
                    dur = ROSDuration()
                    dur.sec = int(pt.time_from_start.sec)
                    dur.nanosec = int(pt.time_from_start.nanosec)
                    new_pt.time_from_start = dur
                    
                    new_traj.points.append(new_pt)
                
                clean_traj.joint_trajectory = new_traj
                msg.trajectory.append(clean_traj)
                
                self.viz_pub.publish(msg)
                log.info("→ Visuzliation Published (Deep Copy Complete)")
            else:
                log.warn("Trajectory format not recognized for visualization.")

        except Exception as e:
            self.get_logger().error(f"Visualization Error: {e}")
            import traceback
            traceback.print_exc()

    def print_trajectory_info(self, trajectory):
        """궤적 정보 로그 출력"""
        try:
            # 데이터 추출
            if hasattr(trajectory, 'getMessage'):
                traj_msg = trajectory.getMessage()
            else:
                traj_msg = trajectory

            if hasattr(traj_msg, 'joint_trajectory'):
                points = traj_msg.joint_trajectory.points
                cnt = len(points)
                if cnt > 0:
                    last_pt = points[-1]
                    duration = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9
                    self.get_logger().info(f"  - Waypoints: {cnt}")
                    self.get_logger().info(f"  - Duration: {duration:.2f}s")
        except:
            pass # 로그 출력 실패는 무시

def main(args=None):
    rclpy.init(args=args)
    node = SimplePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()