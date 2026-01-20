#!/usr/bin/env python3
"""
MoveIt Planning-Only Node (No Execution, No ros2_control needed)
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from moveit.planning import MoveItPy, PlanRequestParameters
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class SimplePlanner(Node):
    def __init__(self):
        super().__init__('simple_planner')
        
        # MoveIt 초기화
        self.moveit = MoveItPy(node_name="moveit_py_planner")
        self.arm = self.moveit.get_planning_component("arm")
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 조인트 상태
        self.joint_state = None
        self.create_subscription(JointState, '/joint_states', 
                                self.joint_state_cb, 10)
        
        # 목표 입력
        self.create_subscription(Point, 'target_input_point', 
                                self.plan_cb, 10)
        
        # 시각화
        self.viz_pub = self.create_publisher(DisplayTrajectory, 
                                            '/display_planned_path', 10)
        
        self.get_logger().info("SimplePlanner ready")

    def joint_state_cb(self, msg):
        self.joint_state = msg

    def plan_cb(self, msg: Point):
        log = self.get_logger()
        log.info("="*60)
        log.info(f"Target: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})")
        
        # 조인트 상태 체크
        if self.joint_state is None:
            log.error("No joint_states")
            return
        
        # 목표 계산
        goal = self.compute_goal(msg)
        if goal is None:
            return
        
        # Planning
        trajectory = self.plan(goal)
        
        if trajectory:
            log.info("✓ SUCCESS")
            self.print_trajectory(trajectory)
            self.visualize(trajectory)  # 시각화 다시 활성화
        else:
            log.error("✗ FAILED")
        
        log.info("="*60)

    def compute_goal(self, point: Point) -> PoseStamped:
        """link8 좌표 → link5 목표"""
        try:
            log = self.get_logger()
            
            # 현재 link8과 link5의 위치 먼저 확인
            try:
                tf_link8_in_base = self.tf_buffer.lookup_transform(
                    "link0", "link8", rclpy.time.Time(),
                    timeout=Duration(seconds=1.0))
                
                tf_link5_in_base = self.tf_buffer.lookup_transform(
                    "link0", "link5", rclpy.time.Time(),
                    timeout=Duration(seconds=1.0))
                
                log.info("Current positions in base frame:")
                log.info(f"  link8: ({tf_link8_in_base.transform.translation.x:.3f}, "
                        f"{tf_link8_in_base.transform.translation.y:.3f}, "
                        f"{tf_link8_in_base.transform.translation.z:.3f})")
                log.info(f"  link5: ({tf_link5_in_base.transform.translation.x:.3f}, "
                        f"{tf_link5_in_base.transform.translation.y:.3f}, "
                        f"{tf_link5_in_base.transform.translation.z:.3f})")
            except:
                pass
            
            # link8 → base
            ps = PointStamped()
            ps.header.frame_id = "link8"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.point = point
            
            tf1 = self.tf_buffer.lookup_transform(
                "link0", "link8", rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            
            target_base = do_transform_point(ps, tf1)
            
            log.info(f"Target in base: ({target_base.point.x:.3f}, "
                    f"{target_base.point.y:.3f}, "
                    f"{target_base.point.z:.3f})")
            
            # link8 → link5 오프셋
            tf2 = self.tf_buffer.lookup_transform(
                "link8", "link5", rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            
            offset = tf2.transform.translation
            log.info(f"link8→link5 offset: ({offset.x:.3f}, {offset.y:.3f}, {offset.z:.3f})")
            
            # link5 목표 (방법 1: 오프셋 빼기)
            goal = PoseStamped()
            goal.header.frame_id = "link0"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = target_base.point.x - offset.x
            goal.pose.position.y = target_base.point.y - offset.y
            goal.pose.position.z = target_base.point.z - offset.z
            goal.pose.orientation.w = 1.0
            
            log.info(f"Goal for link5: ({goal.pose.position.x:.3f}, "
                    f"{goal.pose.position.y:.3f}, "
                    f"{goal.pose.position.z:.3f})")
            
            # 거리 체크
            distance = (goal.pose.position.x**2 + 
                       goal.pose.position.y**2 + 
                       goal.pose.position.z**2)**0.5
            log.info(f"Goal distance from base: {distance:.3f}m")
            
            if distance > 1.0:  # 1m 넘으면 경고
                log.warn(f"Goal seems too far! ({distance:.3f}m)")
            
            return goal
            
        except Exception as e:
            self.get_logger().error(f"Goal computation failed: {e}")
            import traceback
            traceback.print_exc()
            return None

    def plan(self, goal: PoseStamped):
        """Planning 실행"""
        try:
            # 시작 = 현재
            self.arm.set_start_state_to_current_state()
            
            # 목표 설정
            self.arm.set_goal_state(
                pose_stamped_msg=goal,
                pose_link="link5"
            )
            
            # ★ 제약조건 설정 안 함 (기본값 = 제약 없음)
            
            # 파라미터
            params = PlanRequestParameters(self.moveit)
            params.planning_pipeline = "ompl"
            params.planning_time = 5.0
            params.max_velocity_scaling_factor = 0.3
            params.max_acceleration_scaling_factor = 0.3
            
            self.get_logger().info("Planning...")
            
            # Plan
            result = self.arm.plan(parameters=params)
            
            if result and result.trajectory:
                self.get_logger().info("  ✓ Trajectory found")
                return result.trajectory
            else:
                return None
                
        except Exception as e:
            self.get_logger().error(f"Planning error: {e}")
            import traceback
            traceback.print_exc()
            return None

    def visualize(self, trajectory):
        """RViz 시각화"""
        try:
            from moveit_msgs.msg import RobotState
            
            msg = DisplayTrajectory()
            
            # trajectory_start 설정 (현재 로봇 상태)
            msg.trajectory_start.joint_state = self.joint_state
            
            # RobotTrajectory를 메시지로 변환
            if hasattr(trajectory, 'getMessage'):
                traj_msg = trajectory.getMessage()
                msg.trajectory.append(traj_msg)
            else:
                # 이미 메시지 형식인 경우
                msg.trajectory.append(trajectory)
            
            # model_id 설정
            msg.model_id = "arm_v3"
            
            self.viz_pub.publish(msg)
            self.get_logger().info("→ Published to /display_planned_path")
            
        except Exception as e:
            self.get_logger().warn(f"Visualization failed: {e}")
            import traceback
            traceback.print_exc()

    def print_trajectory(self, trajectory):
        """Trajectory 정보 출력"""
        try:
            log = self.get_logger()
            
            # 여러 방법 시도
            try:
                # 방법 1: getMessage()
                if hasattr(trajectory, 'getMessage'):
                    traj_msg = trajectory.getMessage()
                    points = traj_msg.joint_trajectory.points
                    
                    log.info(f"  Waypoints: {len(points)}")
                    
                    if len(points) > 0:
                        duration = points[-1].time_from_start.sec + \
                                  points[-1].time_from_start.nanosec * 1e-9
                        log.info(f"  Duration: {duration:.2f}s")
                        
                        start = points[0].positions
                        end = points[-1].positions
                        
                        log.info(f"  Start joints: {[f'{p:.3f}' for p in start]}")
                        log.info(f"  End joints:   {[f'{p:.3f}' for p in end]}")
                    return
            except:
                pass
            
            try:
                # 방법 2: getWayPointCount()
                if hasattr(trajectory, 'getWayPointCount'):
                    count = trajectory.getWayPointCount()
                    duration = trajectory.getWayPointDurationFromStart(count - 1)
                    log.info(f"  Waypoints: {count}")
                    log.info(f"  Duration: {duration:.2f}s")
                    return
            except:
                pass
            
            # 방법 3: 기본 정보만
            log.info("  Trajectory generated successfully")
            log.info("  (Detailed info not accessible via Python API)")
                
        except Exception as e:
            self.get_logger().warn(f"Failed to print trajectory: {e}")
            import traceback
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = SimplePlanner()
    
    print("\n" + "="*60)
    print("Simple MoveIt Planner")
    print("  Planning: YES")
    print("  Execution: NO")
    print("  Constraints: NO")
    print("="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()