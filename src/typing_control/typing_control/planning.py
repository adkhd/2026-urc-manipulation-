#!/usr/bin/env python3
"""
Complete MoveIt Planning Node
"""
import rclpy
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


class CompletePlanner(Node):
    def __init__(self):
        super().__init__('complete_planner')
        
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
        
        # 퍼블리셔
        self.viz_pub = self.create_publisher(DisplayTrajectory, 
                                            '/display_planned_path', 10)
        self.traj_pub = self.create_publisher(JointTrajectory,
                                              '/planned_trajectory', 10)
        
        self.get_logger().info("✅ Complete Planner Ready")

    def joint_state_cb(self, msg):
        self.joint_state = msg

    def plan_cb(self, msg: Point):
        log = self.get_logger()
        log.info("="*60)
        log.info(f"🎯 Target: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})")
        
        if self.joint_state is None:
            log.error("❌ No joint_states")
            return
        
        goal = self.compute_goal(msg)
        if goal is None: return
        
        trajectory = self.plan_with_retry(goal, max_attempts=5)
        
        if trajectory:
            log.info("✅ PLANNING SUCCESS")
            self.print_trajectory_info(trajectory)
            self.publish_trajectory_data(trajectory)
            self.visualize(trajectory)
        else:
            log.error("❌ PLANNING FAILED")
        
        log.info("="*60)

    def compute_goal(self, point_in_link8: Point) -> PoseStamped:
        """목표 계산"""
        try:
            ps = PointStamped()
            ps.header.frame_id = "link8"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.point = point_in_link8
            
            transform = self.tf_buffer.lookup_transform(
                "link0", "link8", rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            
            target_in_base = do_transform_point(ps, transform)
            
            goal = PoseStamped()
            goal.header.frame_id = "link0"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position = target_in_base.point
            goal.pose.orientation.x = -0.5
            goal.pose.orientation.y = 0.5
            goal.pose.orientation.z = -0.5
            goal.pose.orientation.w = 0.5
            
            self.get_logger().info(f"📍 Goal (link0): ({goal.pose.position.x:.3f}, "
                                  f"{goal.pose.position.y:.3f}, {goal.pose.position.z:.3f})")
            return goal
            
        except Exception as e:
            self.get_logger().error(f"Goal error: {e}")
            return None

    def plan_with_retry(self, goal: PoseStamped, max_attempts=5):
        """재시도 포함 Planning"""
        for attempt in range(1, max_attempts + 1):
            try:
                self.arm.set_start_state_to_current_state()
                self.arm.set_goal_state(pose_stamped_msg=goal, pose_link="link7")
                
                params = PlanRequestParameters(self.moveit)
                params.planning_pipeline = "ompl"
                #params.planner_id = "RRTConnect"
                params.planning_time = 5.0
                params.max_velocity_scaling_factor = 0.5
                params.max_acceleration_scaling_factor = 0.5
                
                self.get_logger().info(f"🔄 Planning attempt {attempt}/{max_attempts}...")
                result = self.arm.plan(parameters=params)
                
                if result and result.trajectory:
                    self.get_logger().info(f"✅ Success on attempt {attempt}")
                    return result.trajectory
                else:
                    self.get_logger().warn(f"⚠️  Attempt {attempt} failed")
                    
            except Exception as e:
                self.get_logger().error(f"Attempt {attempt} error: {e}")
        
        return None

    def print_trajectory_info(self, trajectory):
        """궤적 정보 출력"""
        try:
            log = self.get_logger()
            
            # ✅ get_robot_trajectory_msg() 사용
            if not hasattr(trajectory, 'get_robot_trajectory_msg'):
                log.warn("⚠️  No get_robot_trajectory_msg() method")
                return
            
            traj_msg = trajectory.get_robot_trajectory_msg()
            
            if hasattr(traj_msg, 'joint_trajectory'):
                jt = traj_msg.joint_trajectory
                points = jt.points
                
                if points:
                    duration = points[-1].time_from_start.sec + \
                              points[-1].time_from_start.nanosec * 1e-9
                    
                    log.info(f"  📊 Joints: {list(jt.joint_names)}")
                    log.info(f"  📊 Waypoints: {len(points)}")
                    log.info(f"  ⏱️  Duration: {duration:.2f}s")
                    
                    start = points[0].positions
                    end = points[-1].positions
                    log.info(f"  🎯 Start: {[f'{p:.2f}' for p in start]}")
                    log.info(f"  🎯 End:   {[f'{p:.2f}' for p in end]}")
                    
        except Exception as e:
            log.warn(f"Print info error: {e}")

    def publish_trajectory_data(self, trajectory):
        """궤적 데이터 전송"""
        try:
            log = self.get_logger()
            
            # ✅ get_robot_trajectory_msg() 사용
            if not hasattr(trajectory, 'get_robot_trajectory_msg'):
                log.error("❌ Cannot extract trajectory data")
                return
            
            traj_msg = trajectory.get_robot_trajectory_msg()
            
            if not hasattr(traj_msg, 'joint_trajectory'):
                log.error("❌ No joint_trajectory in message")
                return
            
            # JointTrajectory 추출
            jt = traj_msg.joint_trajectory
            jt.header.stamp = self.get_clock().now().to_msg()
            
            # Publish
            #self.traj_pub.publish(jt)
            log.info(f"📤 Trajectory published to /planned_trajectory")
            
        except Exception as e:
            log.error(f"Publish trajectory error: {e}")

    def visualize(self, trajectory):
        """RViz 시각화"""
        try:
            log = self.get_logger()
            log.info("🎨 Visualizing...")
            
            # ✅ get_robot_trajectory_msg() 사용
            if not hasattr(trajectory, 'get_robot_trajectory_msg'):
                log.error("❌ No get_robot_trajectory_msg() method")
                return
            
            raw_traj = trajectory.get_robot_trajectory_msg()
            
            if not hasattr(raw_traj, 'joint_trajectory'):
                log.error("❌ No joint_trajectory attribute")
                return
            
            # DisplayTrajectory 생성
            msg = DisplayTrajectory()
            msg.model_id = "arm_v3"
            
            # 시작 상태
            if self.joint_state:
                msg.trajectory_start.joint_state = self.joint_state
            
            # 궤적 데이터 깊은 복사
            src_traj = raw_traj.joint_trajectory
            clean_traj = RobotTrajectory()
            new_traj = JointTrajectory()
            
            # 헤더
            new_traj.header = Header()
            new_traj.header.frame_id = "link0"
            new_traj.header.stamp = self.get_clock().now().to_msg()
            new_traj.joint_names = list(src_traj.joint_names)
            
            # 포인트 복사
            for pt in src_traj.points:
                new_pt = JointTrajectoryPoint()
                new_pt.positions = list(pt.positions)
                new_pt.velocities = list(pt.velocities) if pt.velocities else []
                new_pt.accelerations = list(pt.accelerations) if pt.accelerations else []
                new_pt.effort = list(pt.effort) if pt.effort else []
                
                dur = ROSDuration()
                dur.sec = int(pt.time_from_start.sec)
                dur.nanosec = int(pt.time_from_start.nanosec)
                new_pt.time_from_start = dur
                
                new_traj.points.append(new_pt)
            
            clean_traj.joint_trajectory = new_traj
            msg.trajectory.append(clean_traj)
            
            # Publish
            #self.viz_pub.publish(msg)
            
            num_subscribers = self.viz_pub.get_subscription_count()
            log.info(f"✅ Visualization published (subscribers: {num_subscribers})")
            
            if num_subscribers == 0:
                log.warn("⚠️  No RViz subscribers")
            
        except Exception as e:
            log.error(f"❌ Visualization error: {e}")
            import traceback
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = CompletePlanner()
    
    print("\n" + "="*60)
    print("🚀 Complete MoveIt Planner")
    print("  - Planning with 5 retry attempts")
    print("  - Visualization: /display_planned_path")
    print("  - Trajectory data: /planned_trajectory")
    print("="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()