#include <rclcpp/rclcpp.hpp>  //당연히 있어야함.
#include <geometry_msgs/msg/pose.hpp> //pose 메세지

// MoveIt
#include <moveit_msgs/msg/planning_scene.hpp> 
#include <moveit_msgs/msg/collision_object.hpp>

#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometric_shapes/shape_operations.h>

int main(int argc,char **argv){

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("spawn_node", node_options);
        
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
        node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
      while (planning_scene_diff_publisher->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "my_custom_mesh";
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    std::string mesh_file_path = "package://spawn/meshes/gripper_case.STL";
    shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_file_path,Eigen::Vector3d(0.001, 0.001, 0.001));
    
    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg shape_msg;

    shapes::constructMsgFromShape(mesh, shape_msg);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.position.x = 1.0;
    mesh_pose.position.y = 2.0;
    mesh_pose.position.z = -3.0;
    mesh_pose.orientation.w = 1.0;

    collision_object.meshes.push_back(mesh_msg);
    collision_object.mesh_poses.push_back(mesh_pose);
            
    moveit_msgs::msg::PlanningScene planning_scene_msg;
        // world에 collision_object를 추가하라고 지정합니다.
    planning_scene_msg.is_diff = true;
    planning_scene_msg.world.collision_objects.push_back(collision_object);

      // 준비된 메시지를 발행합니다.
    planning_scene_diff_publisher->publish(planning_scene_msg);
    rclcpp::shutdown();
    return 0;

    }