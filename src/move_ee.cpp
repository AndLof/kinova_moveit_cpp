#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

// lib per le protection zone
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char * argv[])
{
  // ----------------------------
  // 1. ROS2 INIT
  // ----------------------------
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("kinova_move_ee");

  RCLCPP_INFO(node->get_logger(), "Starting MoveIt Cartesian goal example");

  // ----------------------------
  // 2. MOVE GROUP INTERFACE
  // ----------------------------
  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s",
            move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(node->get_logger(), "End effector link: %s",
            move_group.getEndEffectorLink().c_str());

  geometry_msgs::msg::PoseStamped current = move_group.getCurrentPose();

  RCLCPP_INFO(node->get_logger(),
            "Current position: x=%.4f y=%.4f z=%.4f",
            current.pose.position.x,
            current.pose.position.y,
            current.pose.position.z);

  RCLCPP_INFO(node->get_logger(),
            "Current orientation: x=%.4f y=%.4f z=%.4f w=%.4f",
            current.pose.orientation.x,
            current.pose.orientation.y,
            current.pose.orientation.z,
            current.pose.orientation.w);

  // ----------------------------
  // 3. SET START STATE
  // ----------------------------
  move_group.setStartStateToCurrentState();

  // ----------------------------
  // PROTECTION ZONES
  // Entrambe ATTACHED a base_link -> restano solidali al frame e si spostano con esso.
  // (Un semplice CollisionObject con frame_id="base_link" verrebbe invece "congelato"
  //  nel planning frame al momento dell'inserimento e NON seguirebbe base_link.)
  // ----------------------------
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Helper: crea un box e lo attacca a base_link come attached collision object.
  //   dx,dy,dz = dimensioni del box [m]
  //   cx,cy,cz = centro del box espresso in base_link [m]
  // touch_links = {"base_link"} -> il contatto col solo base_link e' consentito,
  // mentre il box viene comunque testato contro i link del braccio.
  auto attachBoxToBase = [&](const std::string & id,
                             double dx, double dy, double dz,
                             double cx, double cy, double cz)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "base_link";
    obj.id = id;

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {dx, dy, dz};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = cx;
    pose.position.y = cy;
    pose.position.z = cz;

    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;

    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = "base_link";
    attached.object = obj;
    attached.touch_links = {"base_link"};
    planning_scene_interface.applyAttachedCollisionObject(attached);
  };

  // Helper: stacca un box attached da base_link (operazione REMOVE).
  // Dopo il detach MoveIt lo re-inserisce come world object con lo stesso id,
  // quindi va poi eliminato con removeCollisionObjects({id}).
  auto detachBoxFromBase = [&](const std::string & id)
  {
    moveit_msgs::msg::AttachedCollisionObject detach;
    detach.link_name = "base_link";
    detach.object.id = id;
    detach.object.operation = detach.object.REMOVE;
    planning_scene_interface.applyAttachedCollisionObject(detach);
  };

  // --- ZONA 1: Spot (identica al nodo NBV) ---
  //   box 1.10 x 0.50 x 0.84 m; top del box a z=0 (come base_link)
  //   -> centro z = 0 - 0.84/2 = -0.42 ; centro (x,y) = (-0.30, 0.00)
  {
    const double SPOT_L = 1.10, SPOT_W = 0.50, SPOT_H = 0.84;
    const double SPOT_CENTER_X = -0.30, SPOT_CENTER_Y = 0.00;
    const double SPOT_TOP_Z    = 0.00;
    const double SPOT_CENTER_Z = SPOT_TOP_Z - SPOT_H / 2.0;   // = -0.42
    attachBoxToBase("spot_body", SPOT_L, SPOT_W, SPOT_H,
                    SPOT_CENTER_X, SPOT_CENTER_Y, SPOT_CENTER_Z);
  }

  // --- ZONA 2: parallelepipedo keep-out (forza una config del braccio "gradita") ---
  //   base inferiore alla stessa quota z di base_link (z=0)
  //   centro base inferiore: x=+0.15, y=0.00
  //   base: 0.05 (x) x 0.10 (y) ; altezza: 0.30 (z)
  //   -> centro box: x=0.15, y=0.00, z = 0 + 0.30/2 = 0.15
  {
    const double BOX_DX = 0.05, BOX_DY = 0.10, BOX_DZ = 0.30;
    const double BOX_CENTER_X = 0.15;                 // = centro base inferiore su x
    const double BOX_CENTER_Y = 0.00;
    const double BOX_CENTER_Z = 0.00 + BOX_DZ / 2.0;  // base a z=0 -> centro a 0.15
    attachBoxToBase("keepout_box", BOX_DX, BOX_DY, BOX_DZ,
                    BOX_CENTER_X, BOX_CENTER_Y, BOX_CENTER_Z);
  }

  // fondamentale: aspetta che MoveIt aggiorni la scena
  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(node->get_logger(),
              "Protection zones (spot_body + keepout_box) aggiunte.");

  // ----------------------------
  // 4. DEFINE GOAL POSE (END EFFECTOR)
  // ----------------------------
  geometry_msgs::msg::Pose target_pose;

  //target_pose.position.x = 0.4;
  //target_pose.position.y = 0.0;
  //target_pose.position.z = 0.3;

  //target_pose.position.x = 0.4;
  //target_pose.position.y = -0.18;
  //target_pose.position.z = 0.88;

  //simulation robot test
  //target_pose.position.x = -0.32;
  //target_pose.position.y = 0.00;
  //target_pose.position.z = 0.8;

  //real robot test
  //target_pose.position.x = -0.1435;
  //target_pose.position.y = -0.6285;
  //target_pose.position.z = 0.4315;

  // Quaternion (orientazione)
  //target_pose.orientation.x = 0.0;
  //target_pose.orientation.y = 1.0;
  //target_pose.orientation.z = 0.0;
  //target_pose.orientation.w = 0.0;

  //target_pose.orientation.x = 1.19;
  //target_pose.orientation.y = -0.4;
  //target_pose.orientation.z = -1.91;
  //target_pose.orientation.w = 0.0;

  //simulation robot test
  //target_pose.orientation.x = 0.868;
  //target_pose.orientation.y = 0.00;
  //target_pose.orientation.z = -0.496;
  //target_pose.orientation.w = 0.0;

  //real robot test
  //target_pose.orientation.x = 0.6996;
  //target_pose.orientation.y = 0.0385;
  //target_pose.orientation.z = 0.0759;
  //target_pose.orientation.w = 0.7093;

  //position for real arm on spot -> home position
  target_pose.position.x = 0.396;
  target_pose.position.y = 0.018;
  target_pose.position.z = 0.333;

  target_pose.orientation.x = 0.488;
  target_pose.orientation.y = 0.535;
  target_pose.orientation.z = 0.497;
  target_pose.orientation.w = 0.478;

  RCLCPP_INFO(node->get_logger(), "Setting pose goal...");

  // ----------------------------
  // 5. SET GOAL TO MOVEIT
  // ----------------------------
  move_group.setPoseTarget(target_pose);

  // ----------------------------
  // 6. PLAN
  // ----------------------------
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success =
    (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // ----------------------------
  // 7. EXECUTE
  // ----------------------------
  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Plan OK -> executing...");
    // execute() e' bloccante: ritorna a traiettoria conclusa.
    auto exec_result = move_group.execute(plan);

    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      // ----------------------------
      // GOAL RAGGIUNTA -> rimuovo le protection zones
      // 1) detach da base_link (torna world object con lo stesso id)
      // 2) removeCollisionObjects -> eliminazione definitiva dal world
      // ----------------------------
      RCLCPP_INFO(node->get_logger(),
                  "Goal raggiunta -> rimuovo le protection zones...");
      detachBoxFromBase("spot_body");
      detachBoxFromBase("keepout_box");
      planning_scene_interface.removeCollisionObjects({"spot_body", "keepout_box"});
      rclcpp::sleep_for(std::chrono::milliseconds(500));   // lascia propagare il diff
      RCLCPP_INFO(node->get_logger(), "Protection zones rimosse.");
    }
    else
    {
      RCLCPP_WARN(node->get_logger(),
                  "Esecuzione non completata: lascio le protection zones attive.");
    }
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed!");
  }

  // ----------------------------
  // 8. SHUTDOWN
  // ----------------------------
  rclcpp::shutdown();
  return 0;
}