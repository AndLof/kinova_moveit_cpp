// =============================================================================
// kinova_explorer_nbv_validation.cpp   !!!!VALIDAZIONE --> tabella + selezione manuale della posa da eseguire (per esperimenti YOLO)!!!!


#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <optional>
#include <future>
#include <cstdio>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

#include <Eigen/Geometry>

using namespace std::chrono_literals;
using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

// ----------------------------- PARAMETRI -------------------------------------
static const std::string PLANNING_GROUP = "manipulator";
static const std::string CLOUD_TOPIC    = "/spot/pointcloud/merged";
static const std::string ARM_BASE_FRAME = "base_link";
static const std::string CAMERA_FRAME   = "camera_link";   // asse ottico = +z

//Spot come box --> protection zone
static const double SPOT_L = 1.10, SPOT_W = 0.50, SPOT_H = 0.84;
static const double SPOT_CENTER_X = -0.30, SPOT_CENTER_Y = 0.00;
static const double SPOT_TOP_Z    = 0.00;   // TF: spot_body e base_link stessa z (offset 0). Valutare se mettere offset per la piastra
static const double SPOT_CENTER_Z = SPOT_TOP_Z - SPOT_H / 2.0;

//ROI davanti a spot
// ROI_X_MAX --> da incrementare per poter spingere piu in profondità il target con LOOK_DEPTH (occhio anche a CAM_RANGE)
static const double ROI_X_MIN = 0.10,  ROI_X_MAX = 2.00;
static const double ROI_Y_MIN = -1.00, ROI_Y_MAX = 1.00;
static const double ROI_Z_MIN = -0.85, ROI_Z_MAX = 1.30;   // fino al pavimento (circa 0.84)

//Octree
static const double OCTREE_RES = 0.04;            // lato del voxel [m]
static const double MAX_SENSOR_RANGE = 3.0;       // taglio raggi insert [m]

// Origine del ray casting = posizione della CAMERA DEL BRACCIO (camera_link) in
// base_link, letta dalla TF a ogni ciclo (vedi updateSensorOrigin). camera_link si
// muove col braccio, quindi NON e' una costante. Il valore qui sotto e' solo il
// FALLBACK se la TF non e' disponibile: midpoint delle due camere frontali di Spot
// (comportamento delle versioni precedenti).
//   spot_frontright_fisheye = (0.086, -0.041, -0.043)
//   spot_frontleft_fisheye  = (0.082,  0.032, -0.044)
static const Eigen::Vector3d SENSOR_ORIGIN_FALLBACK(0.084, 0.0, -0.044);

// Spessore d'ombra marcato OCCUPATO dietro ogni superficie osservata (applicato solo lungo x del base_link). Usato sia per ragionamento sia per sicurezza di movit
static const double DEPTH_UNCERTAINTY = 0.10;     // [m]

// Passo (in voxel) della scansione di occlusione: piu' grande = meno raggi = piu' veloce.
static const int    SCAN_STRIDE  = 2;
// Variabile per "mirare" piu' in profondita' nella zona nascosta.
static const double LOOK_DEPTH   = 3.0;

// Z del pavimento approssimata da altezza spot
static const double FLOOR_Z = -0.50; // -0.84

// Variabile per alzare su z il target (come LOOK_DEPTH ma in altezza)
static const double TARGET_MIN_Z = -0.60;

//Camera dell'EE (per l'information gain)
static const double CAM_HFOV  = 70.0 * M_PI / 180.0;  // FOV (cono) [rad]
//Distanza massima di visuale (valore pessimistico)
static const double CAM_RANGE = 4.0;

//Campionamento viewpoint DENTRO il workspace del braccio (approssimazione)
static const std::vector<double> WS_RADII   = {0.40, 0.60, 0.80};   // distanza sferica da base_link
static const std::vector<double> WS_ELEV_DG = {-30, 0, 30, 60};     // elevazioni (z)
static const std::vector<double> WS_AZIM_DG = {-90, -60, -30, 0, 30, 60, 90}; // azimut 0 = verso la scena (+x)
static const int    MAX_FRONTIER_EVAL = 600;   // sotto-campionamento per lo scoring
static const int    MAX_PLAN_ATTEMPTS = 20;    // quanti candidati provare a pianificare

// Traslazioni VIRTUALI di base_link lungo y (destra/sinistra) per valutare, con la
// stessa identica logica, quali pose sarebbero ottimali se il CANE fosse spostato di
// lato. 0.0 = posizione reale (l'unica realmente pianificabile/eseguibile); le altre
// sono solo "what-if" valutate e disegnate in RViz. Modifica BASE_SHIFT_Y per l'entita' [m].
static const double BASE_SHIFT_Y = 0.80;   // [m]
static const std::vector<double> BASE_OFFSETS_Y = {0.0, BASE_SHIFT_Y, -BASE_SHIFT_Y};

// Il blocco d'ombra "interessante" deve essere causato da un OGGETTO, non dal
// pavimento: il voxel bloccante deve stare almeno OBJECT_MIN_H sopra il pavimento.
static const double OBJECT_MIN_H = 0.12;

//Debug / esecuzione
static const std::string MARKER_TOPIC = "nbv_markers";
static const int    MAX_MARKER_CANDS  = 12;    // quanti candidati mostrare in rviz
static const bool   EXECUTE_MOTION    = true;  // false = solo pianifica. Utile per rosbag o per testare in scenario reale senza movimento

//Movimento LATERALE di Spot (per realizzare una base virtuale). Il comando viene pubblicato
//su goaltospot ed eseguito dal nodo Boston Dynamics in ascolto su quel topic. Quel nodo
//interpreta il messaggio come posa ASSOLUTA in odom (goal_x, goal_y, goal_heading), quindi
//per ottenere un laterale RISPETTO AL CORPO (lungo y di spot_body) con yaw invariato, il
//comando va calcolato leggendo la posa attuale di spot_body in spot_odom (vedi moveSpotLateral).
static const std::string SPOT_GOAL_TOPIC = "goaltospot";
static const std::string SPOT_GOAL_FRAME = "spot_odom";   // frame in cui e' espresso il goal (odom)
static const std::string SPOT_BODY_FRAME = "spot_body";   // corpo di Spot (per leggere posa/yaw da TF)
static const double SPOT_LATERAL_STEP = 0.80;      // [m] passo laterale lungo y_body (coerente con BASE_SHIFT_Y)
static const double SPOT_MOVE_WAIT_S = 12.0;       // [s] attesa (senza feedback) perche' Spot completi lo spostamento


static inline octomap::point3d toOcto(const Eigen::Vector3d & v) {
  return octomap::point3d((float)v.x(), (float)v.y(), (float)v.z());
}

// Posa look-at: il frame comandato (camera_link, +z = asse ottico) punta il target.
// Convenzione ottica: +z avanti (verso il target), +x a DESTRA, +y in BASSO.
// Per ottenerla si usa come riferimento il "giu' del mondo" (0,0,-1): cosi'
//   x = up × z  -> a destra ,  y = z × x  -> in basso.
// (Con up = (0,0,+1) la camera risultava capovolta: x a sinistra, y in alto.)
static geometry_msgs::msg::Pose lookAtPose(const Eigen::Vector3d & eye,
                                           const Eigen::Vector3d & target)
{
  Eigen::Vector3d z = (target - eye).normalized();
  Eigen::Vector3d up(0, 0, -1);
  if (std::abs(z.dot(up)) > 0.95) up = Eigen::Vector3d(1, 0, 0);   // caso quasi verticale
  Eigen::Vector3d x = up.cross(z).normalized();
  Eigen::Vector3d y = z.cross(x);
  Eigen::Matrix3d R; R.col(0) = x; R.col(1) = y; R.col(2) = z;
  Eigen::Quaterniond q(R); q.normalize();
  geometry_msgs::msg::Pose p;
  p.position.x = eye.x(); p.position.y = eye.y(); p.position.z = eye.z();
  p.orientation.x = q.x(); p.orientation.y = q.y();
  p.orientation.z = q.z(); p.orientation.w = q.w();
  return p;
}

struct Candidate {
  Eigen::Vector3d eye;
  double score;    // gain + clear (ordinamento)
  double base_y;   // traslazione y della base (0 = reale)
  double gain;     // voxel occlusi visti (metrica chiave per l'esperimento)
  double clear;    // 0..1 avvicinamento alla vista (spareggio quando gain=0)
};

// Riga della tabella di validazione: candidato + fascia (ALTA/MEDIANA/BASSA/-)
struct TableRow { Candidate cand; std::string tier; };

// =============================================================================
class KinovaNbvExplorer
{
public:
  explicit KinovaNbvExplorer(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    move_group_(node, PLANNING_GROUP),
    tf_buffer_(node->get_clock()),
    tf_listener_(tf_buffer_)
  {
    move_group_.setPoseReferenceFrame(ARM_BASE_FRAME);
    move_group_.setPlanningTime(2.0);   // piu' basso: prova piu' candidati
    move_group_.setNumPlanningAttempts(10);
    move_group_.setMaxVelocityScalingFactor(0.10);
    move_group_.setMaxAccelerationScalingFactor(0.10);

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      CLOUD_TOPIC, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(cloud_mtx_);
        last_cloud_ = msg;
      });

    apply_client_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "apply_planning_scene");

    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      MARKER_TOPIC, rclcpp::QoS(1).transient_local());

    spot_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      SPOT_GOAL_TOPIC, rclcpp::QoS(10));

    // RCLCPP_INFO(node_->get_logger(), "KinovaNbvExplorer pronto. EE: %s",
    //             move_group_.getEndEffectorLink().c_str());
  }

  //Protection zone per Spot (attached object su base_link in modo da non essere fisso nel mondo)
  void addSpotProtectionZone()
  {
    moveit_msgs::msg::CollisionObject spot;
    spot.header.frame_id = ARM_BASE_FRAME;
    spot.id = "spot_body";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {SPOT_L, SPOT_W, SPOT_H};
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = SPOT_CENTER_X; pose.position.y = SPOT_CENTER_Y;
    pose.position.z = SPOT_CENTER_Z;
    spot.primitives.push_back(box);
    spot.primitive_poses.push_back(pose);
    spot.operation = spot.ADD;

    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = ARM_BASE_FRAME;
    attached.object = spot;
    attached.touch_links = {ARM_BASE_FRAME};
    planning_scene_interface_.applyAttachedCollisionObject(attached);
    rclcpp::sleep_for(1s);
    // RCLCPP_INFO(node_->get_logger(), "Protection zone Spot aggiunta.");
  }

  bool waitForCloud(std::chrono::seconds timeout)
  {
    auto start = node_->now();
    while (rclcpp::ok()) {
      { std::lock_guard<std::mutex> lk(cloud_mtx_); if (last_cloud_) return true; }
      if ((node_->now() - start).seconds() > timeout.count()) return false;
      rclcpp::sleep_for(200ms);
    }
    return false;
  }

  //UN ciclo di esplorazione next-best-view.
  bool exploreOnce()
  {
    CloudT::Ptr cloud = getCloudInBase();
    if (!cloud) return false;

    updateSensorOrigin();               // origine ray casting = camera_link (camera del braccio)
    buildOctree(cloud);                 // libero / occupato / sconosciuto
    bakeDepthUncertainty();             // ombra dietro le superfici -> occupato
    injectOctreeAsCollision();          // passo a movit forma reale ostacolo + ombra

    std::vector<Eigen::Vector3d> occluded = collectOccluded();
    if (occluded.empty()) {
      RCLCPP_INFO(node_->get_logger(), "Nessuna zona occlusa da esplorare.");
      return false;
    }
    Eigen::Vector3d target = centroid(occluded);
    if (LOOK_DEPTH > 0.0) {
      // spinta ORIZZONTALE (piu' a fondo dietro l'ostacolo)
      Eigen::Vector3d d = target - sensor_origin_; d.z() = 0.0;
      if (d.norm() > 1e-6) target += d.normalized() * LOOK_DEPTH;
    }
    //Clamp: entro la ROI e sopra la quota minima del target (indipendente dal pavimento).
    target.x() = std::clamp(target.x(), ROI_X_MIN, ROI_X_MAX);
    target.y() = std::clamp(target.y(), ROI_Y_MIN, ROI_Y_MAX);
    target.z() = std::clamp(target.z(), TARGET_MIN_Z, ROI_Z_MAX);
    // RCLCPP_INFO(node_->get_logger(),
    //   "Volume occluso: %zu voxel, target (%.2f, %.2f, %.2f)",
    //   occluded.size(), target.x(), target.y(), target.z());

    std::vector<Eigen::Vector3d> fe = subsample(occluded, MAX_FRONTIER_EVAL);

    std::vector<Candidate> cands = buildCandidates(target, fe);
    if (cands.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Nessuna posa candidata valida.");
      publishMarkers(target, fe, cands, std::nullopt);
      return false;
    }

    // =========================== MODALITA' VALIDAZIONE ===========================
    // Invece di scegliere ed eseguire in automatico la posa migliore, si costruisce una
    // TABELLA di pose (per ogni base: 5 alte / 5 mediane / 5 basse per score) e si lascia
    // scegliere all'operatore quale eseguire, digitandone l'ID a terminale. Serve a
    // dimostrare che dalla posa "buona" la persona nascosta e' visibile (YOLO) e dalle
    // altre no. La tabella e' una fotografia della scena iniziale: NON si aggiorna.
    std::vector<TableRow> table = buildTable(cands);
    publishMarkers(target, fe, cands, std::nullopt);   // mostra scena/candidati in rviz
    if (table.empty()) {
      std::printf("\nNessuna posa candidata disponibile.\n");
      return false;
    }

    // Loop di menu: ristampa la tabella, chiede un ID, esegue. 'q' per uscire.
    while (rclcpp::ok()) {
      printTable(table);
      std::printf("\nDigita l'ID della posa da eseguire (q = esci): ");
      std::fflush(stdout);

      std::string line;
      if (!std::getline(std::cin, line)) break;         // stdin chiuso / EOF
      // trim spazi iniziali e finali
      while (!line.empty() &&
             (line.back()==' '||line.back()=='\n'||line.back()=='\r'||line.back()=='\t'))
        line.pop_back();
      const size_t s0 = line.find_first_not_of(" \t");
      if (s0 == std::string::npos) continue;            // riga vuota
      line = line.substr(s0);
      if (line == "q" || line == "Q") { std::printf("Uscita dal nodo di validazione.\n"); break; }

      int id;
      try { id = std::stoi(line); }
      catch (...) { std::printf("Input non valido: digita un numero oppure 'q'.\n"); continue; }
      if (id < 0 || id >= (int)table.size()) {
        std::printf("ID fuori range (0..%d).\n", (int)table.size() - 1); continue;
      }

      const Candidate c = table[id].cand;
      const bool base_virtuale = std::abs(c.base_y) > 1e-6;

      // Verifica di raggiungibilita' PRIMA di muovere qualsiasi cosa: per una base virtuale
      // si pianifica la posa RELATIVA (eye e target traslati di -base_y), cioe' quella che
      // il braccio dovra' assumere DOPO che Spot si sara' spostato.
      Eigen::Vector3d eye_t = c.eye, target_t = target;
      if (base_virtuale) { eye_t.y() -= c.base_y; target_t.y() -= c.base_y; }
      geometry_msgs::msg::Pose goal = lookAtPose(eye_t, target_t);
      move_group_.setStartStateToCurrentState();
      move_group_.setPoseTarget(goal, CAMERA_FRAME);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        std::printf("Posa ID %d NON raggiungibile dal braccio. Scegline un'altra.\n", id);
        continue;
      }
      std::printf("Posa ID %d raggiungibile (base y=%+.2f, gain=%.0f, score=%.1f).\n",
                  id, c.base_y, c.gain, c.score);

      if (!EXECUTE_MOTION) {
        std::printf("EXECUTE_MOTION=false: verifico soltanto, non eseguo. Torno al menu.\n");
        publishMarkers(target, fe, cands, c);
        continue;
      }

      // Base virtuale: PRIMA Spot (laterale lungo y del corpo), POI il braccio.
      if (base_virtuale) {
        std::printf("Base VIRTUALE: sposto prima Spot, poi eseguo il braccio.\n");
        moveSpotLateral(c.base_y);
        std::printf("Attendo %.0f s che Spot completi lo spostamento...\n", SPOT_MOVE_WAIT_S);
        rclcpp::sleep_for(std::chrono::milliseconds((int)(SPOT_MOVE_WAIT_S * 1000)));
      }
      std::printf("Eseguo il movimento del braccio...\n");
      move_group_.execute(plan);
      std::printf("Fatto. Riporta braccio (e Spot, se spostato) allo status quo prima della prossima posa.\n");
      publishMarkers(target, fe, cands, c);
    }
    return true;
  }

private:
  //Origine del ray casting = camera del braccio (camera_link) in base_link, letta da TF.
  //camera_link si muove col braccio: va riletta a ogni ciclo (non e' una costante). Con
  //un solo octree, l'origine incide su spazio libero, ombre e occlusione; se la TF non
  //e' disponibile si torna al fallback (midpoint camere frontali Spot) con un warning.
  bool updateSensorOrigin()
  {
    try {
      auto tf = tf_buffer_.lookupTransform(
        ARM_BASE_FRAME, CAMERA_FRAME, tf2::TimePointZero, 500ms);
      sensor_origin_ = Eigen::Vector3d(tf.transform.translation.x,
                                       tf.transform.translation.y,
                                       tf.transform.translation.z);
      // RCLCPP_INFO(node_->get_logger(),
      //   "Origine ray casting = camera_link (%.3f, %.3f, %.3f).",
      //   sensor_origin_.x(), sensor_origin_.y(), sensor_origin_.z());
      return true;
    } catch (const tf2::TransformException & e) {
      sensor_origin_ = SENSOR_ORIGIN_FALLBACK;
      RCLCPP_WARN(node_->get_logger(),
        "TF %s->%s non disponibile (%s). Uso fallback camere Spot (%.3f, %.3f, %.3f).",
        ARM_BASE_FRAME.c_str(), CAMERA_FRAME.c_str(), e.what(),
        sensor_origin_.x(), sensor_origin_.y(), sensor_origin_.z());
      return false;
    }
  }

  //Comanda a Spot uno spostamento LATERALE rispetto al proprio CORPO (lungo y di spot_body),
  //MANTENENDO lo yaw invariato, qualunque sia l'orientazione di Spot in spot_odom.
  //Il nodo di Spot interpreta goaltospot come posa ASSOLUTA in odom (goal_x, goal_y, goal_heading):
  //  1) leggo la posa attuale di spot_body in spot_odom (px, py, yaw) dalla TF;
  //  2) mi sposto di SPOT_LATERAL_STEP lungo il +y del CORPO, espresso in odom = (-sin yaw, cos yaw);
  //  3) ripubblico lo stesso yaw, cosi' l'orientazione non cambia.
  //Direzione (REP-103: +y del corpo = SINISTRA):
  //  base_y > 0 = base virtuale a SINISTRA (+y) -> Spot a SINISTRA -> lateral = +STEP
  //  base_y < 0 = base virtuale a DESTRA  (-y) -> Spot a DESTRA   -> lateral = -STEP
  void moveSpotLateral(double base_y)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(
        SPOT_GOAL_FRAME, SPOT_BODY_FRAME, tf2::TimePointZero, 500ms);

      const double px = tf.transform.translation.x;
      const double py = tf.transform.translation.y;
      const double qx = tf.transform.rotation.x;
      const double qy = tf.transform.rotation.y;
      const double qz = tf.transform.rotation.z;
      const double qw = tf.transform.rotation.w;
      const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                    1.0 - 2.0 * (qy * qy + qz * qz));

      // Laterale nel corpo: +STEP = sinistra, -STEP = destra (secondo la base scelta).
      const double lateral = (base_y > 0.0) ? SPOT_LATERAL_STEP : -SPOT_LATERAL_STEP;

      // +y del corpo espresso in odom = (-sin yaw, cos yaw). Goal assoluto in odom:
      const double gx = px + lateral * (-std::sin(yaw));
      const double gy = py + lateral * ( std::cos(yaw));

      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = SPOT_GOAL_FRAME;
      msg.header.stamp = node_->now();
      msg.pose.position.x = gx;
      msg.pose.position.y = gy;
      msg.pose.position.z = 0.0;
      // yaw INVARIATO: quaternione dello yaw attuale (asse z).
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = std::sin(yaw * 0.5);
      msg.pose.orientation.w = std::cos(yaw * 0.5);

      RCLCPP_INFO(node_->get_logger(),
        "Spot: laterale %s di %.2f m lungo y_body (yaw invariato %.1f deg). "
        "Posa attuale odom (%.2f, %.2f) -> goal odom (%.2f, %.2f).",
        (base_y > 0.0) ? "SINISTRA" : "DESTRA", SPOT_LATERAL_STEP,
        yaw * 180.0 / M_PI, px, py, gx, gy);
      spot_goal_pub_->publish(msg);
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(node_->get_logger(),
        "TF %s->%s non disponibile (%s): spostamento di Spot ANNULLATO.",
        SPOT_GOAL_FRAME.c_str(), SPOT_BODY_FRAME.c_str(), e.what());
    }
  }

  //Percezione: cloud -> base_link 
  CloudT::Ptr getCloudInBase()
  {
    sensor_msgs::msg::PointCloud2::SharedPtr raw;
    { std::lock_guard<std::mutex> lk(cloud_mtx_); raw = last_cloud_; }
    if (!raw) return nullptr;

    sensor_msgs::msg::PointCloud2 in_base;
    try {
      auto tf = tf_buffer_.lookupTransform(
        ARM_BASE_FRAME, raw->header.frame_id, tf2::TimePointZero, 500ms);
      tf2::doTransform(*raw, in_base, tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(node_->get_logger(), "TF non disponibile: %s", e.what());
      return nullptr;
    }
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(in_base, *cloud);
    if (cloud->empty()) return nullptr;

    // Ritaglio nella ROI per limitare il costo dell'octree (ed escludere ostacoli troppo laterali)
    CloudT::Ptr roi(new CloudT);
    pcl::CropBox<PointT> crop;
    crop.setMin(Eigen::Vector4f(ROI_X_MIN, ROI_Y_MIN, ROI_Z_MIN, 1.0f));
    crop.setMax(Eigen::Vector4f(ROI_X_MAX, ROI_Y_MAX, ROI_Z_MAX, 1.0f));
    crop.setInputCloud(cloud);
    crop.filter(*roi);
    return roi->empty() ? nullptr : roi;
  }

  //Costruzione octree con raycasting dal sensore:
  //insertPointCloud definisce libero lo spazio lungo i raggi tra sensore->punto e marca
  //occupato il punto finale. I voxel mai attraversati sono definiti sconosciuti
  void buildOctree(const CloudT::Ptr & cloud)
  {
    tree_ = std::make_shared<octomap::OcTree>(OCTREE_RES);
    octomap::Pointcloud oc;
    oc.reserve(cloud->size());
    for (const auto & p : cloud->points)
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
        oc.push_back(p.x, p.y, p.z);
    tree_->insertPointCloud(oc, toOcto(sensor_origin_), MAX_SENSOR_RANGE);
    tree_->updateInnerOccupancy();
  }

  enum class Vox { FREE, OCC, UNKNOWN };
  Vox classify(double x, double y, double z) const
  {
    octomap::OcTreeNode * n = tree_->search(x, y, z);
    if (!n) return Vox::UNKNOWN;
    return tree_->isNodeOccupied(n) ? Vox::OCC : Vox::FREE;
  }

  //Strato d'ombra dietro le superfici: occupato (fino a DEPTH_UNCERTAINTY ovviamente)
  void bakeDepthUncertainty()
  {
    std::vector<Eigen::Vector3d> occ;
    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it)
      if (tree_->isNodeOccupied(*it)) {
        auto c = it.getCoordinate();
        occ.emplace_back(c.x(), c.y(), c.z());
      }
    for (const auto & c : occ) {
      Eigen::Vector3d dir = (c - sensor_origin_).normalized();   // lontano dal sensore
      for (double d = OCTREE_RES; d <= DEPTH_UNCERTAINTY; d += OCTREE_RES) {
        Eigen::Vector3d p = c + dir * d;
        tree_->updateNode(toOcto(p), true);   // marca occupata l'ombra
      }
    }
    tree_->updateInnerOccupancy();
  }

  //Passo l'octree a MoveIt per collosion avoidance
  //Forma reale preservata (non AABB) + ombra di profondita' inclusa
  void injectOctreeAsCollision()
  {
    octomap_msgs::msg::Octomap omsg;
    if (!octomap_msgs::binaryMapToMsg(*tree_, omsg)) {
      RCLCPP_WARN(node_->get_logger(), "binaryMapToMsg fallita.");
      return;
    }
    omsg.header.frame_id = ARM_BASE_FRAME;

    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    req->scene.is_diff = true;
    req->scene.world.octomap.header.frame_id = ARM_BASE_FRAME;
    req->scene.world.octomap.origin.orientation.w = 1.0;
    req->scene.world.octomap.octomap = omsg;

    if (!apply_client_->wait_for_service(3s)) {
      RCLCPP_WARN(node_->get_logger(), "apply_planning_scene non disponibile.");
      return;
    }
    auto fut = apply_client_->async_send_request(req);
    if (fut.wait_for(3s) == std::future_status::ready && fut.get()->success)
      { /* RCLCPP_INFO(node_->get_logger(), "Octree iniettato nella planning scene."); */ }
    else
      RCLCPP_WARN(node_->get_logger(), "Iniezione octree non confermata.");
    rclcpp::sleep_for(300ms);
  }

  //Volume OCCLUSO: voxel sconosciuti in ombra dietro le superfici
  std::vector<Eigen::Vector3d> collectOccluded()
  {
    std::vector<Eigen::Vector3d> out;
    const double r = OCTREE_RES * SCAN_STRIDE;   // passo grezzo per contenere i raggi
    for (double x = ROI_X_MIN; x <= ROI_X_MAX; x += r)
      for (double y = ROI_Y_MIN; y <= ROI_Y_MAX; y += r)
        for (double z = ROI_Z_MIN; z <= ROI_Z_MAX; z += r) {
          if (z <= FLOOR_Z + OCTREE_RES) continue;          // scarta il sottopavimento per non considerarare ombra gigante sotto il suolo
          if (classify(x, y, z) != Vox::UNKNOWN) continue;   // solo sconosciuto
          // in ombra dietro un oggetto
          Eigen::Vector3d v(x, y, z);
          double dist = (v - sensor_origin_).norm();
          octomap::point3d end;
          bool blocked = tree_->castRay(
            toOcto(sensor_origin_), toOcto((v - sensor_origin_).normalized()),
            end, true, dist - OCTREE_RES);
          if (blocked && end.z() > FLOOR_Z + OBJECT_MIN_H) out.push_back(v);
        }
    return out;
  }

  //Information gain: quanti voxel della frontiera vedrei da questo viewpoint
  double infoGain(const Eigen::Vector3d & cam, const Eigen::Vector3d & axis,
                  const std::vector<Eigen::Vector3d> & frontier) const
  {
    int vis = 0;
    for (const auto & f : frontier) {
      Eigen::Vector3d v = f - cam;
      double dist = v.norm();
      if (dist > CAM_RANGE || dist < 1e-3) continue;
      Eigen::Vector3d dir = v / dist;
      if (std::acos(std::clamp(axis.dot(dir), -1.0, 1.0)) > CAM_HFOV * 0.5) continue;
      octomap::point3d end;
      bool blocked = tree_->castRay(toOcto(cam), toOcto(dir), end, true, dist - OCTREE_RES);
      if (!blocked) ++vis;   // linea di vista libera fino al voxel occluso
    }
    return (double)vis;
  }

  //frazione [0..1] del raggio camera->target libera (prima di incontrare un ostacolo)
  // 1.0 = vista diretta del target; <1 = raggio bloccato, tanto piu' piccola quanto prima viene ostruito il raggio
  double clearToTarget(const Eigen::Vector3d & cam, const Eigen::Vector3d & target) const
  {
    Eigen::Vector3d v = target - cam;
    double dist = v.norm();
    if (dist < 1e-3) return 1.0;
    octomap::point3d end;
    bool hit = tree_->castRay(toOcto(cam), toOcto(v / dist), end, true, dist);
    if (!hit) return 1.0;
    double hitDist = (Eigen::Vector3d(end.x(), end.y(), end.z()) - cam).norm();
    return std::clamp(hitDist / dist, 0.0, 1.0);
  }

  //Generazione candidati attorno a base_link
  // I viewpoint sono campionati attorno alla BASE del braccio (origine di base_link),
  //Ogni candidato e' orientato verso il target e valutato con score = gain + clearToTarget (e successiva pianificazione di movit)
  std::vector<Candidate> buildCandidates(const Eigen::Vector3d & target,
                                         const std::vector<Eigen::Vector3d> & fe)
  {
    std::vector<Candidate> cands;
    // Oltre alla base REALE (by=0) si valutano anche basi VIRTUALI traslate in y: le
    // stesse pose prestabilite, ma "immaginando" il cane spostato di lato. Serve a
    // capire se un riposizionamento del cane darebbe viste migliori. Solo by=0 verra'
    // poi realmente pianificata/eseguita (vedi exploreOnce).
    for (double by : BASE_OFFSETS_Y) {
      const Eigen::Vector3d base(0.0, by, 0.0);   // origine (anche virtuale) della base del braccio
      for (double R : WS_RADII)
        for (double eldg : WS_ELEV_DG)
          for (double azdg : WS_AZIM_DG) {
            double el = eldg * M_PI/180.0, az = azdg * M_PI/180.0;
            Eigen::Vector3d dir(std::cos(el)*std::cos(az),
                                std::cos(el)*std::sin(az),
                                std::sin(el));
            Eigen::Vector3d eye = base + R * dir;                 // dentro la portata (della base virtuale)

            if (eye.z() < FLOOR_Z + 0.05) continue;               // sopra il pavimento
            if (classify(eye.x(), eye.y(), eye.z()) == Vox::OCC) continue;  // non in un ostacolo

            Eigen::Vector3d axis = (target - eye).normalized();
            double gain  = infoGain(eye, axis, fe);                // voxel occlusi visti (nella scena reale)
            double clear = clearToTarget(eye, target);             // 0..1 avvicinamento alla vista
            // Score graduale: il gain domina; se nessuno vede nulla, clear ordina
            // comunque per "quanto ci si avvicina a vedere il target"
            cands.push_back({eye, gain + clear, by, gain, clear});
          }
    }

    std::sort(cands.begin(), cands.end(),
      [](const Candidate & a, const Candidate & b){ return a.score > b.score; });
    return cands;
  }

  //Costruisce la tabella di validazione: per OGNI base, fino a 15 pose (5 alte, 5 mediane,
  //5 basse per score). Se una base ha <=15 candidati, li mostra tutti senza duplicati.
  //cands e' gia' ordinato per score decrescente, quindi il filtro per base preserva l'ordine.
  static std::vector<TableRow> buildTable(const std::vector<Candidate> & cands)
  {
    std::vector<TableRow> table;
    for (double by : BASE_OFFSETS_Y) {
      std::vector<Candidate> v;
      for (const auto & c : cands)
        if (std::abs(c.base_y - by) < 1e-6) v.push_back(c);
      if (v.empty()) continue;

      if ((int)v.size() <= 15) {                 // poche pose: tutte, senza fasce
        for (const auto & c : v) table.push_back({c, "-"});
      } else {                                   // 5 alte / 5 mediane / 5 basse
        const int n = (int)v.size();
        const int mid = n / 2;
        for (int i = 0; i < 5; ++i)              table.push_back({v[i], "ALTA"});
        for (int i = mid - 2; i <= mid + 2; ++i) table.push_back({v[i], "MEDIANA"});
        for (int i = n - 5; i < n; ++i)          table.push_back({v[i], "BASSA"});
      }
    }
    return table;
  }

  //Stampa la tabella a terminale (std::cout, non il logger, per tenerla pulita e leggibile).
  static void printTable(const std::vector<TableRow> & table)
  {
    std::printf("\n=================== POSE CANDIDATE (validazione) ===================\n");
    std::printf(" ID | base_y | fascia  | gain | clear | score | tipo\n");
    std::printf("-------------------------------------------------------------------\n");
    double cur_by = 1e9;
    for (size_t i = 0; i < table.size(); ++i) {
      const auto & r = table[i];
      if (std::abs(r.cand.base_y - cur_by) > 1e-6) {   // intestazione di base
        cur_by = r.cand.base_y;
        const bool v = std::abs(cur_by) > 1e-6;
        std::printf("  --- BASE y=%+.2f (%s) ---\n", cur_by, v ? "VIRTUALE" : "REALE");
      }
      const bool v = std::abs(r.cand.base_y) > 1e-6;
      std::printf(" %2zu | %+5.2f | %-7s | %4.0f | %5.2f | %5.1f | %s\n",
                  i, r.cand.base_y, r.tier.c_str(),
                  r.cand.gain, r.cand.clear, r.cand.score,
                  v ? "VIRTUALE (muove Spot)" : "REALE");
    }
    std::printf("===================================================================\n");
  }

  //Marker di debug per rviz
  void publishMarkers(const Eigen::Vector3d & target,
                      const std::vector<Eigen::Vector3d> & frontier,
                      const std::vector<Candidate> & cands,
                      const std::optional<Candidate> & chosen)
  {
    visualization_msgs::msg::MarkerArray arr;
    auto stamp = node_->now();
    int id = 0;
    auto base = [&](int32_t type){
      visualization_msgs::msg::Marker m;
      m.header.frame_id = ARM_BASE_FRAME;
      m.header.stamp = stamp;
      m.id = id++;
      m.type = type;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;   // lifetime di default = 0 = per sempre
      return m;
    };

    //TARGET: sfera magenta
    {
      auto m = base(visualization_msgs::msg::Marker::SPHERE);
      m.ns = "target";
      m.pose.position.x = target.x(); m.pose.position.y = target.y(); m.pose.position.z = target.z();
      m.scale.x = m.scale.y = m.scale.z = 0.07;
      m.color.r = 1.0; m.color.b = 1.0; m.color.a = 1.0;
      arr.markers.push_back(m);
    }

    //FRONTIERA OCCLUSA: cubetti semi trasparenti
    {
      auto m = base(visualization_msgs::msg::Marker::CUBE_LIST);
      m.ns = "frontier";
      m.scale.x = m.scale.y = m.scale.z = OCTREE_RES;
      m.color.g = 0.9; m.color.b = 1.0; m.color.a = 0.25;
      for (const auto & f : frontier) {
        geometry_msgs::msg::Point p; p.x = f.x(); p.y = f.y(); p.z = f.z();
        m.points.push_back(p);
      }
      arr.markers.push_back(m);
    }

    //BASI VIRTUALI: box translucido di Spot in ogni posizione y "immaginata" (by != 0),
    //cosi' in RViz si vede dove sarebbe il cane. Colori: +y azzurro, -y arancio.
    for (double by : BASE_OFFSETS_Y) {
      if (std::abs(by) < 1e-6) continue;   // la base reale e' gia' la protection zone
      auto bm = base(visualization_msgs::msg::Marker::CUBE);
      bm.ns = "virtual_bases";
      bm.pose.position.x = SPOT_CENTER_X;
      bm.pose.position.y = SPOT_CENTER_Y + by;
      bm.pose.position.z = SPOT_CENTER_Z;
      bm.scale.x = SPOT_L; bm.scale.y = SPOT_W; bm.scale.z = SPOT_H;
      bm.color.r = (by > 0 ? 0.2f : 1.0f); bm.color.g = 0.6f; bm.color.b = (by > 0 ? 1.0f : 0.2f); bm.color.a = 0.15f;
      arr.markers.push_back(bm);

      auto tx = base(visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      tx.ns = "virtual_bases";
      tx.pose.position.x = 0.0; tx.pose.position.y = by; tx.pose.position.z = 0.12;
      tx.scale.z = 0.06;
      tx.color.r = tx.color.g = tx.color.b = 1.0; tx.color.a = 1.0;
      char lb[32]; std::snprintf(lb, sizeof(lb), "base y=%+.2f", by);
      tx.text = lb;
      arr.markers.push_back(tx);
    }

    //CANDIDATI per ogni base (reale + virtuali): frecce eye->target, verde(score alto)->rosso.
    //Il colore e' normalizzato al MIGLIOR score GLOBALE, cosi' le basi si confrontano tra loro
    //(la base che vede di piu' avra' frecce piu' verdi). Un namespace per base -> toggle in RViz.
    double gmax = cands.empty() ? 1.0 : cands.front().score;   // cands gia' ordinati desc
    const int PER_BASE = std::max(1, MAX_MARKER_CANDS / (int)BASE_OFFSETS_Y.size());
    for (double by : BASE_OFFSETS_Y) {
      char nsbuf[32]; std::snprintf(nsbuf, sizeof(nsbuf), "cand_y%+.2f", by);
      int drawn = 0; bool firstOfBase = true;
      for (const auto & c : cands) {
        if (std::abs(c.base_y - by) > 1e-6) continue;   // solo i candidati di questa base
        if (drawn >= PER_BASE) break;
        ++drawn;
        double t = (gmax > 0) ? c.score / gmax : 0.0;
        auto m = base(visualization_msgs::msg::Marker::ARROW);
        m.ns = nsbuf;
        geometry_msgs::msg::Point a, b;
        a.x = c.eye.x(); a.y = c.eye.y(); a.z = c.eye.z();
        b.x = target.x(); b.y = target.y(); b.z = target.z();
        m.points.push_back(a); m.points.push_back(b);
        m.scale.x = 0.008;   // diametro asta
        m.scale.y = 0.02;    // diametro testa
        m.scale.z = 0.03;    // lunghezza testa
        m.color.r = (float)(1.0 - t); m.color.g = (float)t; m.color.a = 0.9f;
        arr.markers.push_back(m);

        if (firstOfBase) {   // score del MIGLIOR candidato di questa base
          firstOfBase = false;
          auto tx = base(visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
          tx.ns = nsbuf;
          tx.pose.position.x = c.eye.x(); tx.pose.position.y = c.eye.y();
          tx.pose.position.z = c.eye.z() + 0.05;
          tx.scale.z = 0.04;
          tx.color.r = tx.color.g = tx.color.b = 1.0; tx.color.a = 1.0;
          char sb[16]; std::snprintf(sb, sizeof(sb), "%.0f", c.score);
          tx.text = sb;
          arr.markers.push_back(tx);
        }
      }
    }

    //VIEWPOINT SCELTO: sfera + freccia bianca + etichetta.
    //VERDE = base reale (eseguibile subito). ARANCIO = base virtuale (servirebbe spostare Spot).
    if (chosen) {
      const bool virt = std::abs(chosen->base_y) > 1e-6;
      auto m = base(visualization_msgs::msg::Marker::SPHERE);
      m.ns = "chosen";
      m.pose.position.x = chosen->eye.x(); m.pose.position.y = chosen->eye.y(); m.pose.position.z = chosen->eye.z();
      m.scale.x = m.scale.y = m.scale.z = 0.09;
      if (virt) { m.color.r = 1.0; m.color.g = 0.55; m.color.b = 0.0; }
      else      { m.color.g = 1.0; }
      m.color.a = 1.0;
      arr.markers.push_back(m);

      auto a = base(visualization_msgs::msg::Marker::ARROW);
      a.ns = "chosen_arrow";
      geometry_msgs::msg::Point p0, p1;
      p0.x = chosen->eye.x(); p0.y = chosen->eye.y(); p0.z = chosen->eye.z();
      p1.x = target.x();  p1.y = target.y();  p1.z = target.z();
      a.points.push_back(p0); a.points.push_back(p1);
      a.scale.x = 0.015; a.scale.y = 0.035; a.scale.z = 0.05;
      a.color.r = a.color.g = a.color.b = 1.0; a.color.a = 1.0;
      arr.markers.push_back(a);

      //Etichetta: toglie ogni ambiguita' su quale base sia la vincitrice
      auto tx = base(visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      tx.ns = "chosen";
      tx.pose.position.x = chosen->eye.x(); tx.pose.position.y = chosen->eye.y();
      tx.pose.position.z = chosen->eye.z() + 0.13;
      tx.scale.z = 0.05;
      tx.color.r = tx.color.g = tx.color.b = 1.0; tx.color.a = 1.0;
      char cb[80];
      std::snprintf(cb, sizeof(cb), "SCELTA base y=%+.2f score=%.0f%s",
                    chosen->base_y, chosen->score, virt ? " (spostare Spot)" : "");
      tx.text = cb;
      arr.markers.push_back(tx);
    }

    marker_pub_->publish(arr);
    // RCLCPP_INFO(node_->get_logger(), "Marker di debug pubblicati su '%s' (%zu marker).",
    //             MARKER_TOPIC.c_str(), arr.markers.size());
  }

  
  static Eigen::Vector3d centroid(const std::vector<Eigen::Vector3d> & v) {
    Eigen::Vector3d c(0,0,0); for (auto & p : v) c += p; return c / (double)v.size();
  }
  static std::vector<Eigen::Vector3d> subsample(
      const std::vector<Eigen::Vector3d> & v, int maxN) {
    if ((int)v.size() <= maxN) return v;
    std::vector<Eigen::Vector3d> out; out.reserve(maxN);
    int stride = (int)v.size() / maxN + 1;
    for (size_t i = 0; i < v.size(); i += stride) out.push_back(v[i]);
    return out;
  }

  
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr spot_goal_pub_;

  std::mutex cloud_mtx_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
  std::shared_ptr<octomap::OcTree> tree_;
  Eigen::Vector3d sensor_origin_ = SENSOR_ORIGIN_FALLBACK;   // origine ray casting (camera_link, da TF)
};

// =============================================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("kinova_nbv_validation");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  auto explorer = std::make_shared<KinovaNbvExplorer>(node);
  explorer->addSpotProtectionZone();

  // RCLCPP_INFO(node->get_logger(), "In attesa della pointcloud...");
  if (explorer->waitForCloud(10s)) {
    explorer->exploreOnce();   // contiene il menu interattivo: ritorna quando si digita 'q'
  } else {
    RCLCPP_ERROR(node->get_logger(), "Nessuna pointcloud su %s", CLOUD_TOPIC.c_str());
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
