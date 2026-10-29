// Track planner for the missions whose layout is discovered on the fly:
// trackdrive and autocross.
//
// Delaunay-triangulates the mapped cones, walks the gate edges into an ordered
// centreline, and closes the loop once the path comes back on itself.
//
// Two loops run on their own callback groups: planning, which is the expensive
// one, and the heartbeat, which must stay on time because the supervisor reads
// liveness from it. The map subscription only parks the latest message, so a
// burst of map updates can never queue up planning work.
//
// This node and mirena_planning_known are alternative occupants of one slot:
// exactly one runs per mission, under the node name the supervisor monitors.
// They are deliberately independent of each other -- same topics, same
// contract, no shared code.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nlohmann/json.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mirena_common/msg/entity_list.hpp"
#include "mirena_common/msg/heartbeat.hpp"
#include "mirena_common/msg/track.hpp"

#include "mirena_planning/planner.hpp"
#include "mirena_planning/unknown/unknown_planner.hpp"

namespace mirena::planning
{

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

namespace
{

constexpr char kMapTopic[] = "slam/map";
constexpr char kTrackTopic[] = "planning/track";
constexpr char kHealthTopic[] = "health";
constexpr char kDelaunayTopic[] = "planning/debug/delaunay";

using Rgba = std::array<float, 4>;
constexpr Rgba kGrey{0.45f, 0.45f, 0.45f, 0.35f};
constexpr Rgba kGreen{0.15f, 0.9f, 0.3f, 0.9f};
constexpr Rgba kCyan{0.2f, 0.8f, 1.0f, 0.9f};
constexpr Rgba kMagenta{1.0f, 0.2f, 0.8f, 1.0f};
constexpr Rgba kOrange{1.0f, 0.55f, 0.0f, 1.0f};

geometry_msgs::msg::Point toPoint(Vec2 v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x;
  p.y = v.y;
  return p;
}

std_msgs::msg::ColorRGBA toColor(Rgba c)
{
  std_msgs::msg::ColorRGBA out;
  out.r = c[0]; out.g = c[1]; out.b = c[2]; out.a = c[3];
  return out;
}

Marker makeMarker(
  const std_msgs::msg::Header & header, const char * ns, std::int32_t type,
  double scale, Rgba color)
{
  Marker m;
  m.header = header;
  m.ns = ns;
  m.id = 0;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = m.scale.y = m.scale.z = scale;
  m.color = toColor(color);
  return m;
}

void addSegments(Marker & m, const std::vector<std::pair<Vec2, Vec2>> & segments)
{
  m.points.reserve(m.points.size() + 2 * segments.size());
  for (const auto & [a, b] : segments) {
    m.points.push_back(toPoint(a));
    m.points.push_back(toPoint(b));
  }
}

void addPoints(Marker & m, const std::vector<Vec2> & points)
{
  m.points.reserve(m.points.size() + points.size());
  for (const Vec2 & p : points) {m.points.push_back(toPoint(p));}
}

}  // namespace

class UnknownPlannerNode : public rclcpp::Node
{
public:
  UnknownPlannerNode()
  : rclcpp::Node("mirena_planning")
  {
    declareParameters();

    mission_ = get_parameter("mission").as_string();
    RCLCPP_INFO(get_logger(), "Planning mission '%s'", mission_.c_str());

    // The mission launch file picks the planner, so the two should never
    // disagree -- but a hand-rolled `ros2 run` can still get it wrong.
    if (mission_ != "trackdrive" && mission_ != "autocross") {
      RCLCPP_ERROR(
        get_logger(),
        "Mission '%s' has a surveyed layout. Launch mirena_planning_known for it.",
        mission_.c_str());
      throw std::runtime_error("wrong planner node for this mission");
    }

    republish_tolerance_ = get_parameter("republish_tolerance").as_double();
    fallback_frame_ = get_parameter("fallback_frame").as_string();
    publish_debug_ = get_parameter("publish_debug").as_bool();

    planner_ = makePlanner();
    if (!planner_) {throw std::runtime_error("planner could not be built");}

    plan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    report_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions map_options;
    map_options.callback_group = plan_group_;
    map_sub_ = create_subscription<mirena_common::msg::EntityList>(
      kMapTopic, 10,
      std::bind(&UnknownPlannerNode::mapCallback, this, std::placeholders::_1), map_options);

    // Latched: the track changes rarely once it settles, and a controller that
    // subscribes late (or restarts) should not have to wait for the next
    // material change to learn where the track is.
    track_pub_ = create_publisher<mirena_common::msg::Track>(
      kTrackTopic, rclcpp::QoS(1).transient_local());
    report_pub_ = create_publisher<mirena_common::msg::Heartbeat>(kHealthTopic, 10);
    if (publish_debug_) {
      delaunay_pub_ = create_publisher<MarkerArray>(kDelaunayTopic, 1);
    }

    plan_timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(get_parameter("plan_period").as_double()),
      std::bind(&UnknownPlannerNode::planIteration, this), plan_group_);
    report_timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(get_parameter("heartbeat_period").as_double()),
      std::bind(&UnknownPlannerNode::publishHeartbeat, this), report_group_);
  }

private:
  /// Everything the heartbeat reports, written by the planning loop.
  struct Report
  {
    std::optional<Vec2> start;
    std::size_t gates{0};
    bool closed{false};
    bool planned{false};
    double score{0.0};
    double plan_ms{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool has_plan{false};
  };

  /// The last track actually put on the wire, so we can tell a real change from
  /// the millimetre jitter of a re-planned centreline.
  struct Published
  {
    std::vector<Gate> gates;
    bool closed{false};
    bool valid{false};
  };

  void declareParameters()
  {
    // Declared and read in the same breath: the mission's values arrive as
    // launch arguments, so they are already in place by construction.
    const UnknownPlannerParams u;

    declare_parameter<std::string>("mission", "trackdrive");
    declare_parameter<std::string>("fallback_frame", "map");
    declare_parameter<double>("plan_period", 0.1);
    declare_parameter<double>("heartbeat_period", 0.1);
    declare_parameter<bool>("publish_debug", true);
    // Below this the centreline has not meaningfully moved, so republishing it
    // would only make the controller re-fit its spline and shift the arc length
    // its lap counter rides on.
    declare_parameter<double>("republish_tolerance", 0.25);

    declare_parameter<double>("curvature_weight", u.curvature_weight);
    declare_parameter<int>("lookahead", u.lookahead);
    declare_parameter<double>("min_track_width", u.min_track_width);
    declare_parameter<double>("max_track_width", u.max_track_width);
    declare_parameter<double>("cone_spacing", u.cone_spacing);
    declare_parameter<double>("max_gate_gap", u.max_gate_gap);
    declare_parameter<double>("start_gate_radius", u.start_gate_radius);
    declare_parameter<int>("closure_hysteresis", u.closure_hysteresis);
    declare_parameter<int>("max_path_points", u.max_path_points);
  }

  std::unique_ptr<Planner> makePlanner()
  {
    UnknownPlannerParams p;
    p.curvature_weight = get_parameter("curvature_weight").as_double();
    p.lookahead = static_cast<int>(get_parameter("lookahead").as_int());
    p.min_track_width = get_parameter("min_track_width").as_double();
    p.max_track_width = get_parameter("max_track_width").as_double();
    p.cone_spacing = get_parameter("cone_spacing").as_double();
    p.max_gate_gap = get_parameter("max_gate_gap").as_double();
    p.start_gate_radius = get_parameter("start_gate_radius").as_double();
    p.closure_hysteresis = static_cast<int>(get_parameter("closure_hysteresis").as_int());
    p.max_path_points = static_cast<int>(get_parameter("max_path_points").as_int());

    RCLCPP_INFO(get_logger(), "Planning an unknown track by Delaunay triangulation.");
    return std::make_unique<UnknownPlanner>(p);
  }

  void mapCallback(mirena_common::msg::EntityList::SharedPtr msg)
  {
    // Park it and return: planning runs on its own timer, so a fast map
    // publisher cannot build a backlog of expensive callbacks.
    std::lock_guard<std::mutex> lock(map_mutex_);
    pending_map_ = std::move(msg);
  }

  void planIteration()
  {
    if (!planner_ || !track_pub_) {return;}

    mirena_common::msg::EntityList::SharedPtr map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map = std::exchange(pending_map_, nullptr);
    }
    if (!map) {return;}

    std::vector<Observation> observations;
    observations.reserve(map->entities.size());
    for (const auto & entity : map->entities) {
      // Every cone counts. A colourless pipeline never reports a colour, so
      // discarding cones that fail to name one would discard all of them.
      const ConeSize size = coneSizeFromString(entity.type);
      observations.push_back(
        Observation{{entity.position.x, entity.position.y}, size});
    }

    std_msgs::msg::Header header = map->header;
    if (header.frame_id.empty()) {header.frame_id = fallback_frame_;}

    PlanDebug debug;
    debug.verbose = debugWanted();
    const auto started = std::chrono::steady_clock::now();
    const std::optional<Plan> plan = planner_->plan(observations, &debug);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();

    if (debug.verbose) {publishDebug(header, debug, plan);}

    {
      std::lock_guard<std::mutex> lock(report_mutex_);
      report_.plan_ms = elapsed_ms;
      report_.score = debug.score;
      report_.planned = plan.has_value();
      if (plan) {
        report_.start = plan->start;
        report_.gates = plan->gates.size();
        report_.closed = plan->closed;
        report_.stamp = now();
        report_.has_plan = true;
      }
    }

    if (!plan || plan->gates.size() < 2 || !trackChanged(*plan)) {return;}

    mirena_common::msg::Track track;
    track.header = header;
    track.is_closed = plan->closed;
    track.gates.reserve(plan->gates.size());
    for (const Gate & gate : plan->gates) {
      mirena_common::msg::Gate out;
      out.x = gate.p.x;
      out.y = gate.p.y;
      out.psi = gate.psi;
      track.gates.push_back(out);
    }
    track_pub_->publish(std::move(track));

    published_.gates = plan->gates;
    published_.closed = plan->closed;
    published_.valid = true;
  }

  /// True when the new centreline differs from the published one by more than
  /// the planner's own noise.
  bool trackChanged(const Plan & plan) const
  {
    if (!published_.valid || published_.closed != plan.closed) {return true;}
    if (published_.gates.size() != plan.gates.size()) {return true;}
    for (std::size_t i = 0; i < plan.gates.size(); ++i) {
      if (distance(plan.gates[i].p, published_.gates[i].p) > republish_tolerance_) {return true;}
    }
    return false;
  }

  bool debugWanted() const
  {
    return delaunay_pub_ && delaunay_pub_->get_subscription_count() > 0;
  }

  void publishDebug(
    const std_msgs::msg::Header & header, const PlanDebug & debug,
    const std::optional<Plan> & plan)
  {
    Marker clear;
    clear.header = header;
    clear.action = Marker::DELETEALL;

    MarkerArray array;
    array.markers.push_back(std::move(clear));

    Marker edges = makeMarker(header, "delaunay", Marker::LINE_LIST, 0.02, kGrey);
    addSegments(edges, debug.edges);
    array.markers.push_back(std::move(edges));

    Marker gates = makeMarker(header, "gate_edges", Marker::LINE_LIST, 0.05, kGreen);
    addSegments(gates, debug.gate_edges);
    array.markers.push_back(std::move(gates));

    Marker mids = makeMarker(header, "midpoints", Marker::SPHERE_LIST, 0.15, kCyan);
    addPoints(mids, debug.midpoints);
    array.markers.push_back(std::move(mids));

    Marker line = makeMarker(header, "centreline", Marker::LINE_STRIP, 0.10, kMagenta);
    if (plan) {
      for (const Gate & gate : plan->gates) {line.points.push_back(toPoint(gate.p));}
      if (plan->closed && !line.points.empty()) {line.points.push_back(line.points.front());}
    }
    array.markers.push_back(std::move(line));

    Marker start = makeMarker(header, "start_gate", Marker::SPHERE, 0.6, kOrange);
    if (debug.has_start) {
      start.pose.position = toPoint(debug.start);
    } else {
      start.action = Marker::DELETE;
    }
    array.markers.push_back(std::move(start));

    delaunay_pub_->publish(std::move(array));
  }

  void publishHeartbeat()
  {
    if (!report_pub_) {return;}

    Report report;
    {
      std::lock_guard<std::mutex> lock(report_mutex_);
      report = report_;
    }

    nlohmann::json json;
    // Kept as plain numbers (and null before the first plan) so any consumer can
    // read the gate straight out of the heartbeat JSON.
    json["start_point"] = nullptr;
    if (report.start) {
      json["start_point"] = {{"x", report.start->x}, {"y", report.start->y}, {"z", 0.0}};
    }
    json["mission"] = mission_;
    json["gates"] = report.gates;
    json["closed"] = report.closed;
    json["plan_ms"] = report.plan_ms;
    json["planned"] = report.planned;
    json["score"] = report.score;
    json["plan_age_s"] = report.has_plan ?
      (now() - report.stamp).seconds() : std::numeric_limits<double>::quiet_NaN();

    mirena_common::msg::Heartbeat msg;
    msg.node_name = get_name();
    msg.timestamp = now();
    msg.report = json.dump();
    report_pub_->publish(msg);
  }

  // Configuration
  std::string mission_;
  std::string fallback_frame_{"map"};
  double republish_tolerance_{0.25};
  bool publish_debug_{true};

  std::unique_ptr<Planner> planner_;

  rclcpp::CallbackGroup::SharedPtr plan_group_;
  rclcpp::CallbackGroup::SharedPtr report_group_;

  rclcpp::Subscription<mirena_common::msg::EntityList>::SharedPtr map_sub_;
  rclcpp::Publisher<mirena_common::msg::Track>::SharedPtr track_pub_;
  rclcpp::Publisher<mirena_common::msg::Heartbeat>::SharedPtr report_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr delaunay_pub_;

  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::mutex map_mutex_;
  mirena_common::msg::EntityList::SharedPtr pending_map_;

  // Only the planning loop touches this one.
  Published published_;

  std::mutex report_mutex_;
  Report report_;
};

}  // namespace mirena::planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<mirena::planning::UnknownPlannerNode> node;
  try {
    node = std::make_shared<mirena::planning::UnknownPlannerNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mirena_planning"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
