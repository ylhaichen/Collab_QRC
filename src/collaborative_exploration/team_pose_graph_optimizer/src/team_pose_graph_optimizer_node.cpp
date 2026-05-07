#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#ifndef HAVE_GTSAM_CPP
#define HAVE_GTSAM_CPP 0
#endif

#if HAVE_GTSAM_CPP
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#endif

using namespace std::chrono_literals;

namespace {

struct GraphKey {
  std::string robot;
  int keyframe_id{0};

  bool operator<(const GraphKey & other) const
  {
    if (robot != other.robot) {
      return robot < other.robot;
    }
    return keyframe_id < other.keyframe_id;
  }

  bool valid() const { return !robot.empty() && keyframe_id >= 0; }
};

struct PoseRecord {
  GraphKey key;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double stamp_sec{0.0};
};

struct FactorRecord {
  std::string factor_type;
  GraphKey key1;
  std::optional<GraphKey> key2;
  double dx{0.0};
  double dy{0.0};
  double dyaw{0.0};
  double noise_scale{1.0};
  std::string source;
  std::string match_id;
};

struct ParsedGraph {
  bool ok{false};
  bool gt_used_runtime{false};
  std::string error;
  int raw_inter_robot_matches{0};
  int rejected_inter_robot_matches{0};
  double alignment_confidence{0.0};
  std::map<GraphKey, PoseRecord> poses;
  std::vector<FactorRecord> factors;
};

double wrap_pi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

std::string escape_json(const std::string & in)
{
  std::ostringstream out;
  for (char c : in) {
    switch (c) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}

std::string json_number_or_null(std::optional<double> value)
{
  if (!value.has_value() || !std::isfinite(value.value())) {
    return "null";
  }
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(6);
  out << value.value();
  return out.str();
}

bool regex_search_value(const std::string & text, const std::regex & pattern, std::smatch & match)
{
  return std::regex_search(text, match, pattern);
}

std::optional<std::string> string_field(const std::string & object, const std::string & name)
{
  std::smatch match;
  const std::regex pattern("\"" + name + "\"\\s*:\\s*\"([^\"]*)\"");
  if (regex_search_value(object, pattern, match)) {
    return match[1].str();
  }
  return std::nullopt;
}

std::optional<double> double_field(const std::string & object, const std::string & name)
{
  std::smatch match;
  const std::regex pattern(
    "\"" + name + "\"\\s*:\\s*([-+]?\\d+(?:\\.\\d+)?(?:[eE][-+]?\\d+)?)");
  if (regex_search_value(object, pattern, match)) {
    try {
      return std::stod(match[1].str());
    } catch (...) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

std::optional<int> int_field(const std::string & object, const std::string & name)
{
  std::smatch match;
  const std::regex pattern("\"" + name + "\"\\s*:\\s*(-?\\d+)");
  if (regex_search_value(object, pattern, match)) {
    try {
      return std::stoi(match[1].str());
    } catch (...) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

bool bool_field(const std::string & object, const std::string & name, bool default_value)
{
  std::smatch match;
  const std::regex pattern("\"" + name + "\"\\s*:\\s*(true|false)");
  if (regex_search_value(object, pattern, match)) {
    return match[1].str() == "true";
  }
  return default_value;
}

std::optional<GraphKey> key_field(const std::string & object, const std::string & name)
{
  if (object.find("\"" + name + "\"") == std::string::npos) {
    return std::nullopt;
  }
  const std::regex null_pattern("\"" + name + "\"\\s*:\\s*null");
  if (std::regex_search(object, null_pattern)) {
    return std::nullopt;
  }
  std::smatch match;
  const std::regex pattern(
    "\"" + name + "\"\\s*:\\s*\\[\\s*\"([^\"]+)\"\\s*,\\s*(-?\\d+)\\s*\\]");
  if (!regex_search_value(object, pattern, match)) {
    return std::nullopt;
  }
  try {
    return GraphKey{match[1].str(), std::stoi(match[2].str())};
  } catch (...) {
    return std::nullopt;
  }
}

bool measurement_field(const std::string & object, double & dx, double & dy, double & dyaw)
{
  std::smatch match;
  const std::regex pattern(
    "\"measurement\"\\s*:\\s*\\[\\s*"
    "([-+]?\\d+(?:\\.\\d+)?(?:[eE][-+]?\\d+)?)\\s*,\\s*"
    "([-+]?\\d+(?:\\.\\d+)?(?:[eE][-+]?\\d+)?)\\s*,\\s*"
    "([-+]?\\d+(?:\\.\\d+)?(?:[eE][-+]?\\d+)?)\\s*\\]");
  if (!regex_search_value(object, pattern, match)) {
    return false;
  }
  try {
    dx = std::stod(match[1].str());
    dy = std::stod(match[2].str());
    dyaw = std::stod(match[3].str());
  } catch (...) {
    return false;
  }
  return true;
}

std::optional<std::string> extract_array_block(const std::string & text, const std::string & name)
{
  const std::string key = "\"" + name + "\"";
  const std::size_t key_pos = text.find(key);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  const std::size_t start = text.find('[', key_pos);
  if (start == std::string::npos) {
    return std::nullopt;
  }
  bool in_string = false;
  bool escaped = false;
  int depth = 0;
  for (std::size_t i = start; i < text.size(); ++i) {
    const char c = text[i];
    if (escaped) {
      escaped = false;
      continue;
    }
    if (c == '\\') {
      escaped = in_string;
      continue;
    }
    if (c == '"') {
      in_string = !in_string;
      continue;
    }
    if (in_string) {
      continue;
    }
    if (c == '[') {
      ++depth;
    } else if (c == ']') {
      --depth;
      if (depth == 0) {
        return text.substr(start, i - start + 1);
      }
    }
  }
  return std::nullopt;
}

std::vector<std::string> split_json_objects(const std::string & array_block)
{
  std::vector<std::string> objects;
  bool in_string = false;
  bool escaped = false;
  int depth = 0;
  std::size_t start = std::string::npos;
  for (std::size_t i = 0; i < array_block.size(); ++i) {
    const char c = array_block[i];
    if (escaped) {
      escaped = false;
      continue;
    }
    if (c == '\\') {
      escaped = in_string;
      continue;
    }
    if (c == '"') {
      in_string = !in_string;
      continue;
    }
    if (in_string) {
      continue;
    }
    if (c == '{') {
      if (depth == 0) {
        start = i;
      }
      ++depth;
    } else if (c == '}') {
      --depth;
      if (depth == 0 && start != std::string::npos) {
        objects.push_back(array_block.substr(start, i - start + 1));
        start = std::string::npos;
      }
    }
  }
  return objects;
}

ParsedGraph parse_factor_payload(const std::string & text)
{
  ParsedGraph graph;
  if (text.empty()) {
    graph.error = "empty_factor_payload";
    return graph;
  }
  if (text.find("\"schema\"") == std::string::npos ||
    text.find("team_pose_graph_factors/v1") == std::string::npos)
  {
    graph.error = "unsupported_factor_payload_schema";
    return graph;
  }
  graph.gt_used_runtime = bool_field(text, "gt_used_runtime", false);
  graph.raw_inter_robot_matches = int_field(text, "raw_inter_robot_matches").value_or(0);
  graph.rejected_inter_robot_matches = int_field(text, "rejected_inter_robot_matches").value_or(0);
  graph.alignment_confidence = double_field(text, "alignment_confidence").value_or(0.0);

  const auto poses_block = extract_array_block(text, "poses");
  const auto factors_block = extract_array_block(text, "factors");
  if (!poses_block.has_value() || !factors_block.has_value()) {
    graph.error = "missing_poses_or_factors";
    return graph;
  }

  for (const std::string & object : split_json_objects(poses_block.value())) {
    const auto robot = string_field(object, "robot_id");
    const auto keyframe_id = int_field(object, "keyframe_id");
    if (!robot.has_value() || !keyframe_id.has_value()) {
      continue;
    }
    PoseRecord pose;
    pose.key = GraphKey{robot.value(), keyframe_id.value()};
    pose.x = double_field(object, "x").value_or(0.0);
    pose.y = double_field(object, "y").value_or(0.0);
    pose.yaw = double_field(object, "yaw").value_or(0.0);
    pose.stamp_sec = double_field(object, "stamp_sec").value_or(0.0);
    if (pose.key.valid()) {
      graph.poses[pose.key] = pose;
    }
  }

  for (const std::string & object : split_json_objects(factors_block.value())) {
    FactorRecord factor;
    factor.factor_type = string_field(object, "factor_type").value_or("");
    const auto key1 = key_field(object, "key1");
    if (!key1.has_value()) {
      continue;
    }
    factor.key1 = key1.value();
    factor.key2 = key_field(object, "key2");
    if (!measurement_field(object, factor.dx, factor.dy, factor.dyaw)) {
      continue;
    }
    factor.noise_scale = double_field(object, "noise_scale").value_or(1.0);
    factor.source = string_field(object, "source").value_or("");
    factor.match_id = string_field(object, "match_id").value_or("");
    if (factor.factor_type == "inter_robot" && factor.source != "robust") {
      continue;
    }
    if (factor.factor_type == "odom" || factor.factor_type == "inter_robot") {
      if (!factor.key2.has_value()) {
        continue;
      }
      if (graph.poses.find(factor.key1) == graph.poses.end() ||
        graph.poses.find(factor.key2.value()) == graph.poses.end())
      {
        continue;
      }
    }
    if (factor.factor_type == "prior" && graph.poses.find(factor.key1) == graph.poses.end()) {
      continue;
    }
    graph.factors.push_back(factor);
  }

  graph.ok = !graph.poses.empty();
  if (!graph.ok) {
    graph.error = "no_valid_graph_poses";
  }
  return graph;
}

struct GraphMetrics {
  std::string backend{"g2o_export_only"};
  bool success{false};
  std::string dependency_blocker;
  std::string latest_optimization_error;
  std::optional<double> error_before;
  std::optional<double> error_after;
  int num_keyframes_robot_a{0};
  int num_keyframes_robot_b{0};
  int num_odom_factors{0};
  int num_prior_factors{0};
  int num_inter_robot_factors_raw{0};
  int num_inter_robot_factors_inlier{0};
  int num_inter_robot_factors_rejected{0};
  int pose_graph_num_factors{0};
  double alignment_confidence{0.0};
  bool gt_used_runtime{false};
};

GraphMetrics base_metrics(const ParsedGraph & parsed)
{
  GraphMetrics metrics;
  metrics.num_inter_robot_factors_raw = parsed.raw_inter_robot_matches;
  metrics.num_inter_robot_factors_rejected = parsed.rejected_inter_robot_matches;
  metrics.alignment_confidence = parsed.alignment_confidence;
  metrics.gt_used_runtime = parsed.gt_used_runtime;
  for (const auto & [key, _pose] : parsed.poses) {
    if (key.robot == "robot_a") {
      ++metrics.num_keyframes_robot_a;
    } else if (key.robot == "robot_b") {
      ++metrics.num_keyframes_robot_b;
    }
  }
  for (const FactorRecord & factor : parsed.factors) {
    if (factor.factor_type == "prior") {
      ++metrics.num_prior_factors;
    } else if (factor.factor_type == "odom") {
      ++metrics.num_odom_factors;
    } else if (factor.factor_type == "inter_robot") {
      ++metrics.num_inter_robot_factors_inlier;
    }
  }
  metrics.pose_graph_num_factors = static_cast<int>(parsed.factors.size());
  return metrics;
}

std::string metrics_to_json(const GraphMetrics & metrics)
{
  std::ostringstream out;
  out << "{"
      << "\"schema\":\"team_pose_graph_metrics/v1\","
      << "\"optimization_backend\":\"" << escape_json(metrics.backend) << "\","
      << "\"optimization_success\":" << (metrics.success ? "true" : "false") << ","
      << "\"dependency_blocker\":\"" << escape_json(metrics.dependency_blocker) << "\","
      << "\"num_keyframes_robot_a\":" << metrics.num_keyframes_robot_a << ","
      << "\"num_keyframes_robot_b\":" << metrics.num_keyframes_robot_b << ","
      << "\"num_odom_factors\":" << metrics.num_odom_factors << ","
      << "\"num_prior_factors\":" << metrics.num_prior_factors << ","
      << "\"num_inter_robot_factors_raw\":" << metrics.num_inter_robot_factors_raw << ","
      << "\"num_inter_robot_factors_inlier\":" << metrics.num_inter_robot_factors_inlier << ","
      << "\"num_inter_robot_factors_rejected\":" << metrics.num_inter_robot_factors_rejected << ","
      << "\"pose_graph_num_factors\":" << metrics.pose_graph_num_factors << ","
      << "\"pose_graph_inter_robot_factors\":" << metrics.num_inter_robot_factors_inlier << ","
      << "\"alignment_confidence\":" << json_number_or_null(metrics.alignment_confidence) << ","
      << "\"self_loop_events_available\":false,"
      << "\"gt_used_runtime\":" << (metrics.gt_used_runtime ? "true" : "false") << ","
      << "\"pose_graph_error_before\":" << json_number_or_null(metrics.error_before) << ","
      << "\"pose_graph_error_after\":" << json_number_or_null(metrics.error_after) << ","
      << "\"latest_optimization_error\":\"" << escape_json(metrics.latest_optimization_error) << "\""
      << "}";
  return out.str();
}

#if HAVE_GTSAM_CPP
gtsam::Key symbol_for(const GraphKey & key)
{
  const char prefix = key.robot == "robot_a" ? 'a' : 'b';
  return gtsam::Symbol(prefix, static_cast<std::uint64_t>(key.keyframe_id));
}

gtsam::SharedNoiseModel diagonal_noise(double sx, double sy, double syaw)
{
  return gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(sx, sy, syaw));
}

GraphMetrics optimize_with_gtsam(const ParsedGraph & parsed)
{
  GraphMetrics metrics = base_metrics(parsed);
  metrics.backend = "gtsam_cpp";
  if (!parsed.ok) {
    metrics.latest_optimization_error = parsed.error.empty() ? "invalid_factor_payload" : parsed.error;
    return metrics;
  }
  if (parsed.gt_used_runtime) {
    metrics.latest_optimization_error = "gt_runtime_factor_payload_rejected";
    return metrics;
  }
  if (metrics.num_inter_robot_factors_inlier <= 0) {
    metrics.latest_optimization_error = "no_inter_robot_factors";
    return metrics;
  }

  try {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    for (const auto & [key, pose] : parsed.poses) {
      initial.insert(symbol_for(key), gtsam::Pose2(pose.x, pose.y, pose.yaw));
    }

    bool has_prior = false;
    for (const FactorRecord & factor : parsed.factors) {
      const gtsam::Pose2 measurement(factor.dx, factor.dy, wrap_pi(factor.dyaw));
      if (factor.factor_type == "prior") {
        graph.add(gtsam::PriorFactor<gtsam::Pose2>(
          symbol_for(factor.key1), measurement, diagonal_noise(0.05, 0.05, 0.03)));
        has_prior = true;
      } else if (factor.factor_type == "odom" && factor.key2.has_value()) {
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
          symbol_for(factor.key1),
          symbol_for(factor.key2.value()),
          measurement,
          diagonal_noise(0.25, 0.25, 0.15)));
      } else if (factor.factor_type == "inter_robot" && factor.key2.has_value()) {
        const double rmse_scale = std::clamp(std::abs(factor.noise_scale), 0.10, 1.50);
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
          symbol_for(factor.key1),
          symbol_for(factor.key2.value()),
          measurement,
          diagonal_noise(rmse_scale, rmse_scale, std::max(0.08, rmse_scale * 0.50))));
      }
    }
    if (!has_prior && !parsed.poses.empty()) {
      const auto & [key, pose] = *parsed.poses.begin();
      graph.add(gtsam::PriorFactor<gtsam::Pose2>(
        symbol_for(key), gtsam::Pose2(pose.x, pose.y, pose.yaw),
        diagonal_noise(0.05, 0.05, 0.03)));
      ++metrics.num_prior_factors;
      ++metrics.pose_graph_num_factors;
    }

    if (graph.empty()) {
      metrics.latest_optimization_error = "empty_gtsam_graph";
      return metrics;
    }

    const double error_before = graph.error(initial);
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(50);
    params.setRelativeErrorTol(1e-5);
    params.setAbsoluteErrorTol(1e-5);
    const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
    const double error_after = graph.error(result);
    metrics.error_before = error_before;
    metrics.error_after = error_after;
    metrics.latest_optimization_error = std::to_string(error_after);
    metrics.success = std::isfinite(error_before) && std::isfinite(error_after) &&
      error_after <= error_before + 1e-6;
    if (!metrics.success) {
      metrics.latest_optimization_error = "gtsam_error_not_reduced";
    }
  } catch (const std::exception & exc) {
    metrics.latest_optimization_error = std::string("gtsam_cpp_exception:") + exc.what();
    metrics.success = false;
  } catch (...) {
    metrics.latest_optimization_error = "gtsam_cpp_unknown_exception";
    metrics.success = false;
  }
  return metrics;
}
#endif

GraphMetrics fallback_metrics(const ParsedGraph & parsed, const std::string & blocker)
{
  GraphMetrics metrics = base_metrics(parsed);
  metrics.backend = "g2o_export_only";
  metrics.success = false;
  metrics.dependency_blocker = blocker;
  metrics.latest_optimization_error = parsed.error;
  return metrics;
}

std::optional<std::string> read_text_file(const std::string & path)
{
  if (path.empty()) {
    return std::nullopt;
  }
  std::ifstream in(path);
  if (!in.good()) {
    return std::nullopt;
  }
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

void write_text_file(const std::string & path, const std::string & text)
{
  if (path.empty()) {
    return;
  }
  const std::filesystem::path out_path(path);
  if (out_path.has_parent_path()) {
    std::filesystem::create_directories(out_path.parent_path());
  }
  std::ofstream out(path);
  out << text << "\n";
}

}  // namespace

class TeamPoseGraphOptimizerNode : public rclcpp::Node {
public:
  TeamPoseGraphOptimizerNode()
  : Node("team_pose_graph_optimizer_node")
  {
    factors_topic_ = declare_parameter<std::string>(
      "factors_topic", "/team_slam/team_pose_graph_factors");
    metrics_topic_ = declare_parameter<std::string>(
      "metrics_topic", "/team_slam/pose_graph_metrics");
    factors_json_path_ = declare_parameter<std::string>(
      "factors_json_path", "logs/team_pose_graph_factors.json");
    metrics_path_ = declare_parameter<std::string>(
      "metrics_path", "logs/team_pose_graph_metrics.json");
    publish_period_sec_ = declare_parameter<double>("publish_period_sec", 2.0);

    metrics_pub_ = create_publisher<std_msgs::msg::String>(metrics_topic_, 10);
    factors_sub_ = create_subscription<std_msgs::msg::String>(
      factors_topic_, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        latest_factors_ = msg->data;
        publish_metrics();
      });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.5, publish_period_sec_))),
      [this]() { publish_metrics(); });
#if HAVE_GTSAM_CPP
    RCLCPP_INFO(get_logger(), "team_pose_graph_optimizer_node built with GTSAM C++ backend");
#else
    RCLCPP_WARN(
      get_logger(),
      "team_pose_graph_optimizer_node built without GTSAM; publishing explicit g2o_export_only blocker");
#endif
  }

private:
  std::string current_factor_payload()
  {
    if (!latest_factors_.empty()) {
      return latest_factors_;
    }
    const auto from_file = read_text_file(factors_json_path_);
    if (from_file.has_value()) {
      latest_factors_ = from_file.value();
    }
    return latest_factors_;
  }

  void publish_metrics()
  {
    const ParsedGraph parsed = parse_factor_payload(current_factor_payload());
#if HAVE_GTSAM_CPP
    GraphMetrics metrics = optimize_with_gtsam(parsed);
#else
    GraphMetrics metrics = fallback_metrics(parsed, "gtsam_cpp_not_found");
#endif
    std_msgs::msg::String msg;
    msg.data = metrics_to_json(metrics);
    write_text_file(metrics_path_, msg.data);
    metrics_pub_->publish(msg);
  }

  std::string factors_topic_;
  std::string metrics_topic_;
  std::string factors_json_path_;
  std::string metrics_path_;
  double publish_period_sec_{2.0};
  std::string latest_factors_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr factors_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeamPoseGraphOptimizerNode>());
  rclcpp::shutdown();
  return 0;
}
