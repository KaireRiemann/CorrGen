#include <fcntl.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <signal.h>
#include <spawn.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
extern char** environ;
using Json = nlohmann::json;
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>

#include "corrgen/voxel_planner.hpp"

class Demo {
 public:
  Demo() : nh_("~") {
    std::vector<double> origin, size;
    if (!nh_.getParam("origin", origin) || !nh_.getParam("size", size) || origin.size() != 3 ||
        size.size() != 3)
      throw std::invalid_argument("origin and size must have 3 coordinates");
    origin_ = {origin[0], origin[1], origin[2]};
    size_ = {size[0], size[1], size[2]};
    nh_.param("resolution", resolution_, .2);
    nh_.param("inflation", inflation_, 0.0);
    nh_.param("robot_radius", robot_radius_, .2);
    nh_.param("safety_margin", safety_margin_, .05);
    nh_.param("reference_clearance", reference_clearance_, .1);
    nh_.param("clearance_weight", clearance_weight_, 4.);
    nh_.param("job_budget_ms", job_budget_ms_, 5000.);
    nh_.param("solver_budget_ms", solver_budget_ms_, 1000.);
    nh_.param("projection_threads", projection_threads_, 4);
    nh_.param("working_set", working_set_, true);
    if (!std::isfinite(clearance_weight_) || clearance_weight_ < 0 ||
        !std::isfinite(robot_radius_) || robot_radius_ < 0 ||
        !std::isfinite(reference_clearance_) || reference_clearance_ < 0 ||
        !std::isfinite(safety_margin_) || safety_margin_ < 0 || !std::isfinite(job_budget_ms_) ||
        job_budget_ms_ <= 0 || !std::isfinite(solver_budget_ms_) || solver_budget_ms_ <= 0)
      throw std::invalid_argument("Invalid body or time budget");
    nh_.param("height", height_, 1.5);
    nh_.param<std::string>("height_mode", height_mode_, "fixed");
    nh_.param<std::string>("frame", frame_, "map");
    nh_.param<std::string>("output_dir", output_, "/tmp/corrgen_interactive");
    nh_.param("degree", options_.degree, 6);
    nh_.param<std::string>("solver", options_.solver, "highs");
    if (height_mode_ != "fixed" && height_mode_ != "arrow")
      throw std::invalid_argument("height_mode must be fixed or arrow");
    session_dir_ =
        std::filesystem::path(output_) /
        ("run_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
         "_" + std::to_string(getpid()));
    std::filesystem::create_directories(session_dir_);
    nh_.setParam("session_dir", session_dir_.string());
    diagnostics_pub_ = nh_.advertise<std_msgs::String>("diagnostics", 1, true);
    timer_ = nh_.createTimer(ros::Duration(.02), &Demo::poll, this);
    status_pub_ = nh_.advertise<std_msgs::String>("status", 1, true);
    astar_pub_ = nh_.advertise<nav_msgs::Path>("astar_path", 1, true);
    reference_pub_ = nh_.advertise<nav_msgs::Path>("reference_path", 1, true);
    corridor_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("corridor", 1, true);
    selections_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("selection", 1, true);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 1, true);
    map_sub_ = nh_.subscribe("cloud", 1, &Demo::map, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &Demo::goal, this);
    initial_sub_ = nh_.subscribe("/initialpose", 1, &Demo::initial, this);
    clicked_sub_ = nh_.subscribe("/clicked_point", 1, &Demo::clicked, this);
    reset_sub_ =
        nh_.subscribe<std_msgs::Empty>("reset", 1, [this](const std_msgs::Empty::ConstPtr&) {
          cancel("CANCELLED");
          selected_.clear();
          clear();
          status(grid_ ? "WAITING_FOR_START" : "WAITING_FOR_MAP");
        });
    status("WAITING_FOR_MAP");
  }

  ~Demo() {
    try {
      cancel("SHUTDOWN");
    } catch (...) {
    }
  }

 private:
  static geometry_msgs::Point point(const Eigen::Vector3d& v) {
    geometry_msgs::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
  }
  std_msgs::Header header() const {
    std_msgs::Header h;
    h.frame_id = frame_;
    h.stamp = ros::Time::now();
    return h;
  }
  void status(const std::string& state) {
    std_msgs::String msg;
    msg.data = state;
    status_pub_.publish(msg);
    ROS_INFO_STREAM(state);
    selection(state);
  }
  void selection(const std::string& state) {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker wipe;
    wipe.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(wipe);
    for (size_t i = 0; i < selected_.size(); ++i) {
      visualization_msgs::Marker m;
      m.header = header();
      m.ns = "selected";
      m.id = i;
      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.position = point(selected_[i]);
      m.pose.orientation.w = 1;
      m.scale.x = m.scale.y = m.scale.z = .25;
      m.color.a = 1;
      m.color.g = i == 0 ? 1 : .1;
      m.color.r = i == 0 ? .1 : 1;
      array.markers.push_back(m);
    }
    visualization_msgs::Marker text;
    text.header = header();
    text.ns = "instructions";
    text.id = 0;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.orientation.w = 1;
    text.pose.position = point(origin_ + Eigen::Vector3d(size_.x() / 2, size_.y(), size_.z() + .4));
    text.scale.z = .3;
    text.color.r = text.color.g = text.color.b = .95;
    text.color.a = 1;
    text.text = state + "\n2D Nav Goal: START, then GOAL. Repeat for a new request.";
    array.markers.push_back(text);
    selections_pub_.publish(array);
  }
  void clear() {
    nav_msgs::Path empty;
    empty.header = header();
    astar_pub_.publish(empty);
    reference_pub_.publish(empty);
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(m);
    corridor_pub_.publish(markers);
  }
  void map(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (last_map_ && msg->data == last_map_->data && msg->width == last_map_->width &&
        msg->height == last_map_->height && msg->header.frame_id == last_map_->header.frame_id)
      return;
    cancel("STALE_MAP");
    ++map_version_;
    try {
      if (msg->header.frame_id != frame_)
        throw std::invalid_argument("Map frame does not match configured frame");
      std::vector<Eigen::Vector3d> cloud;
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        Eigen::Vector3d p(*x, *y, *z);
        if (!p.allFinite())
          throw std::invalid_argument("Nonfinite cloud point");
        cloud.push_back(p);
      }
      if (cloud.empty())
        throw std::invalid_argument("Empty map");
      auto grid = std::make_unique<corrgen::VoxelPlanner>(
          cloud, origin_, size_, resolution_,
          inflation_ + robot_radius_ + safety_margin_ + reference_clearance_);
      grid_ = std::move(grid);
      cloud_ = std::move(cloud);
      last_map_ = msg;
      snapshot_file_ = session_dir_ / ("map_" + std::to_string(map_version_) + ".xyz");
      std::ofstream snapshot(snapshot_file_);
      snapshot << std::setprecision(17);
      for (const auto& p : cloud_) snapshot << p.transpose() << '\n';
      if (!snapshot)
        throw std::runtime_error("Cannot save map snapshot");
      map_hash_ =
          hash(std::string(reinterpret_cast<const char*>(msg->data.data()), msg->data.size()));
      map_pub_.publish(*msg);
      selected_.clear();
      clear();
      status("WAITING_FOR_START");
    } catch (const std::exception& e) {
      grid_.reset();
      last_map_.reset();
      selected_.clear();
      clear();
      status(std::string("MAP_ERROR: ") + e.what());
    }
  }
  bool validFrame(const std::string& frame) {
    if (frame != frame_) {
      status("INVALID_FRAME: select in " + frame_);
      return false;
    }
    return true;
  }
  double height(const geometry_msgs::Quaternion& orientation) const {
    if (height_mode_ == "fixed")
      return height_;
    double norm = std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                            orientation.z * orientation.z + orientation.w * orientation.w);
    if (!std::isfinite(norm) || norm < 1e-8)
      throw std::invalid_argument("Invalid goal orientation");
    double inset = std::ceil((inflation_ + robot_radius_ + safety_margin_ + reference_clearance_) /
                             resolution_) *
                       resolution_ +
                   .5 * resolution_;
    std::max(resolution_, inflation_ + robot_radius_ + safety_margin_ + reference_clearance_);
    return origin_.z() + inset + std::abs(orientation.z / norm) * (size_.z() - 2 * inset);
  }
  void goal(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!validFrame(msg->header.frame_id))
      return;
    try {
      accept({msg->pose.position.x, msg->pose.position.y, height(msg->pose.orientation)}, false);
    } catch (const std::exception& e) {
      status(std::string("INVALID_POINT: ") + e.what());
    }
  }
  void initial(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!validFrame(msg->header.frame_id))
      return;
    try {
      accept({msg->pose.pose.position.x, msg->pose.pose.position.y,
              height(msg->pose.pose.orientation)},
             true);
    } catch (const std::exception& e) {
      status(std::string("INVALID_POINT: ") + e.what());
    }
  }
  void clicked(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (validFrame(msg->header.frame_id))
      accept({msg->point.x, msg->point.y, msg->point.z}, false);
  }
  void accept(const Eigen::Vector3d& p, bool restart) {
    if (!grid_) {
      status("WAITING_FOR_MAP");
      return;
    }
    if (restart || selected_.size() == 2) {
      cancel("SUPERSEDED");
      selected_.clear();
      clear();
    }
    if (!grid_->free(p)) {
      status("INVALID_POINT: occupied or outside map; choose again");
      return;
    }
    if (selected_.size() == 1 && (p - selected_[0]).norm() < 1e-5) {
      status("INVALID_POINT: start equals goal; choose goal again");
      return;
    }
    selected_.push_back(p);
    if (selected_.size() == 1) {
      clear();
      status("WAITING_FOR_GOAL");
      return;
    }
    plan();
  }
  nav_msgs::Path pathMessage(const std::vector<Eigen::Vector3d>& points) const {
    nav_msgs::Path msg;
    msg.header = header();
    for (const auto& p : points) {
      geometry_msgs::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position = point(p);
      pose.pose.orientation.w = 1;
      msg.poses.push_back(pose);
    }
    return msg;
  }
  struct Job {
    pid_t pid = -1;
    int request = 0;
    uint64_t version = 0;
    std::filesystem::path directory;
    std::chrono::steady_clock::time_point started;
    int sequence = 0;
    std::string phase;
    bool verified = false;
    double first_publish_ms = 0;
    Json candidate;
    std::filesystem::file_time_type modified{};
  };
  static Json readJson(const std::filesystem::path& file) {
    std::ifstream f(file);
    Json j;
    f >> j;
    return j;
  }
  static void writeJson(const std::filesystem::path& file, const Json& j) {
    auto tmp = file;
    tmp += ".tmp";
    {
      std::ofstream f(tmp);
      f << j.dump(2) << '\n';
      if (!f)
        throw std::runtime_error("Cannot write job record");
    }
    std::filesystem::rename(tmp, file);
  }
  static std::string hash(const std::string& data) {
    uint64_t h = 14695981039346656037ull;
    for (unsigned char c : data) {
      h ^= c;
      h *= 1099511628211ull;
    }
    std::ostringstream s;
    s << std::hex << h;
    return s.str();
  }
  void cancel(const std::string& reason) {
    if (!job_)
      return;
    kill(-job_->pid, SIGKILL);
    int code;
    while (waitpid(job_->pid, &code, 0) < 0 && errno == EINTR) {
    }
    writeJson(job_->directory / "completion.json",
              {{"status", reason}, {"request", job_->request}, {"map_version", job_->version}});
    job_.reset();
  }
  std::vector<Eigen::Vector3d> readPoints(const std::filesystem::path& file) {
    std::ifstream f(file);
    double x, y, z;
    std::vector<Eigen::Vector3d> points;
    while (f >> x >> y >> z) points.emplace_back(x, y, z);
    return points;
  }
  void publishCandidate(const Json& result) {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker wipe;
    wipe.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(wipe);
    int id = 0;
    for (const auto& points : result.at("rings")) {
      visualization_msgs::Marker ring;
      ring.header = header();
      ring.ns = "corridor";
      ring.id = id++;
      ring.type = visualization_msgs::Marker::LINE_STRIP;
      ring.action = visualization_msgs::Marker::ADD;
      ring.pose.orientation.w = 1;
      ring.scale.x = .018;
      ring.color.r = .9;
      ring.color.g = .15;
      ring.color.b = .8;
      ring.color.a = .65;
      for (const auto& p : points)
        ring.points.push_back(
            point({p.at(0).get<double>(), p.at(1).get<double>(), p.at(2).get<double>()}));
      array.markers.push_back(ring);
    }
    astar_pub_.publish(pathMessage(readPoints(job_->directory / "astar.xyz")));
    reference_pub_.publish(pathMessage(readPoints(job_->directory / "reference.xyz")));
    corridor_pub_.publish(array);
    Json diagnostic = result;
    diagnostic.erase("rings");
    std_msgs::String msg;
    msg.data = diagnostic.dump();
    diagnostics_pub_.publish(msg);
  }
  void consumeCandidate() {
    auto file = job_->directory / "candidate.json";
    if (std::filesystem::exists(file)) {
      auto modified = std::filesystem::last_write_time(file);
      if (modified != job_->modified) {
        auto candidate = readJson(file);
        if (candidate.at("request").get<int>() != job_->request ||
            candidate.at("map_version").get<uint64_t>() != map_version_ ||
            candidate.at("validation") != "INFLATED_VOXEL_ENCLOSURE")
          throw std::runtime_error("Stale or unverified worker output");
        if (candidate.at("sequence").get<int>() > job_->sequence) {
          publishCandidate(candidate);
          if (!job_->verified)
            job_->first_publish_ms = std::chrono::duration<double, std::milli>(
                                         std::chrono::steady_clock::now() - job_->started)
                                         .count();
          job_->verified = true;
          job_->sequence = candidate.at("sequence");
          job_->candidate = candidate;
          status("FEASIBLE_READY request=" + std::to_string(job_->request) +
                 "; optimizing within budget");
        }
        job_->modified = modified;
      }
    }
  }
  void poll(const ros::TimerEvent&) {
    if (!job_)
      return;
    try {
      if (job_->version != map_version_) {
        cancel("STALE_MAP");
        clear();
        return;
      }
      consumeCandidate();
      auto progress = job_->directory / "progress.json";
      if (!job_->verified && std::filesystem::exists(progress)) {
        std::string phase = readJson(progress).at("phase");
        if (phase != job_->phase) {
          job_->phase = phase;
          status(phase + " request=" + std::to_string(job_->request));
        }
      }
      int code = 0;
      struct rusage usage{};
      pid_t ended = wait4(job_->pid, &code, WNOHANG, &usage);
      double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                                 job_->started)
                           .count();
      bool timeout = elapsed >= job_budget_ms_;
      if (!ended && !timeout)
        return;
      if (!ended) {
        kill(-job_->pid, SIGKILL);
        while (wait4(job_->pid, &code, 0, &usage) < 0 && errno == EINTR) {
        }
      }
      // The worker may finish between the first read and wait4. Read its final
      // atomic result before persisting metrics, even when an incumbent exists.
      consumeCandidate();
      elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                          job_->started)
                    .count();
      Json completion = {{"request", job_->request},
                         {"map_version", job_->version},
                         {"wall_ms", elapsed},
                         {"worker_peak_rss_kib", usage.ru_maxrss},
                         {"first_publish_ms", job_->first_publish_ms},
                         {"timeout", timeout},
                         {"wait_status", code}};
      if (job_->verified) {
        const std::string final_quality =
            timeout ? "FEASIBLE_BUDGET"
                    : (code ? "FEASIBLE_WORKER_FAILURE"
                            : job_->candidate.value("quality", std::string("FEASIBLE")));
        completion["status"] = final_quality;
        auto metrics = job_->candidate;
        metrics.erase("rings");
        metrics["status"] = "PASS";
        metrics["completion"] = completion;
        writeJson(job_->directory / "metrics.json", metrics);
        status("PASS request=" + std::to_string(job_->request) + " " + final_quality +
               "; voxel enclosure verified");
      } else {
        clear();
        std::string failure = timeout ? "TIMEOUT" : "WORKER_FAILED";
        if (std::filesystem::exists(job_->directory / "failure.json"))
          failure = readJson(job_->directory / "failure.json").value("error", failure);
        completion["status"] = failure;
        status("FAIL request=" + std::to_string(job_->request) + ": " + failure);
      }
      writeJson(job_->directory / "completion.json", completion);
      job_.reset();
    } catch (const std::exception& e) {
      cancel("INVALID_WORKER_OUTPUT");
      clear();
      status(std::string("FAIL: ") + e.what());
    }
  }
  void plan() {
    cancel("SUPERSEDED");
    ++request_;
    clear();
    try {
      auto directory = session_dir_ / ("request_" + std::to_string(request_));
      std::filesystem::create_directories(directory);
      Json config = {{"origin", {origin_.x(), origin_.y(), origin_.z()}},
                     {"size", {size_.x(), size_.y(), size_.z()}},
                     {"resolution", resolution_},
                     {"inflation", inflation_},
                     {"reference_clearance", reference_clearance_},
                     {"clearance_weight", clearance_weight_},
                     {"robot_radius", robot_radius_},
                     {"safety_margin", safety_margin_},
                     {"degree", options_.degree},
                     {"solver", options_.solver},
                     {"working_set", working_set_},
                     {"solver_budget_ms", solver_budget_ms_},
                     {"job_budget_ms", job_budget_ms_},
                     {"projection_threads", projection_threads_}};
      config["config_hash"] = hash(config.dump());
      config["request"] = request_;
      config["map_version"] = map_version_;
      config["map_hash"] = map_hash_;
      config["cloud_file"] = snapshot_file_.string();
      config["start"] = {selected_[0].x(), selected_[0].y(), selected_[0].z()};
      config["goal"] = {selected_[1].x(), selected_[1].y(), selected_[1].z()};
      writeJson(directory / "request.json", config);
      auto executable =
          (std::filesystem::canonical("/proc/self/exe").parent_path() / "corrgen_worker").string();
      std::string argument = directory.string();
      char* args[] = {const_cast<char*>(executable.c_str()), const_cast<char*>(argument.c_str()),
                      nullptr};
      posix_spawnattr_t attr;
      posix_spawnattr_init(&attr);
      posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
      posix_spawnattr_setpgroup(&attr, 0);
      posix_spawn_file_actions_t actions;
      posix_spawn_file_actions_init(&actions);
      auto log = (directory / "worker.log").string();
      posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, log.c_str(),
                                       O_WRONLY | O_CREAT | O_TRUNC, 0644);
      posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
      pid_t pid;
      int error = posix_spawn(&pid, executable.c_str(), &actions, &attr, args, environ);
      posix_spawn_file_actions_destroy(&actions);
      posix_spawnattr_destroy(&attr);
      if (error)
        throw std::runtime_error("Cannot spawn worker: " + std::string(strerror(error)));
      job_ = Job{pid, request_, map_version_, directory, std::chrono::steady_clock::now()};
      status("QUEUED request=" + std::to_string(request_));
    } catch (const std::exception& e) {
      status(std::string("FAIL: ") + e.what());
    }
  }
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_, goal_sub_, initial_sub_, clicked_sub_, reset_sub_;
  ros::Publisher status_pub_, astar_pub_, reference_pub_, corridor_pub_, selections_pub_, map_pub_;
  sensor_msgs::PointCloud2::ConstPtr last_map_;
  std::unique_ptr<corrgen::VoxelPlanner> grid_;
  std::vector<Eigen::Vector3d> cloud_, selected_;
  Eigen::Vector3d origin_, size_;
  double resolution_, inflation_, height_;
  std::string frame_, height_mode_, output_;
  corrgen::Options options_;
  int request_ = 0;
  double clearance_weight_, robot_radius_, safety_margin_, reference_clearance_, job_budget_ms_,
      solver_budget_ms_;
  int projection_threads_;
  bool working_set_;
  uint64_t map_version_ = 0;
  std::string map_hash_;
  std::filesystem::path session_dir_, snapshot_file_;
  ros::Timer timer_;
  ros::Publisher diagnostics_pub_;
  std::optional<Job> job_;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "corrgen_demo");
  try {
    Demo demo;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM(e.what());
    return 1;
  }
  return 0;
}
