#include <iostream>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>

// Motion Capture
#include <libmotioncapture/motioncapture.h>

// Rigid Body tracker
#include <librigidbodytracker/rigid_body_tracker.h>
#include <librigidbodytracker/cloudlog.hpp>

void logWarn(rclcpp::Logger logger, const std::string& msg)
{
  RCLCPP_WARN(logger, "%s", msg.c_str());
}

std::set<std::string> extract_names(
  const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
  const std::string& pattern)
{
  std::set<std::string> result;
  for (const auto &i : parameter_overrides)
  {
    if (i.first.find(pattern) == 0)
    {
      size_t start = pattern.size() + 1;
      size_t end = i.first.find(".", start);
      result.insert(i.first.substr(start, end - start));
    }
  }
  return result;
}

std::vector<double> get_vec(const rclcpp::ParameterValue& param_value)
{
  if (param_value.get_type() == rclcpp::PARAMETER_INTEGER_ARRAY) {
    const auto int_vec = param_value.get<std::vector<int64_t>>();
    std::vector<double> result;
    for (int v : int_vec) {
      result.push_back(v);
    }
    return result;
  }
  return param_value.get<std::vector<double>>();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motion_capture_tracking_node");
  node->declare_parameter<std::string>("type", "vicon");
  node->declare_parameter<std::string>("hostname", "localhost");
  node->declare_parameter<std::string>("topics.poses.qos.mode", "none");
  node->declare_parameter<double>("topics.poses.qos.deadline", 100.0);
  node->declare_parameter<std::string>("logfilepath", "");

  std::string motionCaptureType = node->get_parameter("type").as_string();
  std::string motionCaptureHostname = node->get_parameter("hostname").as_string();
  std::string poses_qos = node->get_parameter("topics.poses.qos.mode").as_string();
  double poses_deadline = node->get_parameter("topics.poses.qos.deadline").as_double();
  std::string logFilePath = node->get_parameter("logfilepath").as_string();

  librigidbodytracker::PointCloudLogger pointCloudLogger(logFilePath);
  const bool logClouds = !logFilePath.empty();
  std::cout << "logClouds=" <<logClouds << std::endl;  // 1

  // Make a new client
  std::map<std::string, std::string> cfg;
  cfg["hostname"] = motionCaptureHostname;
  libmotioncapture::MotionCapture *mocap = libmotioncapture::MotionCapture::connect(motionCaptureType, cfg);

  // prepare point cloud publisher
  auto pubPointCloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("pointCloud", 1);

  sensor_msgs::msg::PointCloud2 msgPointCloud;
  msgPointCloud.header.frame_id = "world";
  msgPointCloud.height = 1;

  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  msgPointCloud.fields.push_back(field);
  field.name = "y";
  field.offset = 4;
  msgPointCloud.fields.push_back(field);
  field.name = "z";
  field.offset = 8;
  msgPointCloud.fields.push_back(field);
  msgPointCloud.point_step = 12;
  msgPointCloud.is_bigendian = false;
  msgPointCloud.is_dense = true;

  // prepare pose array publisher
  rclcpp::Publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr pubPoses;
  if (poses_qos == "none") {
    pubPoses = node->create_publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>("poses", 1);
  } else if (poses_qos == "sensor") {
    rclcpp::SensorDataQoS sensor_data_qos;
    sensor_data_qos.keep_last(1);
    sensor_data_qos.deadline(rclcpp::Duration(0/*s*/, (int)1e9/poses_deadline /*ns*/));
    pubPoses = node->create_publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>("poses", sensor_data_qos);
  } else {
    throw std::runtime_error("Unknown QoS mode! " + poses_qos);
  }

  motion_capture_tracking_interfaces::msg::NamedPoseArray msgPoses;
  msgPoses.header.frame_id = "world";

  // prepare rigid body tracker

  auto node_parameters_iface = node->get_node_parameters_interface();
  const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
      node_parameters_iface->get_parameter_overrides();


  auto dynamics_config_names = extract_names(parameter_overrides, "dynamics_configurations");
  std::vector<librigidbodytracker::DynamicsConfiguration> dynamicsConfigurations(dynamics_config_names.size());
  std::map<std::string, size_t> dynamics_name_to_index;
  size_t i = 0;
  for (const auto& name : dynamics_config_names) {
    const auto max_vel = get_vec(parameter_overrides.at("dynamics_configurations." + name + ".max_velocity"));
    dynamicsConfigurations[i].maxXVelocity = max_vel.at(0);
    dynamicsConfigurations[i].maxYVelocity = max_vel.at(1);
    dynamicsConfigurations[i].maxZVelocity = max_vel.at(2);
    const auto max_angular_velocity = get_vec(parameter_overrides.at("dynamics_configurations." + name + ".max_angular_velocity"));
    dynamicsConfigurations[i].maxRollRate = max_angular_velocity.at(0);
    dynamicsConfigurations[i].maxPitchRate = max_angular_velocity.at(1);
    dynamicsConfigurations[i].maxYawRate = max_angular_velocity.at(2);
    dynamicsConfigurations[i].maxRoll = parameter_overrides.at("dynamics_configurations." + name + ".max_roll").get<double>();
    dynamicsConfigurations[i].maxPitch = parameter_overrides.at("dynamics_configurations." + name + ".max_pitch").get<double>();
    dynamicsConfigurations[i].maxFitnessScore = parameter_overrides.at("dynamics_configurations." + name + ".max_fitness_score").get<double>();
    dynamics_name_to_index[name] = i;
    ++i;
  }

  auto marker_config_names = extract_names(parameter_overrides, "marker_configurations");
  std::vector<librigidbodytracker::MarkerConfiguration> markerConfigurations;
  std::map<std::string, size_t> marker_name_to_index;
  i = 0;
  for (const auto &name : marker_config_names)
  {
    markerConfigurations.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
    const auto offset = get_vec(parameter_overrides.at("marker_configurations." + name + ".offset"));
    for (const auto &param : parameter_overrides)
    {
      if (param.first.find("marker_configurations." + name + ".points") == 0)
      {
        const auto points = get_vec(param.second);
        markerConfigurations.back()->push_back(pcl::PointXYZ(points[0] + offset[0], points[1] + offset[1], points[2] + offset[2]));
      }
    }
    marker_name_to_index[name] = i;
    ++i;
  }

  auto rigid_body_names = extract_names(parameter_overrides, "rigid_bodies");
  std::vector<librigidbodytracker::RigidBody> rigidBodies;
  for (const auto &name : rigid_body_names)
  {
    const auto pos = get_vec(parameter_overrides.at("rigid_bodies." + name + ".initial_position"));
    Eigen::Affine3f m;
    m = Eigen::Translation3f(pos[0], pos[1], pos[2]);
    const auto marker = parameter_overrides.at("rigid_bodies." + name + ".marker").get<std::string>();
    const auto dynamics = parameter_overrides.at("rigid_bodies." + name + ".dynamics").get<std::string>();

    rigidBodies.push_back(librigidbodytracker::RigidBody(marker_name_to_index.at(marker), dynamics_name_to_index.at(dynamics), m, name));
  }

  librigidbodytracker::RigidBodyTracker tracker(
      dynamicsConfigurations,
      markerConfigurations,
      rigidBodies);
  tracker.setLogWarningCallback(std::bind(logWarn, node->get_logger(), std::placeholders::_1));

  // prepare TF broadcaster
  tf2_ros::TransformBroadcaster tfbroadcaster(node);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t frameId = 0; rclcpp::ok(); ++frameId) {

    // Get a frame
    mocap->waitForNextFrame();
    auto chrono_now = std::chrono::high_resolution_clock::now();
    auto time = node->now();

    auto pointcloud = mocap->pointCloud();

    // publish as pointcloud
    msgPointCloud.header.stamp = time;
    msgPointCloud.width = pointcloud.rows();
    msgPointCloud.data.resize(pointcloud.rows() * 3 * 4); // width * height * pointstep
    memcpy(msgPointCloud.data.data(), pointcloud.data(), msgPointCloud.data.size());
    msgPointCloud.row_step = msgPointCloud.data.size();

    pubPointCloud->publish(msgPointCloud);
    if (logClouds) {
      // pointCloudLogger.log(timestamp/1000, markers);  // point cloud log format: infinite repetitions of:  timestamp (milliseconds) : uint32
      // std::cout << "0000000000000before log" << std::endl;
      pointCloudLogger.log(markers);
    }


    // run tracker
    markers->clear();
    for (long int i = 0; i < pointcloud.rows(); ++i)
    {
      const auto &point = pointcloud.row(i);
      markers->push_back(pcl::PointXYZ(point(0), point(1), point(2)));
    }
    tracker.update(markers);

    transforms.clear();
    transforms.reserve(mocap->rigidBodies().size());
    for (const auto &iter : mocap->rigidBodies())
    {
      const auto& rigidBody = iter.second;

      // const auto& transform = rigidBody.transformation();
      // transforms.emplace_back(eigenToTransform(transform));
      transforms.resize(transforms.size() + 1);
      transforms.back().header.stamp = time;
      transforms.back().header.frame_id = "world";
      transforms.back().child_frame_id = rigidBody.name();
      transforms.back().transform.translation.x = rigidBody.position().x();
      transforms.back().transform.translation.y = rigidBody.position().y();
      transforms.back().transform.translation.z = rigidBody.position().z();
      transforms.back().transform.rotation.x = rigidBody.rotation().x();
      transforms.back().transform.rotation.y = rigidBody.rotation().y();
      transforms.back().transform.rotation.z = rigidBody.rotation().z();
      transforms.back().transform.rotation.w = rigidBody.rotation().w();
    }

    for (const auto& rigidBody : tracker.rigidBodies())
    {
      if (rigidBody.lastTransformationValid())
      {
        const Eigen::Affine3f &transform = rigidBody.transformation();
        Eigen::Quaternionf q(transform.rotation());
        const auto &translation = transform.translation();

        transforms.resize(transforms.size() + 1);
        transforms.back().header.stamp = time;
        transforms.back().header.frame_id = "world";
        transforms.back().child_frame_id = rigidBody.name();
        transforms.back().transform.translation.x = translation.x();
        transforms.back().transform.translation.y = translation.y();
        transforms.back().transform.translation.z = translation.z();
        if (rigidBody.orientationAvailable()) {
          transforms.back().transform.rotation.x = q.x();
          transforms.back().transform.rotation.y = q.y();
          transforms.back().transform.rotation.z = q.z();
          transforms.back().transform.rotation.w = q.w();
        } else {
          transforms.back().transform.rotation.x = std::nan("");
          transforms.back().transform.rotation.y = std::nan("");
          transforms.back().transform.rotation.z = std::nan("");
          transforms.back().transform.rotation.w = std::nan("");
        }
      }
      else
      {
        std::chrono::duration<double> elapsedSeconds = chrono_now - rigidBody.lastValidTime();
        RCLCPP_WARN(node->get_logger(), "No updated pose for %s for %f s.", rigidBody.name().c_str(), elapsedSeconds.count());
      }
    }

    if (transforms.size() > 0) {
      // publish poses
      msgPointCloud.header.stamp = time;
      msgPoses.poses.resize(transforms.size());
      for (size_t i = 0; i < transforms.size(); ++i) {
        msgPoses.poses[i].name = transforms[i].child_frame_id;
        msgPoses.poses[i].pose.position.x = transforms[i].transform.translation.x;
        msgPoses.poses[i].pose.position.y = transforms[i].transform.translation.y;
        msgPoses.poses[i].pose.position.z = transforms[i].transform.translation.z;
        msgPoses.poses[i].pose.orientation = transforms[i].transform.rotation;
      }
      pubPoses->publish(msgPoses);

      // send TF. Since RViz and others can't handle nan's, report a fake oriention if needed
      for (auto& tf : transforms) {
        if (std::isnan(tf.transform.rotation.x)) {
          tf.transform.rotation.x = 0;
          tf.transform.rotation.y = 0;
          tf.transform.rotation.z = 0;
          tf.transform.rotation.w = 1;
        }
      }

      tfbroadcaster.sendTransform(transforms);
    }
    if (logClouds) {
      pointCloudLogger.flush();
    }
    rclcpp::spin_some(node);
  }

  if (logClouds) {
    pointCloudLogger.flush();
  }

  return 0;
  }
