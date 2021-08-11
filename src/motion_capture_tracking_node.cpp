#include <iostream>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Motion Capture
#include <libmotioncapture/motioncapture.h>

// Object tracker
#include <libobjecttracker/object_tracker.h>
#include <libobjecttracker/cloudlog.hpp>

#if 0
void logWarn(const std::string& msg)
{
  ROS_WARN("%s", msg.c_str());
}
#endif

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motion_capture_tracking_node");
  node->declare_parameter<std::string>("motion_capture_type", "vicon");
  node->declare_parameter<std::string>("motion_capture_hostname", "localhost");

  std::string motionCaptureType = node->get_parameter("motion_capture_type").as_string();
  std::string motionCaptureHostname = node->get_parameter("motion_capture_hostname").as_string();

  // Make a new client
  libmotioncapture::MotionCapture *mocap = libmotioncapture::MotionCapture::connect(motionCaptureType, motionCaptureHostname);

  // prepare point cloud publisher
  auto pubPointCloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("pointCloud", 1);

#if 0
  sensor_msgs::PointCloud msgPointCloud;
  msgPointCloud.header.seq = 0;
  msgPointCloud.header.frame_id = "world";

  std::string save_point_clouds_path;
  nl.param<std::string>("save_point_clouds_path", save_point_clouds_path, "");
  libobjecttracker::PointCloudLogger pointCloudLogger(save_point_clouds_path);
  const bool logClouds = !save_point_clouds_path.empty();

  // prepare object tracker

  std::vector<libobjecttracker::DynamicsConfiguration> dynamicsConfigurations;

  int numConfigurations;
  nl.getParam("numDynamicsConfigurations", numConfigurations);
  dynamicsConfigurations.resize(numConfigurations);
  for (int i = 0; i < numConfigurations; ++i) {
    std::stringstream sstr;
    sstr << "dynamicsConfigurations/" << i;
    nl.getParam(sstr.str() + "/maxXVelocity", dynamicsConfigurations[i].maxXVelocity);
    nl.getParam(sstr.str() + "/maxYVelocity", dynamicsConfigurations[i].maxYVelocity);
    nl.getParam(sstr.str() + "/maxZVelocity", dynamicsConfigurations[i].maxZVelocity);
    nl.getParam(sstr.str() + "/maxPitchRate", dynamicsConfigurations[i].maxPitchRate);
    nl.getParam(sstr.str() + "/maxRollRate", dynamicsConfigurations[i].maxRollRate);
    nl.getParam(sstr.str() + "/maxYawRate", dynamicsConfigurations[i].maxYawRate);
    nl.getParam(sstr.str() + "/maxRoll", dynamicsConfigurations[i].maxRoll);
    nl.getParam(sstr.str() + "/maxPitch", dynamicsConfigurations[i].maxPitch);
    nl.getParam(sstr.str() + "/maxFitnessScore", dynamicsConfigurations[i].maxFitnessScore);
  }

  std::vector<libobjecttracker::MarkerConfiguration> markerConfigurations;
  nl.getParam("numMarkerConfigurations", numConfigurations);
  for (int i = 0; i < numConfigurations; ++i) {
    markerConfigurations.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
    std::stringstream sstr;
    sstr << "markerConfigurations/" << i << "/numPoints";
    int numPoints;
    nl.getParam(sstr.str(), numPoints);

    std::vector<double> offset;
    std::stringstream sstr2;
    sstr2 << "markerConfigurations/" << i << "/offset";
    nl.getParam(sstr2.str(), offset);
    for (int j = 0; j < numPoints; ++j) {
      std::stringstream sstr3;
      sstr3 << "markerConfigurations/" << i << "/points/" << j;
      std::vector<double> points;
      nl.getParam(sstr3.str(), points);
      markerConfigurations.back()->push_back(pcl::PointXYZ(points[0] + offset[0], points[1] + offset[1], points[2] + offset[2]));
    }
  }

  std::vector<libobjecttracker::Object> objects;
  XmlRpc::XmlRpcValue yamlObjects;
  nl.getParam("objects", yamlObjects);
  ROS_ASSERT(yamlObjects.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < yamlObjects.size(); ++i) {
    ROS_ASSERT(yamlObjects[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue yamlObject = yamlObjects[i];
    std::string name = yamlObject["name"];
    XmlRpc::XmlRpcValue yamlPos = yamlObject["initialPosition"];
    ROS_ASSERT(yamlPos.getType() == XmlRpc::XmlRpcValue::TypeArray);

    std::vector<double> posVec(3);
    for (int32_t j = 0; j < yamlPos.size(); ++j) {
      ROS_ASSERT(yamlPos[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      double f = static_cast<double>(yamlPos[j]);
      posVec[j] = f;
    }
    Eigen::Affine3f m;
    m = Eigen::Translation3f(posVec[0], posVec[1], posVec[2]);
    int markerConfigurationIdx = yamlObject["markerConfiguration"];
    int dynamicsConfigurationIdx = yamlObject["dynamicsConfiguration"];

    objects.push_back(libobjecttracker::Object(markerConfigurationIdx, dynamicsConfigurationIdx, m, name));
  }

  libobjecttracker::ObjectTracker tracker(
      dynamicsConfigurations,
      markerConfigurations,
      objects);
  tracker.setLogWarningCallback(logWarn);
#endif
  // prepare TF broadcaster
  tf2_ros::TransformBroadcaster tfbroadcaster(node);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  for (size_t frameId = 0; rclcpp::ok(); ++frameId) {

    // Get a frame
    mocap->waitForNextFrame();

#if 0
    auto markers = mocap->pointCloud();

    // publish as pointcloud
    msgPointCloud.header.seq += 1;
    msgPointCloud.header.stamp = ros::Time::now();
    msgPointCloud.points.resize(markers->size());
    for (size_t i = 0; i < markers->size(); ++i) {
      const pcl::PointXYZ& point = markers->at(i);
      msgPointCloud.points[i].x = point.x;
      msgPointCloud.points[i].y = point.y;
      msgPointCloud.points[i].z = point.z;
    }
    pubPointCloud.publish(msgPointCloud);
    
    if (logClouds) {
      pointCloudLogger.log(timestamp/1000, markers);
    }

    // run tracker
    tracker.update(markers);
#endif

    transforms.clear();
    transforms.reserve(mocap->rigidBodies().size());
    auto time = node->now();
    for (const auto &iter : mocap->rigidBodies())
    {
      const auto& rigidBody = iter.second;

      if (!rigidBody.occluded()) {
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
    }
    if (transforms.size() > 0) {
      tfbroadcaster.sendTransform(transforms);
    }

    rclcpp::spin_some(node);
  }
#if 0
  if (logClouds) {
    pointCloudLogger.flush();
  }
#endif
  return 0;
}

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }