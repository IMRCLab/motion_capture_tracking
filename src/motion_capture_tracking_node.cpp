#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>

// Motion Capture
#ifdef ENABLE_VICON
#include <libmotioncapture/vicon.h>
#endif
#ifdef ENABLE_OPTITRACK
#include <libmotioncapture/optitrack.h>
#endif
#ifdef ENABLE_PHASESPACE
#include <libmotioncapture/phasespace.h>
#endif
#ifdef ENABLE_QUALISYS
#include <libmotioncapture/qualisys.h>
#endif
#ifdef ENABLE_VRPN
#include <libmotioncapture/vrpn.h>
#endif

// Object tracker
#include <libobjecttracker/object_tracker.h>
#include <libobjecttracker/cloudlog.hpp>

void logWarn(const std::string& msg)
{
  ROS_WARN("%s", msg.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_capture_tracking_node");

  ros::NodeHandle nl("~");

  std::string motionCaptureType;
  nl.param<std::string>("motion_capture_type", motionCaptureType, "vicon");

  // Make a new client
  libmotioncapture::MotionCapture* mocap = nullptr;
  if (false)
  {
  }
#ifdef ENABLE_VICON
  else if (motionCaptureType == "vicon")
  {
    std::string hostName;
    nl.getParam("vicon_host_name", hostName);
    mocap = new libmotioncapture::MotionCaptureVicon(hostName,
      /*enableObjects*/ true,
      /*enablePointcloud*/ true);
  }
#endif
#ifdef ENABLE_OPTITRACK
  else if (motionCaptureType == "optitrack")
  {
    std::string hostName;
    nl.getParam("optitrack_host_name", hostName);
    mocap = new libmotioncapture::MotionCaptureOptitrack(hostName);
  }
#endif
#ifdef ENABLE_PHASESPACE
  else if (motionCaptureType == "phasespace")
  {
    std::string ip;
    int numMarkers;
    nl.getParam("phasespace_ip", ip);
    nl.getParam("phasespace_num_markers", numMarkers);
    std::map<size_t, std::pair<int, int> > cfs;
    cfs[231] = std::make_pair<int, int>(10, 11);
    mocap = new libmotioncapture::MotionCapturePhasespace(ip, numMarkers, cfs);
  }
#endif
#ifdef ENABLE_QUALISYS
  else if (motionCaptureType == "qualisys")
  {
    std::string hostname;
    int port;
    nl.getParam("qualisys_host_name", hostname);
    nl.getParam("qualisys_base_port", port);
    mocap = new libmotioncapture::MotionCaptureQualisys(hostname, port,
      /*enableObjects*/ true,
      /*enablePointcloud*/ true);
  }
#endif
#ifdef ENABLE_VRPN
  else if (motionCaptureType == "vrpn")
  {
    std::string hostname;
    int port;
    nl.getParam("vrpn_host_name", hostname);
    mocap = new libmotioncapture::MotionCaptureVrpn(hostname);
  }
#endif
  else {
    throw std::runtime_error("Unknown motion capture type!");
  }

  // prepare point cloud publisher
  ros::Publisher pubPointCloud = nl.advertise<sensor_msgs::PointCloud>("pointCloud", 1);
  sensor_msgs::PointCloud msgPointCloud;
  msgPointCloud.header.seq = 0;
  msgPointCloud.header.frame_id = "world";

  std::string save_point_clouds_path;
  nl.param<std::string>("save_point_clouds_path", save_point_clouds_path, "");
  libobjecttracker::PointCloudLogger pointCloudLogger(save_point_clouds_path);
  const bool logClouds = !save_point_clouds_path.empty();

  pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);

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

  // prepare TF broadcaster
  tf::TransformBroadcaster tfbroadcaster;

  for (size_t frameId = 0; ros::ok(); ++frameId) {

    // Get a frame
    mocap->waitForNextFrame();
    uint64_t timestamp = mocap->getTimeStamp();
    std::cout << "frame " << frameId << ":" << timestamp << std::endl;

    mocap->getPointCloud(markers);

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

    for (const auto& object : tracker.objects()) {

      if (object.lastTransformationValid()) {
        const auto& transform = object.transformation();

        Eigen::Quaternionf q(transform.rotation());
        const auto& translation = transform.translation();

        tf::Transform tftransform;
        tftransform.setOrigin(tf::Vector3(translation.x(), translation.y(), translation.z()));
        tftransform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        tfbroadcaster.sendTransform(tf::StampedTransform(tftransform, ros::Time::now(), "world", object.name()));
      }
    }



    // std::cout << "    points:" << std::endl;

    // for (size_t i = 0; i < markers->size(); ++i) {
    //   const pcl::PointXYZ& point = markers->at(i);
    //   std::cout << "      \"" << i << "\": [" << point.x << "," << point.y << "," << point.z << "]" << std::endl;
    // }

    // if (mocap->supportsObjectTracking()) {
    //   mocap->getObjects(objects);

    //   std::cout << "    objects:" << std::endl;

    //   for (auto const& object: objects) {
    //     std::cout << "      \"" << object.name() << "\":" << std::endl;
    //     std::cout << "         occluded: " << object.occluded() << std::endl;

    //     if (object.occluded() == false) {
    //       Eigen::Vector3f position = object.position();
    //       Eigen::Quaternionf rotation = object.rotation();
    //       std::cout << "         position: [" << position(0) << ", " << position(1) << ", " << position(2) << "]" << std::endl;
    //       std::cout << "         rotation: [" << rotation.w() << ", " << rotation.vec()(0) << ", "
    //                                           << rotation.vec()(1) << ", " << rotation.vec()(2) << "]" << std::endl;
    //     }
    //   }
    // }


    ros::spinOnce();
  }

  if (logClouds) {
    pointCloudLogger.flush();
  }

  return 0;
}
