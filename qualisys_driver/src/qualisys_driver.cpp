// Copyright 2020 National Institute of Advanced Industrial Science and Technology, Japan
// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Floris Erich <floris.erich@aist.go.jp>,
//         David Vargas Frutos <david.vargas@urjc.es>
//         José Miguel Guerrero Hernández <josemiguel.guerrero@urjc.es>
//
// Also includes code fragments from Kumar Robotics ROS 1 Qualisys driver

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <utility>
#include "qualisys_driver/qualisys_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;


struct Quaternion {
    float w, x, y, z;
};

Quaternion matrixToQuaternion(float* matrix) {
    Quaternion quaternion;

    float trace = matrix[0] + matrix[4] + matrix[8];
    if (trace > 0) {
        float s = 0.5f / std::sqrt(trace + 1.0f);
        quaternion.w = 0.25f / s;
        quaternion.x = (matrix[5] - matrix[7]) * s;
        quaternion.y = (matrix[6] - matrix[2]) * s;
        quaternion.z = (matrix[1] - matrix[3]) * s;
    } else {
        if (matrix[0] > matrix[4] && matrix[0] > matrix[8]) {
            float s = 2.0f * std::sqrt(1.0f + matrix[0] - matrix[4] - matrix[8]);
            quaternion.w = (matrix[5] - matrix[7]) / s;
            quaternion.x = 0.25f * s;
            quaternion.y = (matrix[3] + matrix[1]) / s;
            quaternion.z = (matrix[6] + matrix[2]) / s;
        } else if (matrix[4] > matrix[8]) {
            float s = 2.0f * std::sqrt(1.0f + matrix[4] - matrix[0] - matrix[8]);
            quaternion.w = (matrix[6] - matrix[2]) / s;
            quaternion.x = (matrix[3] + matrix[1]) / s;
            quaternion.y = 0.25f * s;
            quaternion.z = (matrix[7] + matrix[5]) / s;
        } else {
            float s = 2.0f * std::sqrt(1.0f + matrix[8] - matrix[0] - matrix[4]);
            quaternion.w = (matrix[1] - matrix[3]) / s;
            quaternion.x = (matrix[6] + matrix[2]) / s;
            quaternion.y = (matrix[7] + matrix[5]) / s;
            quaternion.z = 0.25f * s;
        }
    }

    return quaternion;
}

void QualisysDriver::set_settings_qualisys()
{
}

void QualisysDriver::loop()
{
  CRTPacket * prt_packet = port_protocol_.GetRTPacket();
  CRTPacket::EPacketType e_type;
  if (port_protocol_.Receive(e_type, true) == CNetwork::ResponseType::success) {
    switch (e_type) {
      case CRTPacket::PacketError:
        {
          std::string s = "Error when streaming frames: ";
          s += port_protocol_.GetRTPacket()->GetErrorString();
          RCLCPP_ERROR(get_logger(), s.c_str());
          break;
        }
      case CRTPacket::PacketNoMoreData:
        RCLCPP_WARN(get_logger(), "No data received");
        break;
      case CRTPacket::PacketData:
        process_packet(prt_packet);
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown CRTPacket");
    }
  }
}

void QualisysDriver::process_packet(CRTPacket * const packet)
{
  unsigned int marker_count = packet->Get3DMarkerCount();
  unsigned int rb_count = packet->Get6DOFBodyCount();
  int frame_number = packet->GetFrameNumber();

  int frame_diff = 0;
  if (last_frame_number_ != 0) {
    frame_diff = frame_number - last_frame_number_;
    frame_count_ += frame_diff;

    if (frame_diff > 1) {
      dropped_frame_count_ += frame_diff;
      double dropped_frame_pct = (frame_count_ > 0) ?
        static_cast<double>(dropped_frame_count_) / frame_count_ * 100.0 : 0.0;

      RCLCPP_DEBUG(
        get_logger(),
        "%d more (total %d / %d, %f %%) frame(s) dropped. Consider adjusting rates",
        frame_diff, dropped_frame_count_, frame_count_, dropped_frame_pct
      );
    }
  }
  last_frame_number_ = frame_number;

  if (!mocap_markers_pub_->is_activated() &&
      !mocap_rigid_bodies_pub_->is_activated() &&
      !mocap_skeleton_pub_->is_activated())
  {
    return;
  }

  if (mocap_markers_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::Markers markers_msg;
    markers_msg.header.frame_id = frame_id_;
    markers_msg.header.stamp = rclcpp::Clock().now();
    markers_msg.frame_number = frame_number;

    for (unsigned int i = 0; i < marker_count; ++i) {
      float x, y, z;
      packet->Get3DMarker((float)i, x, y, z);
      mocap4r2_msgs::msg::Marker this_marker;
      this_marker.marker_index = i;
      this_marker.translation.x = x / 1000;
      this_marker.translation.y = y / 1000;
      this_marker.translation.z = z / 1000;
      if (!std::isnan(this_marker.translation.x) && !std::isnan(this_marker.translation.y) && !std::isnan(this_marker.translation.z)){
        markers_msg.markers.push_back(this_marker);
      }
    }

    mocap_markers_pub_->publish(markers_msg);
  }

  if (mocap_rigid_bodies_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::RigidBodies msg_rb;
    msg_rb.header.frame_id = frame_id_;
    msg_rb.header.stamp = rclcpp::Clock().now();
    msg_rb.frame_number = frame_number;

    for (unsigned int i = 0; i < rb_count; i++) {
      mocap4r2_msgs::msg::RigidBody rb;

      float x, y, z;
      float rot_matrix[9];
      // Get6DOFBody(unsigned int nBodyIndex, float &fX, float &fY, float &fZ, float afRotMatrix[9]);
      packet->Get6DOFBody(i, x, y, z, rot_matrix);
      Quaternion quaternion = matrixToQuaternion(rot_matrix);

      const char* label = port_protocol_.Get6DOFBodyName(i);
  
      rb.rigid_body_name = label;
      rb.pose.position.x = x / 1000;
      rb.pose.position.y = y / 1000;
      rb.pose.position.z = z / 1000;
      rb.pose.orientation.x = quaternion.x;
      rb.pose.orientation.y = quaternion.y;
      rb.pose.orientation.z = quaternion.z;
      rb.pose.orientation.w = quaternion.w;

      msg_rb.rigidbodies.push_back(rb);
    }

    mocap_rigid_bodies_pub_->publish(msg_rb);
  }

  // Process skeleton data (always process for TF, publish message only if subscribers)
  if (enable_skeleton_ && skeleton_data_available_) {
    unsigned int skeleton_count = packet->GetSkeletonCount();

    if (skeleton_count > 0) {
      mocap4r2_msgs::msg::RigidBodies msg_skel;
      msg_skel.header.frame_id = frame_id_;
      msg_skel.header.stamp = rclcpp::Clock().now();
      msg_skel.frame_number = frame_number;

      geometry_msgs::msg::PoseArray pose_array;
      pose_array.header = msg_skel.header;

      for (unsigned int skel_idx = 0; skel_idx < skeleton_count; ++skel_idx) {
        unsigned int segment_count = packet->GetSkeletonSegmentCount(skel_idx);

        for (unsigned int seg_idx = 0; seg_idx < segment_count; ++seg_idx) {
          CRTPacket::SSkeletonSegment segment;
          if (packet->GetSkeletonSegment(skel_idx, seg_idx, segment)) {
            mocap4r2_msgs::msg::RigidBody rb;

            // Look up segment name from settings
            auto key = std::make_pair(skel_idx, segment.id);
            auto it = skeleton_segment_map_.find(key);
            if (it != skeleton_segment_map_.end()) {
              rb.rigid_body_name = it->second.skeleton_name + "/" + it->second.segment_name;
            } else {
              rb.rigid_body_name = "skeleton_" + std::to_string(skel_idx) +
                "/segment_" + std::to_string(segment.id);
            }

            // Position (convert mm to m)
            rb.pose.position.x = segment.positionX / 1000.0;
            rb.pose.position.y = segment.positionY / 1000.0;
            rb.pose.position.z = segment.positionZ / 1000.0;

            // Rotation (already quaternion from SDK)
            rb.pose.orientation.x = segment.rotationX;
            rb.pose.orientation.y = segment.rotationY;
            rb.pose.orientation.z = segment.rotationZ;
            rb.pose.orientation.w = segment.rotationW;

            // Skip invalid segments (NaN check for position and orientation)
            bool valid_position = !std::isnan(rb.pose.position.x) &&
                                  !std::isnan(rb.pose.position.y) &&
                                  !std::isnan(rb.pose.position.z);
            bool valid_orientation = !std::isnan(rb.pose.orientation.x) &&
                                     !std::isnan(rb.pose.orientation.y) &&
                                     !std::isnan(rb.pose.orientation.z) &&
                                     !std::isnan(rb.pose.orientation.w);

            if (valid_position && valid_orientation)
            {
              msg_skel.rigidbodies.push_back(rb);
              pose_array.poses.push_back(rb.pose);

              // Publish TF for each skeleton segment
              geometry_msgs::msg::TransformStamped tf_msg;
              tf_msg.header.stamp = msg_skel.header.stamp;
              tf_msg.header.frame_id = frame_id_;
              tf_msg.child_frame_id = rb.rigid_body_name;
              tf_msg.transform.translation.x = rb.pose.position.x;
              tf_msg.transform.translation.y = rb.pose.position.y;
              tf_msg.transform.translation.z = rb.pose.position.z;
              tf_msg.transform.rotation.x = rb.pose.orientation.x;
              tf_msg.transform.rotation.y = rb.pose.orientation.y;
              tf_msg.transform.rotation.z = rb.pose.orientation.z;
              tf_msg.transform.rotation.w = rb.pose.orientation.w;
              tf_broadcaster_->sendTransform(tf_msg);
            }
          }
        }
      }

      if (mocap_skeleton_pub_->get_subscription_count() > 0) {
        mocap_skeleton_pub_->publish(msg_skel);
      }

      if (skeleton_pose_array_pub_->get_subscription_count() > 0) {
        skeleton_pose_array_pub_->publish(pose_array);
      }
    }
  }
}

bool QualisysDriver::stop_qualisys()
{
  RCLCPP_INFO(get_logger(), "Stopping the Qualisys motion capture");
  port_protocol_.StreamFramesStop();
  port_protocol_.Disconnect();

  return true;
}

QualisysDriver::QualisysDriver(const rclcpp::NodeOptions node_options)
: rclcpp_lifecycle::LifecycleNode("qualisys_driver_node", node_options)
{
  initParameters();
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT QualisysDriver::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  auto rmw_qos_history_policy = name_to_history_policy_map.find(qos_history_policy_);
  auto rmw_qos_reliability_policy = name_to_reliability_policy_map.find(qos_reliability_policy_);
  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      // The history policy determines how messages are saved until taken by
      // the reader.
      // KEEP_ALL saves all messages until they are taken.
      // KEEP_LAST enforces a limit on the number of messages that are saved,
      // specified by the "depth" parameter.
      rmw_qos_history_policy->second,
      // Depth represents how many messages to store in history when the
      // history policy is KEEP_LAST.
      qos_depth_
  ));
  // The reliability policy can be reliable, meaning that the underlying transport layer will try
  // ensure that every message gets received in order, or best effort, meaning that the transport
  // makes no guarantees about the order or reliability of delivery.
  qos.reliability(rmw_qos_reliability_policy->second);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "/qualisys_driver/change_state");

  mocap_markers_pub_ = create_publisher<mocap4r2_msgs::msg::Markers>(
    "/markers", 100);

  mocap_rigid_bodies_pub_ = create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "rigid_bodies", rclcpp::QoS(1000));

  mocap_skeleton_pub_ = create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "skeleton_segments", rclcpp::QoS(1000));

  skeleton_pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
    "skeleton_poses", rclcpp::QoS(100));

  update_pub_ = create_publisher<std_msgs::msg::Empty>(
    "/qualisys_driver/update_notify", qos);

  set_settings_qualisys();

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  update_pub_->on_activate();
  mocap_markers_pub_->on_activate();
  mocap_rigid_bodies_pub_->on_activate();
  mocap_skeleton_pub_->on_activate();
  skeleton_pose_array_pub_->on_activate();
  bool success = connect_qualisys();

  if (success) {
    unsigned int components = CRTProtocol::cComponent3d + CRTProtocol::cComponent6d;
    CRTProtocol::SComponentOptions options;
    if (enable_skeleton_ && skeleton_data_available_) {
      components += CRTProtocol::cComponentSkeleton;
      options.mSkeletonGlobalData = true;
    }
    if (!port_protocol_.StreamFrames(CRTProtocol::RateAllFrames, 0, udp_port_, nullptr,
        components, options))
    {
      RCLCPP_ERROR(get_logger(), "Failed to start streaming frames: '%s'",
        port_protocol_.GetErrorString());
      return CallbackReturnT::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "Streaming started");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&QualisysDriver::loop, this));
    RCLCPP_INFO(get_logger(), "Activated!\n");

    return CallbackReturnT::SUCCESS;
  } else {
    RCLCPP_INFO(get_logger(), "Unable to activate!\n");

    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT QualisysDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  timer_->reset();
  update_pub_->on_deactivate();
  mocap_markers_pub_->on_deactivate();
  mocap_rigid_bodies_pub_->on_deactivate();
  mocap_skeleton_pub_->on_deactivate();
  skeleton_pose_array_pub_->on_deactivate();
  stop_qualisys();
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  update_pub_.reset();
  mocap_markers_pub_.reset();
  mocap_rigid_bodies_pub_.reset();
  mocap_skeleton_pub_.reset();
  skeleton_pose_array_pub_.reset();
  timer_->reset();
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  /* Shut down stuff */
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  return CallbackReturnT::SUCCESS;
}

bool QualisysDriver::connect_qualisys()
{
  RCLCPP_WARN(
    get_logger(),
    "Trying to connect to Qualisys host at %s:%d", host_name_.c_str(), port_);

  // Use protocol version 1.22+ for skeleton support
  if (!port_protocol_.Connect(
      reinterpret_cast<const char *>(host_name_.data()), port_, 0, 1, 22))
  {
    RCLCPP_FATAL(get_logger(), "Connection error");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Connected");

  bool settings_6dof;
  port_protocol_.Read6DOFSettings(settings_6dof);
  if (!settings_6dof) {
    RCLCPP_WARN(get_logger(), "Could not read 6DOF settings");
  }

  // Read skeleton settings if enabled
  skeleton_data_available_ = false;
  if (enable_skeleton_) {
    bool skeleton_available;
    port_protocol_.ReadSkeletonSettings(skeleton_available, true);

    if (skeleton_available) {
      skeleton_data_available_ = true;
      skeleton_segment_map_.clear();
      unsigned int skeleton_count = port_protocol_.GetSkeletonCount();
      RCLCPP_INFO(get_logger(), "Found %d skeleton(s)", skeleton_count);

      for (unsigned int skel_idx = 0; skel_idx < skeleton_count; ++skel_idx) {
        const char * skeleton_name = port_protocol_.GetSkeletonName(skel_idx);
        unsigned int segment_count = port_protocol_.GetSkeletonSegmentCount(skel_idx);
        RCLCPP_INFO(
          get_logger(), "Skeleton '%s' has %d segments",
          skeleton_name, segment_count);

        for (unsigned int seg_idx = 0; seg_idx < segment_count; ++seg_idx) {
          CRTProtocol::SSettingsSkeletonSegment segment;
          if (port_protocol_.GetSkeletonSegment(skel_idx, seg_idx, &segment)) {
            SkeletonSegmentInfo info;
            info.skeleton_name = skeleton_name;
            info.segment_name = segment.name;
            skeleton_segment_map_[std::make_pair(skel_idx, segment.id)] = info;
            RCLCPP_DEBUG(
              get_logger(), "  Segment %d: id=%d name='%s'",
              seg_idx, segment.id, segment.name.c_str());
          }
        }
      }
    } else {
      RCLCPP_WARN(get_logger(), "No skeleton data available");
    }
  }

  // Succeed if either 6DOF or skeleton settings were read
  return settings_6dof || skeleton_data_available_;
}

void QualisysDriver::initParameters()
{
  declare_parameter<std::string>("host_name", "mocap");
  declare_parameter<int>("port", 22222);
  // UDP port for streaming data. 0 = use TCP, any other value = use UDP on that port.
  // Note: UDP may not work in WSL2 due to NAT limitations.
  declare_parameter<int>("udp_port", 0);
  declare_parameter<int>("last_frame_number", 0);
  declare_parameter<int>("frame_count", 0);
  declare_parameter<int>("dropped_frame_count", 0);
  declare_parameter<std::string>("qos_history_policy", "keep_all");
  declare_parameter<std::string>("qos_reliability_policy", "best_effort");
  declare_parameter<int>("qos_depth", 10);
  declare_parameter<bool>("use_markers_with_id", true);
  declare_parameter<int>("publish_rate", 10);
  declare_parameter<std::string>("frame_id", "map");
  declare_parameter<bool>("enable_skeleton", true);

  get_parameter<std::string>("host_name", host_name_);
  get_parameter<int>("port", port_);
  get_parameter<int>("udp_port", udp_port_);
  get_parameter<int>("last_frame_number", last_frame_number_);
  get_parameter<int>("frame_count", frame_count_);
  get_parameter<int>("dropped_frame_count", dropped_frame_count_);
  get_parameter<std::string>("qos_history_policy", qos_history_policy_);
  get_parameter<std::string>("qos_reliability_policy", qos_reliability_policy_);
  get_parameter<int>("qos_depth", qos_depth_);
  get_parameter<bool>("use_markers_with_id", use_markers_with_id_);
  get_parameter<int>("publish_rate", publish_rate_);
  get_parameter<std::string>("frame_id", frame_id_);
  get_parameter<bool>("enable_skeleton", enable_skeleton_);

  RCLCPP_INFO(get_logger(), "Param host_name: %s", host_name_.c_str());
  RCLCPP_INFO(get_logger(), "Param port: %d", port_);
  RCLCPP_INFO(get_logger(), "Param udp_port: %d (0 = TCP, >0 = UDP)", udp_port_);
  RCLCPP_INFO(get_logger(), "Param last_frame_number: %d", last_frame_number_);
  RCLCPP_INFO(get_logger(), "Param frame_count: %d", frame_count_);
  RCLCPP_INFO(get_logger(), "Param dropped_frame_count: %d", dropped_frame_count_);
  RCLCPP_INFO(get_logger(), "Param qos_history_policy: %s", qos_history_policy_.c_str());
  RCLCPP_INFO(get_logger(), "Param qos_reliability_policy: %s", qos_reliability_policy_.c_str());
  RCLCPP_INFO(get_logger(), "Param qos_depth: %d", qos_depth_);
  RCLCPP_INFO(get_logger(), "Param use_markers_with_id: %s", use_markers_with_id_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "Param publish_rate: %d", publish_rate_);
  RCLCPP_INFO(get_logger(), "Param frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "Param enable_skeleton: %s", enable_skeleton_ ? "true" : "false");
}
