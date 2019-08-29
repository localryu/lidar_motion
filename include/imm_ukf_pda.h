/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OBJECT_TRACKING_IMM_UKF_JPDAF_H
#define OBJECT_TRACKING_IMM_UKF_JPDAF_H


#include <vector>
#include <chrono>
#include <stdio.h>


#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>


#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include "ukf.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/common/distances.h>

typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

class ImmUkfPda
{
private:
  char buf[256];

  int target_id_;
  bool init_;
  double timestamp_;
  //ryu
  //int is_init_buf;
  //std::vector<double> input_buf_x;
  //std::vector<double> input_buf_y;
  std::vector<UKF> targets_;

  // probabilistic data association params
  double gating_thres_;
  double gate_probability_;
  double detection_probability_;

  // object association param
  int life_time_thres_;

  // static classification param
  double static_velocity_thres_;
  int static_num_history_thres_;

  // switch sukf and ImmUkfPda
  bool use_sukf_;
  bool has_subscribed_vectormap_;

  // whether if benchmarking tracking result
  bool is_benchmark_;
  int frame_count_;
  std::string kitti_data_dir_;

  // for benchmark
  std::string result_file_path_;

  // prevent explode param for ukf
  double prevent_explosion_thres_;
  
  double merge_distance_threshold_;
  const double CENTROID_DISTANCE = 0.2;//distance to consider centroids the same

  std::string input_topic_;
  std::string output_topic_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_detected_array_;
  ros::Publisher pub_object_array_;

  ros::Publisher pub_object_center_;

  std_msgs::Header input_header_;

  void callback(const autoware_msgs::DetectedObjectArray& input);

  void measurementValidation(const autoware_msgs::DetectedObjectArray& input, UKF& target, const bool second_init,
                             const Eigen::VectorXd& max_det_z, const Eigen::MatrixXd& max_det_s,
                             std::vector<autoware_msgs::DetectedObject>& object_vec, std::vector<bool>& matching_vec);

  void updateBehaviorState(const UKF& target, autoware_msgs::DetectedObject& object);

  void initTracker(const autoware_msgs::DetectedObjectArray& input, double timestamp);
  void secondInit(UKF& target, const std::vector<autoware_msgs::DetectedObject>& object_vec, double dt);

  void updateTrackingNum(const std::vector<autoware_msgs::DetectedObject>& object_vec, UKF& target);

  bool probabilisticDataAssociation(const autoware_msgs::DetectedObjectArray& input, const double dt,
                                    std::vector<bool>& matching_vec,
                                    std::vector<autoware_msgs::DetectedObject>& object_vec, UKF& target);
  void makeNewTargets(const double timestamp, const autoware_msgs::DetectedObjectArray& input, const std::vector<bool>& matching_vec);

  void staticClassification();

  void makeOutput(const autoware_msgs::DetectedObjectArray& input,
                  const std::vector<bool>& matching_vec,
                  autoware_msgs::DetectedObjectArray& detected_objects_output);

  void removeUnnecessaryTarget();

  void dumpResultText(autoware_msgs::DetectedObjectArray& detected_objects);

  void tracker(const autoware_msgs::DetectedObjectArray& transformed_input,
               autoware_msgs::DetectedObjectArray& detected_objects_output);

  autoware_msgs::DetectedObjectArray
  removeRedundantObjects(const autoware_msgs::DetectedObjectArray& in_detected_objects,
                         const std::vector<size_t> in_tracker_indices);

  

  bool
  arePointsClose(const geometry_msgs::Point& in_point_a,
                 const geometry_msgs::Point& in_point_b,
                 float in_radius);

  bool
  arePointsEqual(const geometry_msgs::Point& in_point_a,
                 const geometry_msgs::Point& in_point_b);

  bool
  isPointInPool(const std::vector<geometry_msgs::Point>& in_pool,
                const geometry_msgs::Point& in_point);

  void updateTargetWithAssociatedObject(const std::vector<autoware_msgs::DetectedObject>& object_vec,
                                        UKF& target);

public:
  ImmUkfPda();
  void run();
};

#endif /* OBJECT_TRACKING_IMM_UKF_JPDAF_H */
