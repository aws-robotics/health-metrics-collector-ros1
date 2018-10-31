/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <health_metric_collector/collect_and_publish.h>
#include <health_metric_collector/cpu_metric_collector.h>
#include <health_metric_collector/metric_collector.h>
#include <health_metric_collector/metric_manager.h>
#include <health_metric_collector/sys_info_collector.h>
#include <ros/ros.h>
#include <ros_monitoring_msgs/MetricList.h>

#include <vector>

#define DEFAULT_INTERVAL_SEC 5
#define TOPIC_BUFFER_SIZE 1000
#define INTERVAL_PARAM_NAME "interval"
#define ROBOT_ID_DIMENSION "robot_id"
#define CATEGORY_DIMENSION "category"
#define HEALTH_CATEGORY "RobotHealth"
#define DEFAULT_ROBOT_ID "Default_Robot"
#define DEFAULT_NODE_NAME "health_metric_collector"
#define INTERVAL_PARAM_NAME "interval"
#define METRICS_TOPIC_NAME "metrics"

using namespace ros_monitoring_msgs;

int main(int argc, char ** argv)
{
  ros::init(argc, argv, DEFAULT_NODE_NAME);

  auto param_reader = std::make_shared<Aws::Client::Ros1NodeParameterReader>();

  // get interval param
  double interval = DEFAULT_INTERVAL_SEC;
  param_reader->ReadDouble(INTERVAL_PARAM_NAME, interval);

  // get robot id
  std::string robot_id = DEFAULT_ROBOT_ID;
  param_reader->ReadStdString(ROBOT_ID_DIMENSION, robot_id);

  // advertise
  ros::NodeHandle public_nh;
  ros::Publisher publisher =
    public_nh.advertise<ros_monitoring_msgs::MetricList>(METRICS_TOPIC_NAME, TOPIC_BUFFER_SIZE);
  AWS_LOG_INFO(__func__, "Starting Health Metric Collector Node...");
  MetricManager mg(publisher);
  mg.AddDimension(ROBOT_ID_DIMENSION, robot_id);
  mg.AddDimension(CATEGORY_DIMENSION, HEALTH_CATEGORY);

  std::vector<MetricCollectorInterface *> collectors;
  CPUMetricCollector cpu_collector(mg);
  collectors.push_back(&cpu_collector);

  SysInfoCollector sys_collector(mg);
  collectors.push_back(&sys_collector);

  // start metrics collection
  CollectAndPublish f(mg, collectors);
  ros::NodeHandle nh("~");
  ros::Timer timer = nh.createTimer(ros::Duration(interval), f);

  ros::spin();
  AWS_LOG_INFO(__func__, "Shutting down Health Metric Collector Node...");
  return 0;
}
