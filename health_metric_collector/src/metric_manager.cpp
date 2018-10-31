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

#include <health_metric_collector/metric_manager.h>
#include <ros_monitoring_msgs/MetricDimension.h>

MetricData ros_monitoring_msgs::MetricManager::CreateMetric() const
{
  MetricData md;
  md.header.stamp = ros::Time(ros::WallTime::now().toSec());
  md.time_stamp = md.header.stamp;
  md.dimensions = dimensions_.dimensions;
  return md;
}

void ros_monitoring_msgs::MetricManager::AddDimension(const std::string & name,
                                                      const std::string & value)
{
  ros_monitoring_msgs::MetricDimension dim;
  dim.name = name;
  dim.value = value;
  dimensions_.dimensions.push_back(dim);
}

void ros_monitoring_msgs::MetricManager::AddMetric(MetricData md) { mlist_.metrics.push_back(md); }

void ros_monitoring_msgs::MetricManager::Publish()
{
  publisher_.publish(mlist_);
  mlist_.metrics.clear();
}
