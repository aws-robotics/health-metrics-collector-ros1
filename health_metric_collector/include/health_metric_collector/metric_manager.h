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

#pragma once

#include <ros/ros.h>
#include <ros_monitoring_msgs/MetricData.h>
#include <ros_monitoring_msgs/MetricList.h>

#include <vector>

using namespace ros_monitoring_msgs;

namespace ros_monitoring_msgs {

/**
 * @brief Interface for MetricManager.
 */
class MetricManagerInterface
{
public:
  /**
   * @brief add global dimension (applies to all metrics).
   */
  virtual void AddDimension(const std::string & name, const std::string & value) = 0;

  /**
   * @brief create a metric.
   */
  virtual MetricData CreateMetric() const = 0;

  /**
   * @brief add a metric to list of metrics to be published.
   *
   * @param md a metric.
   */
  virtual void AddMetric(MetricData md) = 0;

  /**
   * @brief publishes all metrics and then discards them.
   */
  virtual void Publish() = 0;

  /** @brief destructor. */
  virtual ~MetricManagerInterface() {}
};

/**
 * @brief Create, aggregate and publish metrics to ros topic.
 **/
class MetricManager : public MetricManagerInterface
{
public:
  MetricManager(ros::Publisher & p) : publisher_(p) {}

  virtual void AddDimension(const std::string & name, const std::string & value) override final;

  virtual MetricData CreateMetric() const override final;

  virtual void AddMetric(MetricData md) override final;

  virtual void Publish() override final;

private:
  ros::Publisher & publisher_;
  MetricList mlist_;
  ros_monitoring_msgs::MetricData dimensions_;
};
}  // namespace ros_monitoring_msgs
