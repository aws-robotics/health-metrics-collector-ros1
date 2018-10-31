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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <health_metric_collector/collect_and_publish.h>
#include <health_metric_collector/cpu_metric_collector.h>
#include <health_metric_collector/metric_collector.h>
#include <health_metric_collector/metric_manager.h>
#include <health_metric_collector/sys_info_collector.h>
#include <ros/ros.h>

#include <vector>

namespace ros_monitoring_msgs {
class MockMetricManager : public MetricManagerInterface
{
public:
  MOCK_METHOD0(Publish, void());

  MOCK_METHOD2(AddDimension, void(const std::string &, const std::string &));

  MOCK_CONST_METHOD0(CreateMetric, MetricData());

  MOCK_METHOD1(AddMetric, void(MetricData));
};

class MockMetricCollector : public MetricCollectorInterface
{
public:
  MockMetricCollector(MetricManagerInterface & m) : MetricCollectorInterface(m) {}
  MOCK_METHOD0(Collect, void());
};
}  // namespace ros_monitoring_msgs

using ::testing::Return;

TEST(CollectorSuite, Child)
{
  ros_monitoring_msgs::MockMetricManager mg;

  std::vector<MetricCollectorInterface *> collectors;
  MockMetricCollector mc(mg);
  collectors.push_back(&mc);

  // start metrics collection
  CollectAndPublish f(mg, collectors);

  MetricData md;
  ON_CALL(mg, CreateMetric()).WillByDefault(Return(md));
  EXPECT_CALL(mg, Publish()).Times(1);
  EXPECT_CALL(mc, Collect()).Times(1);

  ros::TimerEvent event;
  f(event);
}

TEST(CollectorSuite, sysinfo)
{
  ros_monitoring_msgs::MockMetricManager mg;
  MetricData md;
  ON_CALL(mg, CreateMetric()).WillByDefault(Return(md));

  EXPECT_CALL(mg, AddMetric(testing::_)).Times(4);
  SysInfoCollector sys_collector(mg);
  sys_collector.Collect();
}

TEST(CollectorSuite, cpu_usage_0)
{
  ros_monitoring_msgs::MockMetricManager mg;
  MetricData md;
  ON_CALL(mg, CreateMetric()).WillByDefault(Return(md));

  EXPECT_CALL(mg, AddMetric(testing::_)).Times(0);

  CPUMetricCollector cpu_collector(mg);
  cpu_collector.Collect();
}

TEST(CollectorSuite, cpu_usage_1)
{
  ros_monitoring_msgs::MockMetricManager mg;
  MetricData md;
  ON_CALL(mg, CreateMetric()).WillByDefault(Return(md));

  EXPECT_CALL(mg, AddMetric(testing::_)).Times(testing::AtLeast(1));

  CPUMetricCollector cpu_collector(mg);
  cpu_collector.Collect();
  cpu_collector.Collect();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_collector");

  return RUN_ALL_TESTS();
}
